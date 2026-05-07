#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>
#include <math.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "config.h"
#include "protocol.h"
#include "log.h"
#include "drone_table.h"
#include "mission.h"

typedef struct {
    int             listen_fd;
    drone_table_t  *table;
} accept_arg_t;

typedef struct {
    int             sock;
    drone_table_t  *table;
} worker_arg_t;

static volatile int g_shutdown = 0;

static void on_sigint(int sig) { (void)sig; g_shutdown = 1; }

/* ---------- 워커 쓰레드: 한 드론 클라이언트의 수신 루프 ---------- */

static void *worker_thread(void *arg) {
    worker_arg_t *w = (worker_arg_t *)arg;
    int sock = w->sock;
    drone_table_t *table = w->table;
    free(w);

    uint8_t  type;
    uint8_t  buf[MAX_PAYLOAD];
    uint16_t len;

    /* 1) HELLO 수신 */
    int r = recv_msg(sock, &type, buf, sizeof(buf), &len);
    if (r != 0 || type != MSG_HELLO || len != 4) {
        log_msg("[server] bad HELLO from sock=%d", sock);
        close(sock);
        return NULL;
    }
    int my_id = (int)unpack_int32(buf, 0);

    int idx = table_add(table, my_id, sock);
    if (idx < 0) {
        log_msg("[server] table full, rejecting drone %d", my_id);
        close(sock);
        return NULL;
    }

    /* 2) HELLO_ACK 송신 */
    pack_int32(buf, 0, my_id);
    table_send_to(table, my_id, MSG_HELLO_ACK, buf, 4);
    log_msg("[server] drone %d connected (slot=%d)", my_id, idx);

    /* 3) 메시지 수신 루프 */
    for (;;) {
        r = recv_msg(sock, &type, buf, sizeof(buf), &len);
        if (r != 0) {
            log_msg("[server] drone %d disconnected", my_id);
            break;
        }

        if (type == MSG_POS_REPORT && len == 12) {
            double x = coord_from_wire(unpack_int32(buf, 4));
            double y = coord_from_wire(unpack_int32(buf, 8));
            table_update_pos(table, my_id, x, y);
            log_msg("[server] RX POS_REPORT drone=%d (%.2f, %.2f)",
                    my_id, x, y);
        } else if (type == MSG_ARRIVED && len == 12) {
            double x = coord_from_wire(unpack_int32(buf, 4));
            double y = coord_from_wire(unpack_int32(buf, 8));
            table_update_pos(table, my_id, x, y);
            log_msg("[server] RX ARRIVED   drone=%d (%.2f, %.2f)",
                    my_id, x, y);
        } else {
            log_msg("[server] RX unknown type=0x%02x len=%u from drone=%d",
                    type, len, my_id);
        }
    }

    table_remove_by_sock(table, sock);
    return NULL;
}

/* ---------- accept 쓰레드 ---------- */

static void *accept_thread(void *arg) {
    accept_arg_t *a = (accept_arg_t *)arg;
    while (!g_shutdown) {
        struct sockaddr_in cli;
        socklen_t clen = sizeof(cli);
        int csock = accept(a->listen_fd, (struct sockaddr *)&cli, &clen);
        if (csock < 0) {
            if (errno == EINTR) continue;
            break;
        }
        worker_arg_t *w = (worker_arg_t *)malloc(sizeof(*w));
        if (!w) { close(csock); continue; }
        w->sock = csock;
        w->table = a->table;

        pthread_t tid;
        if (pthread_create(&tid, NULL, worker_thread, w) != 0) {
            close(csock);
            free(w);
            continue;
        }
        pthread_detach(tid);
    }
    return NULL;
}

/* ---------- 미션 컨트롤러 헬퍼 ---------- */

static int collect_active_ids(drone_table_t *t, int *ids, int max) {
    drone_entry_t snap[MAX_DRONES];
    int n = table_snapshot(t, snap, max);
    for (int i = 0; i < n; i++) ids[i] = snap[i].id;
    return n;
}

static int find_in_snap(const drone_entry_t *snap, int sn, int id) {
    for (int i = 0; i < sn; i++) {
        if (snap[i].id == id) return i;
    }
    return -1;
}

static int all_arrived(drone_table_t *t,
                       const int *ids, int n,
                       const double *tx, const double *ty) {
    drone_entry_t snap[MAX_DRONES];
    int sn = table_snapshot(t, snap, MAX_DRONES);
    for (int i = 0; i < n; i++) {
        int j = find_in_snap(snap, sn, ids[i]);
        if (j < 0) return 0;
        double dx = snap[j].x - tx[i];
        double dy = snap[j].y - ty[i];
        if (sqrt(dx * dx + dy * dy) > VALIDATION_TOL) return 0;
    }
    return 1;
}

static int phase1_global_ok(drone_table_t *t) {
    drone_entry_t snap[MAX_DRONES];
    int n = table_snapshot(t, snap, MAX_DRONES);
    return check_phase1_complete(snap, n);
}

static void send_move_cmd(drone_table_t *t, int id, double tx, double ty) {
    uint8_t buf[8];
    pack_int32(buf, 0, coord_to_wire(tx));
    pack_int32(buf, 4, coord_to_wire(ty));
    table_send_to(t, id, MSG_MOVE_CMD, buf, 8);
    log_msg("[server] TX MOVE_CMD  drone=%d -> (%.2f, %.2f)", id, tx, ty);
}

static void send_phase_ack(drone_table_t *t, int id, int phase) {
    uint8_t buf[4];
    pack_int32(buf, 0, phase);
    table_send_to(t, id, MSG_PHASE_ACK, buf, 4);
}

static void send_terminate(drone_table_t *t, int id) {
    table_send_to(t, id, MSG_TERMINATE, NULL, 0);
    log_msg("[server] TX TERMINATE drone=%d", id);
}

static void wait_for_arrival(drone_table_t *t,
                             const int *ids, int n,
                             const double *tx, const double *ty) {
    while (!all_arrived(t, ids, n, tx, ty)) {
        usleep(MISSION_POLL_MS * 1000);
    }
}

/* ---------- main ---------- */

int main(int argc, char **argv) {
    int port     = DEFAULT_PORT;
    int expected = DEFAULT_EXPECTED_DRONES;
    if (argc >= 2) port     = atoi(argv[1]);
    if (argc >= 3) expected = atoi(argv[2]);
    if (expected < 3) expected = 3;
    if (expected > MAX_DRONES) expected = MAX_DRONES;

    log_init();
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, on_sigint);

    int lfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lfd < 0) { perror("socket"); return 1; }
    int one = 1;
    setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(lfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }
    if (listen(lfd, 16) < 0) { perror("listen"); return 1; }

    log_msg("[server] listening on port %d, expecting %d drones",
            port, expected);

    drone_table_t table;
    table_init(&table);

    accept_arg_t aarg = { lfd, &table };
    pthread_t atid;
    pthread_create(&atid, NULL, accept_thread, &aarg);

    /* ===== 미션 시퀀스 ===== */

    log_msg("[mission] %s — waiting for %d drones",
            phase_name(MISSION_INIT), expected);
    table_wait_count(&table, expected);
    log_msg("[mission] all %d drones connected", expected);

    int ids[MAX_DRONES];
    int n = collect_active_ids(&table, ids, MAX_DRONES);

    /* ----- 1차 이동 ----- */
    double tx1[MAX_DRONES], ty1[MAX_DRONES];
    compute_phase1_targets(n, tx1, ty1);

    log_msg("[mission] %s — dispatching phase 1 targets",
            phase_name(MISSION_PHASE1_GO));
    for (int i = 0; i < n; i++) {
        send_move_cmd(&table, ids[i], tx1[i], ty1[i]);
    }

    wait_for_arrival(&table, ids, n, tx1, ty1);

    if (!phase1_global_ok(&table)) {
        log_msg("[mission] WARNING: phase1 global check (radius/gap) failed");
    } else {
        log_msg("[mission] phase1 global check OK (radius<=30m, gap>=10m)");
    }
    log_msg("[mission] %s", phase_name(MISSION_PHASE1_OK));
    for (int i = 0; i < n; i++) send_phase_ack(&table, ids[i], 1);

    /* ----- 2차 이동: 좌측 50m ----- */
    double tx2[MAX_DRONES], ty2[MAX_DRONES];
    for (int i = 0; i < n; i++) {
        tx2[i] = tx1[i] - PHASE2_LEFT_SHIFT;
        ty2[i] = ty1[i];
    }

    log_msg("[mission] %s — dispatching phase 2 targets",
            phase_name(MISSION_PHASE2_GO));
    for (int i = 0; i < n; i++) {
        send_move_cmd(&table, ids[i], tx2[i], ty2[i]);
    }

    wait_for_arrival(&table, ids, n, tx2, ty2);
    log_msg("[mission] %s", phase_name(MISSION_PHASE2_OK));
    for (int i = 0; i < n; i++) send_phase_ack(&table, ids[i], 2);

    /* ----- 종료 ----- */
    for (int i = 0; i < n; i++) send_terminate(&table, ids[i]);
    log_msg("[mission] %s", phase_name(MISSION_DONE));

    sleep(1);
    g_shutdown = 1;
    shutdown(lfd, SHUT_RDWR);
    close(lfd);

    /* 워커들이 정리될 시간 부여 후 종료 */
    sleep(1);
    table_destroy(&table);
    return 0;
}
