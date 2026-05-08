#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>
#include <math.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "config.h"
#include "protocol.h"
#include "log.h"
#include "event_bus.h"
#include "drone_table.h"
#include "mission.h"
#include "ui.h"

typedef struct {
    int             listen_fd;
    drone_table_t  *table;
} accept_arg_t;

typedef struct {
    int             sock;
    drone_table_t  *table;
} worker_arg_t;

static volatile int              g_shutdown = 0;
static volatile mission_phase_t  g_phase    = MISSION_INIT;

static void on_sigint(int sig) {
    (void)sig;
    g_shutdown = 1;
    ui_request_quit();
}

static long now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long)tv.tv_sec * 1000L + (long)tv.tv_usec / 1000L;
}

/* ---------- 워커 쓰레드 ---------- */

static void *worker_thread(void *arg) {
    worker_arg_t *w = (worker_arg_t *)arg;
    int sock = w->sock;
    drone_table_t *table = w->table;
    free(w);

    uint8_t  type;
    uint8_t  buf[MAX_PAYLOAD];
    uint16_t len;

    int r = recv_msg(sock, &type, buf, sizeof(buf), &len);
    if (r != 0 || type != MSG_HELLO || len != 4) {
        log_event(EV_WARN, "[server] bad HELLO from sock=%d", sock);
        close(sock);
        return NULL;
    }
    int my_id = (int)unpack_int32(buf, 0);

    int idx = table_add(table, my_id, sock);
    if (idx < 0) {
        log_event(EV_WARN, "[server] table full, rejecting drone %d", my_id);
        close(sock);
        return NULL;
    }

    pack_int32(buf, 0, my_id);
    table_send_to(table, my_id, MSG_HELLO_ACK, buf, 4);
    log_event(EV_TX, "[server] TX HELLO_ACK drone=%d", my_id);
    log_event(EV_INFO, "[server] drone %d connected (slot=%d)", my_id, idx);

    for (;;) {
        r = recv_msg(sock, &type, buf, sizeof(buf), &len);
        if (r != 0) {
            log_event(EV_INFO, "[server] drone %d disconnected", my_id);
            break;
        }

        if (type == MSG_POS_REPORT && len == 12) {
            double x = coord_from_wire(unpack_int32(buf, 4));
            double y = coord_from_wire(unpack_int32(buf, 8));
            table_update_pos(table, my_id, x, y);
            log_event(EV_RX, "POS_REPORT drone=%d (%.2f, %.2f)",
                      my_id, x, y);
        } else if (type == MSG_ARRIVED && len == 12) {
            double x = coord_from_wire(unpack_int32(buf, 4));
            double y = coord_from_wire(unpack_int32(buf, 8));
            table_update_pos(table, my_id, x, y);
            table_set_arrived(table, my_id);
            log_event(EV_RX, "ARRIVED    drone=%d (%.2f, %.2f)",
                      my_id, x, y);
        } else {
            log_event(EV_WARN,
                      "[server] RX unknown type=0x%02x len=%u from drone=%d",
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

/* ---------- 미션 헬퍼 ---------- */

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
    table_set_goto(t, id, tx, ty);
    log_event(EV_TX, "MOVE_CMD   drone=%d -> (%.2f, %.2f)", id, tx, ty);
}

static void send_phase_ack(drone_table_t *t, int id, int phase) {
    uint8_t buf[4];
    pack_int32(buf, 0, phase);
    table_send_to(t, id, MSG_PHASE_ACK, buf, 4);
    log_event(EV_TX, "PHASE_ACK  drone=%d phase=%d", id, phase);
}

static void send_terminate(drone_table_t *t, int id) {
    table_send_to(t, id, MSG_TERMINATE, NULL, 0);
    table_set_terminated(t, id);
    log_event(EV_TX, "TERMINATE  drone=%d", id);
}

static void wait_for_arrival(drone_table_t *t,
                             const int *ids, int n,
                             const double *tx, const double *ty) {
    while (!g_shutdown && !ui_should_quit() &&
           !all_arrived(t, ids, n, tx, ty)) {
        usleep(MISSION_POLL_MS * 1000);
    }
}

static void set_phase(mission_phase_t p) {
    g_phase = p;
    log_event(EV_PHASE, "*** %s ***", phase_name(p));
}

/* ---------- main ---------- */

static void usage(const char *p) {
    fprintf(stderr,
            "usage: %s [port] [expected_drones] [--no-tui]\n"
            "  default: port=%d expected=%d\n",
            p, DEFAULT_PORT, DEFAULT_EXPECTED_DRONES);
}

int main(int argc, char **argv) {
    int port     = DEFAULT_PORT;
    int expected = DEFAULT_EXPECTED_DRONES;
    int use_tui  = 1;

    int positional = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--no-tui") == 0) {
            use_tui = 0;
        } else if (strcmp(argv[i], "-h") == 0 ||
                   strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else if (positional == 0) {
            port = atoi(argv[i]); positional++;
        } else if (positional == 1) {
            expected = atoi(argv[i]); positional++;
        } else {
            usage(argv[0]);
            return 1;
        }
    }
    if (expected < 3) expected = 3;
    if (expected > MAX_DRONES) expected = MAX_DRONES;

    log_init();
    event_bus_init(200);
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

    log_event(EV_INFO, "[server] listening on port %d, expecting %d drones",
              port, expected);

    drone_table_t table;
    table_init(&table);

    accept_arg_t aarg = { lfd, &table };
    pthread_t atid;
    pthread_create(&atid, NULL, accept_thread, &aarg);

    /* TUI 시작 */
    pthread_t ui_tid = 0;
    int ui_running = 0;
    ui_ctx_t uctx = { &table, &g_phase, now_ms(), expected };

    if (use_tui) {
        if (ui_init() == 0) {
            log_set_quiet(1);
            if (pthread_create(&ui_tid, NULL, ui_thread, &uctx) == 0) {
                ui_running = 1;
            } else {
                ui_shutdown();
                log_set_quiet(0);
            }
        } else {
            fprintf(stderr,
                    "ui_init failed, falling back to stdout mode\n");
        }
    }

    /* ===== 미션 시퀀스 ===== */

    set_phase(MISSION_INIT);
    log_event(EV_INFO, "[mission] %s — waiting for %d drones",
              phase_name(MISSION_INIT), expected);
    table_wait_count(&table, expected);
    log_event(EV_INFO, "[mission] all %d drones connected", expected);

    int ids[MAX_DRONES];
    int n = collect_active_ids(&table, ids, MAX_DRONES);

    /* 1차 이동 */
    double tx1[MAX_DRONES], ty1[MAX_DRONES];
    compute_phase1_targets(n, tx1, ty1);

    set_phase(MISSION_PHASE1_GO);
    log_event(EV_INFO, "[mission] dispatching phase 1 targets");
    for (int i = 0; i < n; i++) {
        send_move_cmd(&table, ids[i], tx1[i], ty1[i]);
    }

    wait_for_arrival(&table, ids, n, tx1, ty1);

    if (!g_shutdown && !ui_should_quit()) {
        if (!phase1_global_ok(&table)) {
            log_event(EV_WARN,
                      "[mission] phase1 global check (radius/gap) FAILED");
        } else {
            log_event(EV_INFO,
                      "[mission] phase1 global check OK (r<=30m, gap>=10m)");
        }
        set_phase(MISSION_PHASE1_OK);
        for (int i = 0; i < n; i++) send_phase_ack(&table, ids[i], 1);
    }

    /* 2차 이동 */
    if (!g_shutdown && !ui_should_quit()) {
        double tx2[MAX_DRONES], ty2[MAX_DRONES];
        for (int i = 0; i < n; i++) {
            tx2[i] = tx1[i] - PHASE2_LEFT_SHIFT;
            ty2[i] = ty1[i];
        }

        set_phase(MISSION_PHASE2_GO);
        log_event(EV_INFO, "[mission] dispatching phase 2 targets");
        for (int i = 0; i < n; i++) {
            send_move_cmd(&table, ids[i], tx2[i], ty2[i]);
        }

        wait_for_arrival(&table, ids, n, tx2, ty2);

        if (!g_shutdown && !ui_should_quit()) {
            set_phase(MISSION_PHASE2_OK);
            for (int i = 0; i < n; i++) send_phase_ack(&table, ids[i], 2);
        }
    }

    /* 종료 */
    for (int i = 0; i < n; i++) send_terminate(&table, ids[i]);
    set_phase(MISSION_DONE);

    /* 사용자가 결과 화면을 볼 시간 */
    if (ui_running) {
        for (int i = 0; i < 30 && !ui_should_quit(); i++) {
            usleep(100 * 1000);
        }
        ui_request_quit();
        pthread_join(ui_tid, NULL);
        ui_shutdown();
        log_set_quiet(0);
    } else {
        sleep(1);
    }

    g_shutdown = 1;
    shutdown(lfd, SHUT_RDWR);
    close(lfd);
    sleep(1);
    table_destroy(&table);
    event_bus_destroy();
    return 0;
}
