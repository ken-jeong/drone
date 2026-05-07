#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "config.h"
#include "protocol.h"
#include "log.h"

typedef enum {
    DR_RANDOM = 0,
    DR_GOTO,
    DR_HOLD,
    DR_TERM
} drone_mode_t;

typedef struct {
    int     id;
    int     sock;

    pthread_mutex_t state_lock;   /* x, y, target, mode 보호 */
    double  x, y;
    double  target_x, target_y;
    drone_mode_t mode;

    pthread_mutex_t send_lock;    /* 소켓 송신 직렬화 */
} drone_t;

static drone_t g;

static double rand_unit(void) { return (double)rand() / (double)RAND_MAX; }

static double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void set_mode(drone_mode_t m) {
    pthread_mutex_lock(&g.state_lock);
    g.mode = m;
    pthread_mutex_unlock(&g.state_lock);
}

static drone_mode_t get_mode(void) {
    pthread_mutex_lock(&g.state_lock);
    drone_mode_t m = g.mode;
    pthread_mutex_unlock(&g.state_lock);
    return m;
}

/* ---------- 송신 헬퍼 ---------- */

static int send_msg_locked(uint8_t type, const void *p, uint16_t len) {
    pthread_mutex_lock(&g.send_lock);
    int r = send_msg(g.sock, type, p, len);
    pthread_mutex_unlock(&g.send_lock);
    return r;
}

static void tx_hello(void) {
    uint8_t buf[4];
    pack_int32(buf, 0, g.id);
    send_msg_locked(MSG_HELLO, buf, 4);
    log_msg("[drone %d] TX HELLO", g.id);
}

static void tx_pos_report(double x, double y) {
    uint8_t buf[12];
    pack_int32(buf, 0, g.id);
    pack_int32(buf, 4, coord_to_wire(x));
    pack_int32(buf, 8, coord_to_wire(y));
    send_msg_locked(MSG_POS_REPORT, buf, 12);
    log_msg("[drone %d] TX POS_REPORT (%.2f, %.2f)", g.id, x, y);
}

static void tx_arrived(double x, double y) {
    uint8_t buf[12];
    pack_int32(buf, 0, g.id);
    pack_int32(buf, 4, coord_to_wire(x));
    pack_int32(buf, 8, coord_to_wire(y));
    send_msg_locked(MSG_ARRIVED, buf, 12);
    log_msg("[drone %d] TX ARRIVED   (%.2f, %.2f)", g.id, x, y);
}

/* ---------- 이동 쓰레드 ---------- */

static void *move_thread(void *arg) {
    (void)arg;

    while (get_mode() != DR_TERM) {
        /* 현재 상태 스냅샷 */
        pthread_mutex_lock(&g.state_lock);
        double x = g.x, y = g.y;
        double tx = g.target_x, ty = g.target_y;
        drone_mode_t mode = g.mode;
        pthread_mutex_unlock(&g.state_lock);

        if (mode == DR_RANDOM) {
            /* 임의 방향으로 소폭 이동 */
            double dx = (rand_unit() - 0.5) * 4.0;
            double dy = (rand_unit() - 0.5) * 4.0;
            x = clampd(x + dx, AREA_X_MIN, AREA_X_MAX);
            y = clampd(y + dy, AREA_Y_MIN, AREA_Y_MAX);

            pthread_mutex_lock(&g.state_lock);
            g.x = x; g.y = y;
            pthread_mutex_unlock(&g.state_lock);

            tx_pos_report(x, y);

        } else if (mode == DR_GOTO) {
            double dx = tx - x;
            double dy = ty - y;
            double d  = sqrt(dx * dx + dy * dy);
            int arrived = 0;

            if (d <= MOVE_STEP_M) {
                x = tx; y = ty;
                arrived = 1;
            } else {
                x += MOVE_STEP_M * dx / d;
                y += MOVE_STEP_M * dy / d;
            }
            x = clampd(x, AREA_X_MIN, AREA_X_MAX);
            y = clampd(y, AREA_Y_MIN, AREA_Y_MAX);

            pthread_mutex_lock(&g.state_lock);
            g.x = x; g.y = y;
            if (arrived && g.mode == DR_GOTO) g.mode = DR_HOLD;
            pthread_mutex_unlock(&g.state_lock);

            tx_pos_report(x, y);
            if (arrived) tx_arrived(x, y);

        } else if (mode == DR_HOLD) {
            tx_pos_report(x, y);
        }

        usleep(POS_REPORT_INTERVAL_MS * 1000);
    }
    return NULL;
}

/* ---------- 수신 쓰레드 ---------- */

static void *recv_thread(void *arg) {
    (void)arg;
    uint8_t  type;
    uint8_t  buf[MAX_PAYLOAD];
    uint16_t len;

    for (;;) {
        int r = recv_msg(g.sock, &type, buf, sizeof(buf), &len);
        if (r != 0) {
            log_msg("[drone %d] connection lost", g.id);
            break;
        }

        if (type == MSG_HELLO_ACK) {
            log_msg("[drone %d] RX HELLO_ACK", g.id);
        } else if (type == MSG_MOVE_CMD && len == 8) {
            double tx = coord_from_wire(unpack_int32(buf, 0));
            double ty = coord_from_wire(unpack_int32(buf, 4));
            pthread_mutex_lock(&g.state_lock);
            g.target_x = tx;
            g.target_y = ty;
            g.mode     = DR_GOTO;
            pthread_mutex_unlock(&g.state_lock);
            log_msg("[drone %d] RX MOVE_CMD  -> (%.2f, %.2f)", g.id, tx, ty);
        } else if (type == MSG_PHASE_ACK && len == 4) {
            int phase = (int)unpack_int32(buf, 0);
            log_msg("[drone %d] RX PHASE_ACK phase=%d", g.id, phase);
        } else if (type == MSG_TERMINATE) {
            log_msg("[drone %d] RX TERMINATE", g.id);
            set_mode(DR_TERM);
            break;
        } else {
            log_msg("[drone %d] RX unknown type=0x%02x len=%u",
                    g.id, type, len);
        }
    }

    set_mode(DR_TERM);
    return NULL;
}

/* ---------- main ---------- */

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <server_ip> <port> <drone_id>\n", argv[0]);
        return 1;
    }
    const char *ip = argv[1];
    int port       = atoi(argv[2]);
    int id         = atoi(argv[3]);

    log_init();
    signal(SIGPIPE, SIG_IGN);
    srand((unsigned)time(NULL) ^ (unsigned)(id * 7919));

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) { perror("socket"); return 1; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) <= 0) {
        fprintf(stderr, "invalid server ip: %s\n", ip);
        return 1;
    }

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("connect");
        return 1;
    }

    /* 초기 상태 */
    g.id   = id;
    g.sock = sock;
    g.x = AREA_X_MIN + rand_unit() * (AREA_X_MAX - AREA_X_MIN);
    g.y = AREA_Y_MIN + rand_unit() * (AREA_Y_MAX - AREA_Y_MIN);
    g.target_x = g.x;
    g.target_y = g.y;
    g.mode = DR_RANDOM;
    pthread_mutex_init(&g.state_lock, NULL);
    pthread_mutex_init(&g.send_lock, NULL);

    log_msg("[drone %d] connected to %s:%d, init pos=(%.2f, %.2f)",
            id, ip, port, g.x, g.y);

    tx_hello();

    pthread_t rtid, mtid;
    pthread_create(&rtid, NULL, recv_thread, NULL);
    pthread_create(&mtid, NULL, move_thread, NULL);

    pthread_join(rtid, NULL);
    pthread_join(mtid, NULL);

    close(sock);
    pthread_mutex_destroy(&g.state_lock);
    pthread_mutex_destroy(&g.send_lock);
    log_msg("[drone %d] terminated", id);
    return 0;
}
