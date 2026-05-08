#include "drone_table.h"
#include "protocol.h"

#include <string.h>
#include <unistd.h>
#include <sys/time.h>

static long now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long)tv.tv_sec * 1000L + (long)tv.tv_usec / 1000L;
}

void table_init(drone_table_t *t) {
    memset(t, 0, sizeof(*t));
    pthread_mutex_init(&t->table_lock, NULL);
    pthread_cond_init(&t->state_changed, NULL);
    for (int i = 0; i < MAX_DRONES; i++) {
        pthread_mutex_init(&t->send_locks[i], NULL);
        t->drones[i].socket_fd = -1;
    }
}

void table_destroy(drone_table_t *t) {
    for (int i = 0; i < MAX_DRONES; i++) {
        pthread_mutex_destroy(&t->send_locks[i]);
    }
    pthread_mutex_destroy(&t->table_lock);
    pthread_cond_destroy(&t->state_changed);
}

int table_add(drone_table_t *t, int id, int sock) {
    int idx = -1;
    pthread_mutex_lock(&t->table_lock);
    for (int i = 0; i < MAX_DRONES; i++) {
        if (!t->drones[i].active) {
            drone_entry_t *e = &t->drones[i];
            e->id           = id;
            e->active       = 1;
            e->socket_fd    = sock;
            e->x            = 0.0;
            e->y            = 0.0;
            e->mode         = DRONE_RANDOM;
            e->target_x     = 0.0;
            e->target_y     = 0.0;
            e->last_seen_ms = now_ms();
            e->phase1_done  = 0;
            e->phase2_done  = 0;
            t->count++;
            idx = i;
            break;
        }
    }
    pthread_cond_broadcast(&t->state_changed);
    pthread_mutex_unlock(&t->table_lock);
    return idx;
}

void table_update_pos(drone_table_t *t, int id, double x, double y) {
    pthread_mutex_lock(&t->table_lock);
    for (int i = 0; i < MAX_DRONES; i++) {
        if (t->drones[i].active && t->drones[i].id == id) {
            t->drones[i].x = x;
            t->drones[i].y = y;
            t->drones[i].last_seen_ms = now_ms();
            break;
        }
    }
    pthread_cond_broadcast(&t->state_changed);
    pthread_mutex_unlock(&t->table_lock);
}

void table_set_goto(drone_table_t *t, int id, double tx, double ty) {
    pthread_mutex_lock(&t->table_lock);
    for (int i = 0; i < MAX_DRONES; i++) {
        if (t->drones[i].active && t->drones[i].id == id) {
            t->drones[i].target_x = tx;
            t->drones[i].target_y = ty;
            t->drones[i].mode     = DRONE_GOTO;
            break;
        }
    }
    pthread_cond_broadcast(&t->state_changed);
    pthread_mutex_unlock(&t->table_lock);
}

void table_set_arrived(drone_table_t *t, int id) {
    pthread_mutex_lock(&t->table_lock);
    for (int i = 0; i < MAX_DRONES; i++) {
        if (t->drones[i].active && t->drones[i].id == id) {
            t->drones[i].mode = DRONE_HOLD;
            break;
        }
    }
    pthread_cond_broadcast(&t->state_changed);
    pthread_mutex_unlock(&t->table_lock);
}

void table_set_terminated(drone_table_t *t, int id) {
    pthread_mutex_lock(&t->table_lock);
    for (int i = 0; i < MAX_DRONES; i++) {
        if (t->drones[i].active && t->drones[i].id == id) {
            t->drones[i].mode = DRONE_TERMINATED;
            break;
        }
    }
    pthread_cond_broadcast(&t->state_changed);
    pthread_mutex_unlock(&t->table_lock);
}

void table_remove_by_sock(drone_table_t *t, int sock) {
    int closed_fd = -1;
    pthread_mutex_lock(&t->table_lock);
    for (int i = 0; i < MAX_DRONES; i++) {
        if (t->drones[i].active && t->drones[i].socket_fd == sock) {
            t->drones[i].active = 0;
            closed_fd = t->drones[i].socket_fd;
            t->drones[i].socket_fd = -1;
            t->count--;
            break;
        }
    }
    pthread_cond_broadcast(&t->state_changed);
    pthread_mutex_unlock(&t->table_lock);

    if (closed_fd >= 0) close(closed_fd);
}

int table_snapshot(drone_table_t *t, drone_entry_t *out, int max) {
    int n = 0;
    pthread_mutex_lock(&t->table_lock);
    for (int i = 0; i < MAX_DRONES && n < max; i++) {
        if (t->drones[i].active) {
            out[n++] = t->drones[i];
        }
    }
    pthread_mutex_unlock(&t->table_lock);
    return n;
}

int table_send_to(drone_table_t *t, int id,
                  uint8_t type, const void *payload, uint16_t len) {
    int sock = -1, idx = -1;

    pthread_mutex_lock(&t->table_lock);
    for (int i = 0; i < MAX_DRONES; i++) {
        if (t->drones[i].active && t->drones[i].id == id) {
            sock = t->drones[i].socket_fd;
            idx  = i;
            break;
        }
    }
    pthread_mutex_unlock(&t->table_lock);

    if (idx < 0 || sock < 0) return -1;

    pthread_mutex_lock(&t->send_locks[idx]);
    int r = send_msg(sock, type, payload, len);
    pthread_mutex_unlock(&t->send_locks[idx]);
    return r;
}

void table_wait_count(drone_table_t *t, int expected) {
    pthread_mutex_lock(&t->table_lock);
    while (t->count < expected) {
        pthread_cond_wait(&t->state_changed, &t->table_lock);
    }
    pthread_mutex_unlock(&t->table_lock);
}
