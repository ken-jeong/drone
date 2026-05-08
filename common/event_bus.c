#include "event_bus.h"

#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

static log_event_t      *g_buf  = NULL;
static int               g_cap  = 0;
static int               g_head = 0;
static int               g_count = 0;
static pthread_mutex_t   g_lock = PTHREAD_MUTEX_INITIALIZER;

static long now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long)tv.tv_sec * 1000L + (long)tv.tv_usec / 1000L;
}

void event_bus_init(int capacity) {
    if (capacity <= 0) capacity = 200;
    pthread_mutex_lock(&g_lock);
    free(g_buf);
    g_buf   = (log_event_t *)calloc((size_t)capacity, sizeof(log_event_t));
    g_cap   = g_buf ? capacity : 0;
    g_head  = 0;
    g_count = 0;
    pthread_mutex_unlock(&g_lock);
}

void event_bus_destroy(void) {
    pthread_mutex_lock(&g_lock);
    free(g_buf);
    g_buf   = NULL;
    g_cap   = 0;
    g_head  = 0;
    g_count = 0;
    pthread_mutex_unlock(&g_lock);
}

void event_bus_push(ev_kind_t kind, const char *fmt, ...) {
    char body[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);

    pthread_mutex_lock(&g_lock);
    if (g_buf && g_cap > 0) {
        log_event_t *e = &g_buf[g_head];
        e->ts_ms = now_ms();
        e->kind  = kind;
        strncpy(e->text, body, sizeof(e->text) - 1);
        e->text[sizeof(e->text) - 1] = '\0';
        g_head = (g_head + 1) % g_cap;
        if (g_count < g_cap) g_count++;
    }
    pthread_mutex_unlock(&g_lock);
}

int event_bus_snapshot(log_event_t *out, int max) {
    pthread_mutex_lock(&g_lock);
    int n = g_count < max ? g_count : max;
    if (n > 0 && g_buf) {
        int start = (g_head - n + g_cap) % g_cap;
        for (int i = 0; i < n; i++) {
            out[i] = g_buf[(start + i) % g_cap];
        }
    }
    pthread_mutex_unlock(&g_lock);
    return n;
}
