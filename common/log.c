#include "log.h"
#include "event_bus.h"

#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>

static pthread_mutex_t g_log_lock = PTHREAD_MUTEX_INITIALIZER;
static int             g_quiet    = 0;

void log_init(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);
}

void log_set_quiet(int quiet) { g_quiet = quiet; }

static void emit(ev_kind_t kind, const char *body) {
    if (!g_quiet) {
        time_t t = time(NULL);
        struct tm tm;
        localtime_r(&t, &tm);
        char ts[16];
        strftime(ts, sizeof(ts), "%H:%M:%S", &tm);

        pthread_mutex_lock(&g_log_lock);
        fprintf(stdout, "[%s] %s\n", ts, body);
        pthread_mutex_unlock(&g_log_lock);
    }
    event_bus_push(kind, "%s", body);
}

void log_msg(const char *fmt, ...) {
    char body[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    emit(EV_INFO, body);
}

void log_event(ev_kind_t kind, const char *fmt, ...) {
    char body[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    emit(kind, body);
}
