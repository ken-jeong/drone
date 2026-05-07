#include "log.h"

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <pthread.h>

static pthread_mutex_t g_log_lock = PTHREAD_MUTEX_INITIALIZER;

void log_init(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);
}

void log_msg(const char *fmt, ...) {
    char body[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);

    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    char ts[16];
    strftime(ts, sizeof(ts), "%H:%M:%S", &tm);

    pthread_mutex_lock(&g_log_lock);
    fprintf(stdout, "[%s] %s\n", ts, body);
    pthread_mutex_unlock(&g_log_lock);
}
