#include "drone_ui.h"
#include "event_bus.h"

#include <ncurses.h>
#include <locale.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define CP_TX     1
#define CP_RX     2
#define CP_PHASE  3
#define CP_WARN   4
#define CP_INFO   5

#define LOG_BUF_MAX 200

static int          g_initialized = 0;
static volatile int g_quit        = 0;
static int          g_have_color  = 0;
static long         g_clear_ts    = 0;

static long now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long)tv.tv_sec * 1000L + (long)tv.tv_usec / 1000L;
}

int drone_ui_init(void) {
    if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) return -1;
    const char *term = getenv("TERM");
    if (!term || !*term ||
        strcmp(term, "unknown") == 0 || strcmp(term, "dumb") == 0) {
        return -1;
    }
    setlocale(LC_ALL, "");
    if (!initscr()) return -1;
    g_have_color = has_colors();
    if (g_have_color) {
        start_color();
        use_default_colors();
        init_pair(CP_TX,    COLOR_GREEN,  -1);
        init_pair(CP_RX,    COLOR_BLUE,   -1);
        init_pair(CP_PHASE, COLOR_YELLOW, -1);
        init_pair(CP_WARN,  COLOR_RED,    -1);
        init_pair(CP_INFO,  COLOR_WHITE,  -1);
    }
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    g_initialized = 1;
    return 0;
}

void drone_ui_shutdown(void) {
    if (g_initialized) {
        endwin();
        g_initialized = 0;
    }
}

int  drone_ui_should_quit(void) { return g_quit; }
void drone_ui_request_quit(void) { g_quit = 1; }

static int kind_to_pair(ev_kind_t k) {
    switch (k) {
        case EV_TX:    return CP_TX;
        case EV_RX:    return CP_RX;
        case EV_PHASE: return CP_PHASE;
        case EV_WARN:  return CP_WARN;
        case EV_INFO:  return CP_INFO;
    }
    return CP_INFO;
}

static const char *kind_to_tag(ev_kind_t k) {
    switch (k) {
        case EV_TX:    return "TX  ";
        case EV_RX:    return "RX  ";
        case EV_PHASE: return "PHS ";
        case EV_WARN:  return "WARN";
        case EV_INFO:  return "INFO";
    }
    return "    ";
}

static const char *mode_str(int m) {
    /* drone_mode_t in drone_main.c: 0=RAND 1=GOTO 2=HOLD 3=TERM */
    switch (m) {
        case 0: return "RAND";
        case 1: return "GOTO";
        case 2: return "HOLD";
        case 3: return "TERM";
        default: return "?";
    }
}

static void draw_header(drone_ui_ctx_t *ctx, drone_ui_state_t *st, int cols) {
    long elapsed = (now_ms() - ctx->start_ts_ms) / 1000;

    if (g_have_color) attron(COLOR_PAIR(CP_PHASE) | A_BOLD);
    mvprintw(0, 0, " DRONE %d  ", ctx->id);
    if (g_have_color) attroff(COLOR_PAIR(CP_PHASE) | A_BOLD);

    mvprintw(0, 12, "Mode: %-4s", mode_str(st->mode));
    mvprintw(0, 24, "Pos: (%7.2f, %7.2f)", st->x, st->y);
    if (st->has_target) {
        mvprintw(0, 56, "Target: (%6.1f,%6.1f)",
                 st->target_x, st->target_y);
    }
    mvprintw(1, 0, " Elapsed: %02ld:%02ld", elapsed / 60, elapsed % 60);
    if (st->has_target) {
        double dx = st->target_x - st->x;
        double dy = st->target_y - st->y;
        double d  = sqrt(dx * dx + dy * dy);
        mvprintw(1, 24, "Distance to target: %6.2f m", d);
    }
    mvhline(2, 0, ACS_HLINE, cols);
}

static void draw_log(int row, int log_h, int cols) {
    log_event_t buf[LOG_BUF_MAX];
    int n = event_bus_snapshot(buf, LOG_BUF_MAX);
    int start = 0;
    while (start < n && buf[start].ts_ms < g_clear_ts) start++;
    n -= start;
    log_event_t *evs = buf + start;

    int show = log_h - 1;
    if (show < 1) show = 1;
    int from = (n > show) ? n - show : 0;

    if (g_have_color) attron(A_BOLD);
    mvprintw(row, 0, "MESSAGE LOG");
    if (g_have_color) attroff(A_BOLD);

    int line = row + 1;
    for (int i = from; i < n && line < row + log_h; i++, line++) {
        time_t t = (time_t)(evs[i].ts_ms / 1000);
        int ms = (int)(evs[i].ts_ms % 1000);
        struct tm tm;
        localtime_r(&t, &tm);
        char ts[16];
        strftime(ts, sizeof(ts), "%H:%M:%S", &tm);

        int cp = kind_to_pair(evs[i].kind);
        int extra = (evs[i].kind == EV_PHASE || evs[i].kind == EV_WARN)
                    ? A_BOLD : 0;

        int prefix = 13 + 5;
        int avail = cols - prefix;
        if (avail < 1) avail = 1;
        if (avail > (int)sizeof(evs[i].text)) avail = (int)sizeof(evs[i].text);

        if (g_have_color) attron(COLOR_PAIR(cp));
        if (extra) attron(extra);
        mvprintw(line, 0, "%s.%03d  %s  %.*s",
                 ts, ms, kind_to_tag(evs[i].kind), avail, evs[i].text);
        if (extra) attroff(extra);
        if (g_have_color) attroff(COLOR_PAIR(cp));
    }
}

void *drone_ui_thread(void *arg) {
    drone_ui_ctx_t *ctx = (drone_ui_ctx_t *)arg;
    while (!g_quit) {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);

        drone_ui_state_t st;
        memset(&st, 0, sizeof(st));
        ctx->fetch(&st, ctx->user);

        erase();
        draw_header(ctx, &st, cols);
        int log_row = 4;
        int log_h = rows - log_row - 1;
        if (log_h < 4) log_h = 4;
        draw_log(log_row, log_h, cols);

        if (g_have_color) attron(A_DIM);
        mvprintw(rows - 1, 0, "[q] quit   [c] clear log");
        if (g_have_color) attroff(A_DIM);
        refresh();

        int ch = getch();
        if (ch == 'q' || ch == 'Q') {
            g_quit = 1;
        } else if (ch == 'c' || ch == 'C') {
            g_clear_ts = now_ms();
        }

        usleep(100 * 1000);
    }
    return NULL;
}
