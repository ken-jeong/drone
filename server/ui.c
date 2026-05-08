#include "ui.h"
#include "config.h"
#include "event_bus.h"

#include <ncurses.h>
#include <locale.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* color pairs */
#define CP_DRONE1   1
#define CP_DRONE2   2
#define CP_DRONE3   3
#define CP_TX       4
#define CP_RX       5
#define CP_PHASE    6
#define CP_WARN     7
#define CP_HEADER   8
#define CP_INFO     9

#define LOG_BUF_MAX 200

static int               g_initialized = 0;
static volatile int      g_quit        = 0;
static int               g_paused      = 0;
static long              g_pause_ts    = 0;
static long              g_clear_ts    = 0;
static int               g_have_color  = 0;

static long now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long)tv.tv_sec * 1000L + (long)tv.tv_usec / 1000L;
}

int ui_init(void) {
    /* ncurses initscr() exits the process on failure; pre-check first. */
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
        init_pair(CP_DRONE1, COLOR_CYAN,    -1);
        init_pair(CP_DRONE2, COLOR_YELLOW,  -1);
        init_pair(CP_DRONE3, COLOR_MAGENTA, -1);
        init_pair(CP_TX,     COLOR_GREEN,   -1);
        init_pair(CP_RX,     COLOR_BLUE,    -1);
        init_pair(CP_PHASE,  COLOR_YELLOW,  -1);
        init_pair(CP_WARN,   COLOR_RED,     -1);
        init_pair(CP_HEADER, COLOR_WHITE,   -1);
        init_pair(CP_INFO,   COLOR_WHITE,   -1);
    }
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    g_initialized = 1;
    return 0;
}

void ui_shutdown(void) {
    if (g_initialized) {
        endwin();
        g_initialized = 0;
    }
}

int  ui_should_quit(void) { return g_quit; }
void ui_request_quit(void) { g_quit = 1; }

static int drone_color_pair(int id) {
    static const int pairs[3] = { CP_DRONE1, CP_DRONE2, CP_DRONE3 };
    int i = (id - 1);
    if (i < 0) i = 0;
    return pairs[i % 3];
}

static int world_to_col(double x, int x0, int w) {
    double t = (x - AREA_X_MIN) / (AREA_X_MAX - AREA_X_MIN);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return x0 + (int)(t * (w - 1) + 0.5);
}

static int world_to_row(double y, int y0, int h) {
    double t = (AREA_Y_MAX - y) / (AREA_Y_MAX - AREA_Y_MIN);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return y0 + (int)(t * (h - 1) + 0.5);
}

static void aon(int pair, int extra) {
    if (g_have_color && pair) attron(COLOR_PAIR(pair));
    if (extra) attron(extra);
}
static void aoff(int pair, int extra) {
    if (extra) attroff(extra);
    if (g_have_color && pair) attroff(COLOR_PAIR(pair));
}

/* ---------- header ---------- */

static void draw_header(ui_ctx_t *ctx, int row, int n_active) {
    long elapsed = (now_ms() - ctx->start_ts_ms) / 1000;
    mission_phase_t ph = *(ctx->phase_ptr);

    aon(CP_HEADER, A_BOLD);
    mvprintw(row, 0, "DRONE CONTROL CENTER");
    aoff(CP_HEADER, A_BOLD);

    aon(CP_PHASE, A_BOLD);
    mvprintw(row, 24, "Phase: %-12s", phase_name(ph));
    aoff(CP_PHASE, A_BOLD);

    mvprintw(row, 50, "Elapsed: %02ld:%02ld", elapsed / 60, elapsed % 60);
    mvprintw(row, 68, "Drones: %d/%d", n_active, ctx->expected);
}

/* ---------- map ---------- */

static void draw_circle(double cx, double cy, double r,
                        int x0, int y0, int w, int h, char ch) {
    double step = 0.06;
    for (double th = 0.0; th < 2.0 * M_PI; th += step) {
        double wx = cx + r * cos(th);
        double wy = cy + r * sin(th);
        int cc = world_to_col(wx, x0, w);
        int rr = world_to_row(wy, y0, h);
        if (cc >= x0 && cc < x0 + w && rr >= y0 && rr < y0 + h) {
            mvaddch(rr, cc, ch);
        }
    }
}

static void draw_box(int row, int col, int h, int w) {
    mvaddch(row,         col,         ACS_ULCORNER);
    mvaddch(row,         col + w - 1, ACS_URCORNER);
    mvaddch(row + h - 1, col,         ACS_LLCORNER);
    mvaddch(row + h - 1, col + w - 1, ACS_LRCORNER);
    for (int i = 1; i < w - 1; i++) {
        mvaddch(row,         col + i, ACS_HLINE);
        mvaddch(row + h - 1, col + i, ACS_HLINE);
    }
    for (int i = 1; i < h - 1; i++) {
        mvaddch(row + i, col,         ACS_VLINE);
        mvaddch(row + i, col + w - 1, ACS_VLINE);
    }
}

static void draw_map(ui_ctx_t *ctx, int row, int col, int h, int w) {
    draw_box(row, col, h, w);

    int ix = col + 1;
    int iy = row + 1;
    int iw = w - 2;
    int ih = h - 2;

    /* phase1 검증 원 (반경 30m) — 더 옅게 */
    aon(CP_INFO, A_DIM);
    draw_circle(PHASE1_CENTER_X, PHASE1_CENTER_Y, PHASE1_RADIUS,
                ix, iy, iw, ih, '.');
    aoff(CP_INFO, A_DIM);

    /* phase1 목표 원 (반경 15m) */
    aon(CP_INFO, 0);
    draw_circle(PHASE1_CENTER_X, PHASE1_CENTER_Y, PHASE1_TARGET_RADIUS,
                ix, iy, iw, ih, '*');
    aoff(CP_INFO, 0);

    /* 기지국 (0, AREA_Y_MIN 부근) */
    int bx = world_to_col(0.0, ix, iw);
    int by = world_to_row(AREA_Y_MIN, iy, ih);
    if (by < iy + ih) {
        aon(CP_HEADER, A_BOLD);
        mvaddch(by, bx, '^');
        aoff(CP_HEADER, A_BOLD);
    }

    /* 드론 목표점 */
    drone_entry_t snap[MAX_DRONES];
    int n = table_snapshot(ctx->table, snap, MAX_DRONES);

    for (int i = 0; i < n; i++) {
        if (snap[i].mode != DRONE_GOTO && snap[i].mode != DRONE_HOLD) continue;
        int cc = world_to_col(snap[i].target_x, ix, iw);
        int rr = world_to_row(snap[i].target_y, iy, ih);
        if (cc < ix || cc >= ix + iw || rr < iy || rr >= iy + ih) continue;
        int cp = drone_color_pair(snap[i].id);
        aon(cp, A_DIM);
        mvaddch(rr, cc, '+');
        aoff(cp, A_DIM);
    }

    /* 드론 본체 */
    for (int i = 0; i < n; i++) {
        int cc = world_to_col(snap[i].x, ix, iw);
        int rr = world_to_row(snap[i].y, iy, ih);
        char glyph = (char)('0' + (snap[i].id % 10));
        int cp = drone_color_pair(snap[i].id);
        int extra = A_BOLD;
        if (snap[i].mode == DRONE_HOLD) extra = A_BOLD;
        if (snap[i].mode == DRONE_TERMINATED) extra = A_DIM;
        aon(cp, extra);
        mvaddch(rr, cc, glyph);
        aoff(cp, extra);
    }

    /* y, x 축 라벨 */
    aon(CP_HEADER, A_DIM);
    mvprintw(iy,             col - 8, "y=%4dm", (int)AREA_Y_MAX);
    mvprintw(iy + ih - 1,    col - 8, "y=%4dm", (int)AREA_Y_MIN);
    mvprintw(row + h, col,                  "x=%-+5d", (int)AREA_X_MIN);
    mvprintw(row + h, col + w - 7,          "x=%-+5d", (int)AREA_X_MAX);
    {
        const char *bs = "Base Station";
        int xc = col + (w - (int)strlen(bs)) / 2;
        mvprintw(row + h, xc, "%s", bs);
    }
    aoff(CP_HEADER, A_DIM);
}

/* ---------- drone table ---------- */

static int cmp_drone_id(const void *a, const void *b) {
    int ia = ((const drone_entry_t *)a)->id;
    int ib = ((const drone_entry_t *)b)->id;
    return (ia > ib) - (ia < ib);
}

static const char *mode_str(drone_mode_t m) {
    switch (m) {
        case DRONE_RANDOM:     return "RAND";
        case DRONE_GOTO:       return "GOTO";
        case DRONE_HOLD:       return "HOLD";
        case DRONE_TERMINATED: return "TERM";
        default:               return "?";
    }
}

static void draw_drone_table(ui_ctx_t *ctx, int row, int *out_rows) {
    drone_entry_t snap[MAX_DRONES];
    int n = table_snapshot(ctx->table, snap, MAX_DRONES);
    qsort(snap, n, sizeof(snap[0]), cmp_drone_id);

    aon(CP_HEADER, A_BOLD);
    mvprintw(row, 0, "DRONES");
    aoff(CP_HEADER, A_BOLD);

    aon(CP_HEADER, A_DIM);
    mvprintw(row + 1, 0,
             " ID | Mode |   X(m)   |   Y(m)   |    Target        | Last Seen | State");
    aoff(CP_HEADER, A_DIM);

    long now = now_ms();
    for (int i = 0; i < n; i++) {
        const drone_entry_t *d = &snap[i];

        char target[24];
        if (d->mode == DRONE_GOTO || d->mode == DRONE_HOLD) {
            snprintf(target, sizeof(target), "(%6.1f,%6.1f)",
                     d->target_x, d->target_y);
        } else {
            snprintf(target, sizeof(target), "%-16s", "-");
        }

        char seen[16];
        long age_ms = now - d->last_seen_ms;
        if (age_ms < 0) age_ms = 0;
        snprintf(seen, sizeof(seen), "%4.1fs ago", age_ms / 1000.0);

        const char *state;
        switch (d->mode) {
            case DRONE_GOTO:       state = "moving";     break;
            case DRONE_HOLD:       state = "arrived";    break;
            case DRONE_RANDOM:     state = "idle/rand";  break;
            case DRONE_TERMINATED: state = "terminated"; break;
            default:               state = "?";          break;
        }

        int cp = drone_color_pair(d->id);
        int extra = (d->mode == DRONE_TERMINATED) ? A_DIM : A_BOLD;
        aon(cp, extra);
        mvprintw(row + 2 + i, 0,
                 " %2d | %-4s | %8.2f | %8.2f | %-16s | %-9s | %s",
                 d->id, mode_str(d->mode), d->x, d->y,
                 target, seen, state);
        aoff(cp, extra);
    }
    if (out_rows) *out_rows = 2 + n;
}

/* ---------- log ---------- */

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

static void draw_log(int row, int log_h, int cols) {
    log_event_t buf[LOG_BUF_MAX];
    int n = event_bus_snapshot(buf, LOG_BUF_MAX);

    /* 시간 필터 (clear / pause 처리) */
    int start = 0;
    while (start < n && buf[start].ts_ms < g_clear_ts) start++;
    int end = n;
    if (g_paused) {
        while (end > start && buf[end - 1].ts_ms > g_pause_ts) end--;
    }
    n = end - start;
    log_event_t *evs = buf + start;

    /* 표시 영역에 맞게 마지막 (log_h - 1) 건만 출력 */
    int show = log_h - 1;
    if (show < 1) show = 1;
    int from = (n > show) ? n - show : 0;

    aon(CP_HEADER, A_BOLD);
    mvprintw(row, 0, "MESSAGE LOG  %s",
             g_paused ? "[PAUSED]" : "[tail]");
    aoff(CP_HEADER, A_BOLD);

    int line = row + 1;
    for (int i = from; i < n && line < row + log_h; i++, line++) {
        time_t t = (time_t)(evs[i].ts_ms / 1000);
        int    ms = (int)(evs[i].ts_ms % 1000);
        struct tm tm;
        localtime_r(&t, &tm);
        char ts[16];
        strftime(ts, sizeof(ts), "%H:%M:%S", &tm);

        int cp = kind_to_pair(evs[i].kind);
        int extra = (evs[i].kind == EV_PHASE || evs[i].kind == EV_WARN)
                    ? A_BOLD : 0;

        /* 좁은 터미널이면 잘라서 출력 */
        int prefix_len = 13 + 5;  /* "HH:MM:SS.mmm  XXXX  " */
        int avail = cols - prefix_len;
        if (avail < 1) avail = 1;
        if (avail > (int)sizeof(evs[i].text)) avail = (int)sizeof(evs[i].text);

        aon(cp, extra);
        mvprintw(line, 0, "%s.%03d  %s  %.*s",
                 ts, ms, kind_to_tag(evs[i].kind), avail, evs[i].text);
        aoff(cp, extra);
    }
}

/* ---------- too small ---------- */

static void draw_too_small(int rows, int cols) {
    erase();
    const char *m1 = "Terminal too small";
    const char *m2 = "Need at least 80 cols x 24 rows";
    mvprintw(rows / 2,     (cols - (int)strlen(m1)) / 2, "%s", m1);
    mvprintw(rows / 2 + 1, (cols - (int)strlen(m2)) / 2, "%s", m2);
    refresh();
}

/* ---------- main loop ---------- */

void *ui_thread(void *arg) {
    ui_ctx_t *ctx = (ui_ctx_t *)arg;

    while (!g_quit) {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);

        if (rows < 24 || cols < 80) {
            draw_too_small(rows, cols);
        } else {
            erase();

            drone_entry_t snap[MAX_DRONES];
            int n_active = table_snapshot(ctx->table, snap, MAX_DRONES);

            /* 레이아웃 산정 */
            int header_row = 0;

            /* map */
            int map_row = 2;
            int map_h   = rows / 2;
            if (map_h < 12) map_h = 12;
            if (map_h > 22) map_h = 22;
            int map_col = 8;
            int map_w   = cols - map_col - 1;

            /* drone table */
            int xaxis_row = map_row + map_h;
            int table_row = xaxis_row + 2;
            int table_h   = 2 + n_active;
            if (table_h < 3) table_h = 3;

            /* log */
            int log_row = table_row + table_h + 1;
            int log_h   = rows - log_row - 2;
            if (log_h < 4) log_h = 4;

            draw_header(ctx, header_row, n_active);
            draw_map(ctx, map_row, map_col, map_h, map_w);
            int actual_table_h = 0;
            draw_drone_table(ctx, table_row, &actual_table_h);
            draw_log(log_row, log_h, cols);

            aon(CP_HEADER, A_DIM);
            mvprintw(rows - 1, 0,
                     "[q] quit   [p] pause log   [c] clear log");
            aoff(CP_HEADER, A_DIM);

            refresh();
        }

        /* 키 입력 */
        int ch = getch();
        if (ch == 'q' || ch == 'Q') {
            g_quit = 1;
        } else if (ch == 'p' || ch == 'P') {
            g_paused = !g_paused;
            if (g_paused) g_pause_ts = now_ms();
        } else if (ch == 'c' || ch == 'C') {
            g_clear_ts = now_ms();
        }

        usleep(100 * 1000);  /* 10 fps */
    }
    return NULL;
}
