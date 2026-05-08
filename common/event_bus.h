#ifndef EVENT_BUS_H
#define EVENT_BUS_H

typedef enum {
    EV_INFO = 0,
    EV_TX,
    EV_RX,
    EV_PHASE,
    EV_WARN
} ev_kind_t;

typedef struct {
    long      ts_ms;
    ev_kind_t kind;
    char      text[160];
} log_event_t;

void event_bus_init(int capacity);
void event_bus_destroy(void);
void event_bus_push(ev_kind_t kind, const char *fmt, ...);

/* 최신 max건을 시간순(오래된 → 최신)으로 out 에 복사. 반환: 복사 개수. */
int  event_bus_snapshot(log_event_t *out, int max);

#endif
