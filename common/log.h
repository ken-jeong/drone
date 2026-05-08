#ifndef LOG_H
#define LOG_H

#include "event_bus.h"

void log_init(void);

/* EV_INFO 로 push. 기존 호출 부 호환. */
void log_msg(const char *fmt, ...);

/* 분류된 이벤트 push (TX/RX/PHASE/WARN). */
void log_event(ev_kind_t kind, const char *fmt, ...);

/* TUI 모드일 때 stdout 출력 비활성. */
void log_set_quiet(int quiet);

#endif
