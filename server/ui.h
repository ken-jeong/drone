#ifndef UI_H
#define UI_H

#include "drone_table.h"
#include "mission.h"

typedef struct {
    drone_table_t           *table;
    volatile mission_phase_t *phase_ptr;
    long                     start_ts_ms;
    int                      expected;
} ui_ctx_t;

int   ui_init(void);
void  ui_shutdown(void);
void *ui_thread(void *arg);   /* arg = ui_ctx_t* */
int   ui_should_quit(void);
void  ui_request_quit(void);

#endif
