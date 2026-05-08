#ifndef DRONE_UI_H
#define DRONE_UI_H

typedef struct {
    int     id;
    int     mode;       /* drone_mode_t (DR_RANDOM/GOTO/HOLD/TERM) */
    double  x, y;
    double  target_x, target_y;
    int     has_target; /* 0: no target yet, 1: target set */
} drone_ui_state_t;

typedef void (*drone_ui_state_fn)(drone_ui_state_t *out, void *user);

typedef struct {
    int                 id;
    drone_ui_state_fn   fetch;
    void               *user;
    long                start_ts_ms;
} drone_ui_ctx_t;

int   drone_ui_init(void);
void  drone_ui_shutdown(void);
void *drone_ui_thread(void *arg);
int   drone_ui_should_quit(void);
void  drone_ui_request_quit(void);

#endif
