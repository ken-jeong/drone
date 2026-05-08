#ifndef DRONE_TABLE_H
#define DRONE_TABLE_H

#include <pthread.h>
#include <stdint.h>
#include "config.h"

typedef enum {
    DRONE_RANDOM = 0,
    DRONE_GOTO,
    DRONE_HOLD,
    DRONE_TERMINATED
} drone_mode_t;

/* 테이블 엔트리 (뮤텍스 미포함, 복사 가능). */
typedef struct {
    int             id;
    int             active;
    int             socket_fd;
    double          x, y;
    drone_mode_t    mode;
    double          target_x, target_y;
    long            last_seen_ms;   /* 마지막 POS_REPORT/ARRIVED 시각 */
    int             phase1_done;
    int             phase2_done;
} drone_entry_t;

/* send_locks 는 entry 와 같은 인덱스를 공유하는 평행 배열. */
typedef struct {
    drone_entry_t   drones[MAX_DRONES];
    pthread_mutex_t send_locks[MAX_DRONES];
    int             count;
    pthread_mutex_t table_lock;
    pthread_cond_t  state_changed;
} drone_table_t;

void table_init(drone_table_t *t);
void table_destroy(drone_table_t *t);

/* 신규 드론 등록. 반환: 슬롯 인덱스 또는 -1. */
int  table_add(drone_table_t *t, int id, int sock);

/* 위치 갱신 (없으면 무시). 마지막 갱신 시각도 갱신. */
void table_update_pos(drone_table_t *t, int id, double x, double y);

/* 목표/모드 갱신 (UI 표시용). */
void table_set_goto(drone_table_t *t, int id, double tx, double ty);
void table_set_arrived(drone_table_t *t, int id);
void table_set_terminated(drone_table_t *t, int id);

/* 소켓으로 엔트리 제거 (소켓도 close). */
void table_remove_by_sock(drone_table_t *t, int sock);

/* 활성 엔트리 스냅샷 복사. 반환: 복사된 개수. */
int  table_snapshot(drone_table_t *t, drone_entry_t *out, int max);

/* 안전한 메시지 송신. 내부적으로 송신 락을 잡고 send_msg 호출. */
int  table_send_to(drone_table_t *t, int id,
                   uint8_t type, const void *payload, uint16_t len);

/* count >= expected 가 될 때까지 블록. */
void table_wait_count(drone_table_t *t, int expected);

#endif
