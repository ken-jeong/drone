#ifndef MISSION_H
#define MISSION_H

#include "drone_table.h"

typedef enum {
    MISSION_INIT = 0,
    MISSION_PHASE1_GO,
    MISSION_PHASE1_OK,
    MISSION_PHASE2_GO,
    MISSION_PHASE2_OK,
    MISSION_DONE
} mission_phase_t;

const char *phase_name(mission_phase_t p);

/* 1차 집결 목표 좌표 (중심 (0,100), 반경 PHASE1_TARGET_RADIUS 의 원주에 균등 배치). */
void compute_phase1_targets(int n, double *tx, double *ty);

/* 1차 집결 전역 검증: 모두 반경 30m 내 + 상호 간격 10m 이상. */
int  check_phase1_complete(const drone_entry_t *snap, int n);

#endif
