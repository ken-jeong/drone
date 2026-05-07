#ifndef CONFIG_H
#define CONFIG_H

#define DEFAULT_PORT             9000
#define DEFAULT_EXPECTED_DRONES  3

#define MAX_DRONES               16
#define MAX_PAYLOAD              256

/* 영역 제약 (m) */
#define AREA_X_MIN              -100.0
#define AREA_X_MAX               100.0
#define AREA_Y_MIN                20.0
#define AREA_Y_MAX               200.0

/* 1차 집결 */
#define PHASE1_CENTER_X            0.0
#define PHASE1_CENTER_Y          100.0
#define PHASE1_RADIUS             30.0      /* 검증 반경 */
#define PHASE1_TARGET_RADIUS      15.0      /* 목표점 배치 반경 */
#define PHASE1_MIN_GAP            10.0

/* 2차 이동 */
#define PHASE2_LEFT_SHIFT         50.0

/* 타이밍 / 모션 */
#define POS_REPORT_INTERVAL_MS   500
#define MOVE_STEP_M                2.0
#define ARRIVE_TOLERANCE           0.5
#define VALIDATION_TOL             0.5
#define MISSION_POLL_MS          200

#endif
