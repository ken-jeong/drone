/*
    protocol.h - 드론 군집 제어 시스템 공유 헤더
    서버(server.c)와 드론 클라이언트(drone.c)가 함께 포함한다.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

// 메시지 종류
#define MSG_POSITION_REPORT  1 // 드론 -> 서버: 현재 위치 보고
#define MSG_MOVE_CMD         2 // 서버 -> 드론: 목표 위치로 이동
#define MSG_ARRIVED          3 // 드론 -> 서버: 목표 도달 통보 (선택)
#define MSG_TERMINATE        4 // 서버 -> 드론: 종료

// 고정 크기 메시지 (RecvFixed/SendFixed로 송수신)
typedef struct {
    int   type;                // MSG_* 종류
    int   droneId;             // 드론 식별자
    float x;                   // 위치/목표 X (좌우)
    float y;                   // 위치/목표 Y (고도)
    int   seq;                 // 순서 번호 (로깅용)
} Message;

// 월드(영역) 상수
#define WORLD_X_MIN  (-100.0f)
#define WORLD_X_MAX  ( 100.0f)
#define WORLD_Y_MIN  (  20.0f) // 최저 고도 20m
#define WORLD_Y_MAX  ( 200.0f) // 최고 고도 200m

// 군집 목표 상수
#define CENTER_X        (0.0f) // 집결 중심 X
#define CENTER_Y      (100.0f) // 집결 중심 Y (고도 100m)
#define GATHER_RADIUS  (30.0f) // 집결 허용 반경 30m
#define RING_RADIUS    (15.0f) // 목표 배치 원주 반경
#define MIN_SEPARATION (10.0f) // 드론 상호 최소 간격
#define SHIFT_LEFT     (50.0f) // 2차 좌측 이동 거리

// 운동/판정 상수
#define MOVE_STEP       (2.0f) // 틱당 이동 거리
#define ARRIVE_EPS      (1.0f) // 목표 도달 허용 오차

// 운용 상수
#define MIN_DRONES           3 // 최소 운용 드론 수
#define REPORT_PERIOD_MS   200 // 위치 보고 주기
#define CONTROL_PERIOD_MS  200 // 제어 루프 주기
#define RENDER_PERIOD_MS   100 // 렌더 주기 (10 FPS)

#endif // PROTOCOL_H
