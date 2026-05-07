# 드론 관제 시스템 (네트워크 프로그래밍 프로젝트)

> 2026학년도 1학기 네트워크 프로그래밍(12) 프로젝트
> TCP · 다중 쓰레드 · C언어

기지국 서버 1대가 3대 이상의 드론 클라이언트의 위치를 추적하고, 단계적으로 집결(1차) → 좌측 이동(2차) 명령을 내리는 협응 시스템.

## 시스템 개요

```
   클라이언트1 ─┐
   클라이언트2 ─┼── TCP ──► [기지국 서버] ──► 미션 컨트롤
   클라이언트3 ─┘                              · 단계 전이
                                               · 명령 발행
                                               · 위치 검증
```

| 항목 | 값 |
|------|-----|
| 언어 | C (C99) |
| 동시성 | POSIX Threads (`pthread`) |
| 전송 | TCP |
| 영역 | x ∈ [-100, +100] m, y ∈ [20, 200] m |
| 1차 집결 | 중심 (0, 100), 반경 ≤ 30 m, 드론 간 간격 ≥ 10 m |
| 2차 이동 | 각 드론 좌측 50 m |

자세한 설계는 [DESIGN_SPEC.md](DESIGN_SPEC.md) 참조.

## 디렉터리 구조

```
network_v2/
├── Makefile
├── DESIGN_SPEC.md           # 설계 문서
├── README.md
├── common/
│   ├── config.h             # 영역·타이밍 상수
│   ├── protocol.{h,c}       # 메시지 프레이밍 (TYPE 1B + LEN 2B + payload)
│   └── log.{h,c}            # 쓰레드 안전 로깅
├── server/
│   ├── drone_table.{h,c}    # 드론 테이블 + 동기화/송신 락
│   ├── mission.{h,c}        # 목표 좌표 계산, phase1 전역 검증
│   └── server_main.c        # accept · worker · 미션 컨트롤러
├── client/
│   └── drone_main.c         # 드론 클라이언트 (move + recv 쓰레드)
└── bin/                     # 빌드 산출물
    ├── server
    └── drone
```

## 빌드

```bash
make           # bin/server, bin/drone 생성
make clean     # 산출물 정리
```

요구 사항: `gcc`, GNU Make, POSIX Threads, libm. macOS / Linux에서 동작 확인.

## 실행

서버 먼저 띄우고 드론 3대를 각각 다른 터미널에서 실행:

```bash
# 터미널 1 — 서버 (포트 9000, 드론 3대 대기)
./bin/server 9000 3

# 터미널 2~4 — 드론 (서버주소, 포트, 드론ID)
./bin/drone 127.0.0.1 9000 1
./bin/drone 127.0.0.1 9000 2
./bin/drone 127.0.0.1 9000 3
```

서버 인자: `./bin/server [port] [expected_drones]` (기본 9000, 3)

## 미션 시나리오

```
INIT
  │  N대 모두 접속
  ▼
PHASE1_GO   서버가 각 드론에게 (0,100) 주변 원주의 목표 좌표 발행
  │  모든 드론 도착 + 전역 검증(반경/간격) 통과
  ▼
PHASE1_OK
  │  자동 전이
  ▼
PHASE2_GO   각 드론의 1차 좌표에서 x -= 50
  │  모든 드론 도착
  ▼
PHASE2_OK → TERMINATE → DONE
```

3대 기준 한 사이클은 약 35초 소요 (이동 속도 4 m/s).

## 메시지 프로토콜

```
0       1       3                                   N
+-------+-------+-----------------------------------+
| TYPE  | LEN   |          PAYLOAD (LEN bytes)      |
| (1B)  | (2B)  |     (network byte order)          |
+-------+-------+-----------------------------------+
```

| 코드 | 이름 | 방향 | 페이로드 |
|------|------|------|----------|
| `0x01` | `HELLO` | C→S | `int32 drone_id` |
| `0x02` | `HELLO_ACK` | S→C | `int32 assigned_id` |
| `0x10` | `POS_REPORT` | C→S | `int32 id, int32 x_mm, int32 y_mm` |
| `0x20` | `MOVE_CMD` | S→C | `int32 target_x_mm, int32 target_y_mm` |
| `0x21` | `ARRIVED` | C→S | `int32 id, int32 x_mm, int32 y_mm` |
| `0x30` | `PHASE_ACK` | S→C | `int32 phase` |
| `0xFF` | `TERMINATE` | S→C | (없음) |

좌표는 부동소수점 직렬화 이슈를 피하기 위해 mm 단위 `int32`로 송신.

## 동시성 설계 핵심

- **서버**: `accept` 쓰레드 1 + 드론별 `worker` 쓰레드 N + 메인(미션 컨트롤러).
- **클라이언트**: `move` 쓰레드 + `recv` 쓰레드.
- 드론 테이블은 `pthread_mutex_t` + `pthread_cond_t`로 보호.
- **send()는 절대 테이블 락을 잡은 채 호출하지 않는다** — 소켓별 송신 락만 잡고 송신.

## 동작 검증 예시 (서버 로그)

```
[mission] INIT — waiting for 3 drones
[server] drone 1 connected (slot=1)
[server] drone 2 connected (slot=0)
[server] drone 3 connected (slot=2)
[mission] all 3 drones connected
[mission] PHASE1_GO — dispatching phase 1 targets
[server] TX MOVE_CMD  drone=2 -> (15.00, 100.00)
[server] TX MOVE_CMD  drone=1 -> (-7.50, 112.99)
[server] TX MOVE_CMD  drone=3 -> (-7.50, 87.01)
... POS_REPORT 수신 ...
[server] RX ARRIVED   drone=2 (15.00, 100.00)
[server] RX ARRIVED   drone=1 (-7.50, 112.99)
[server] RX ARRIVED   drone=3 (-7.50, 87.01)
[mission] phase1 global check OK (radius<=30m, gap>=10m)
[mission] PHASE1_OK
[mission] PHASE2_GO — dispatching phase 2 targets
... POS_REPORT 수신 ...
[mission] PHASE2_OK
[server] TX TERMINATE drone=1
[server] TX TERMINATE drone=2
[server] TX TERMINATE drone=3
[mission] DONE
```

## 종료

미션이 `DONE`에 도달하면 서버는 모든 드론에 `TERMINATE`를 송신한 뒤 자체 종료한다. 비정상 종료 시 `Ctrl+C`로 서버를 내리면 워커 쓰레드들이 소켓 닫힘을 감지하고 정리된다.

## 주요 파라미터 조정

`common/config.h` 에서 변경:

| 매크로 | 기본값 | 의미 |
|--------|--------|------|
| `DEFAULT_PORT` | 9000 | 서버 포트 |
| `MAX_DRONES` | 16 | 동시 접속 최대 드론 수 |
| `POS_REPORT_INTERVAL_MS` | 500 | 위치 보고 주기 |
| `MOVE_STEP_M` | 2.0 | 한 틱당 이동 거리 |
| `PHASE1_TARGET_RADIUS` | 15.0 | 1차 목표 원주 반경 |
| `VALIDATION_TOL` | 0.5 | 검증 허용 오차 |
