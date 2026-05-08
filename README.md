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

자세한 설계는 [DESIGN_SPEC.md](DESIGN_SPEC.md), UI 계층은 [UI_DESIGN_SPEC.md](UI_DESIGN_SPEC.md) 참조.

## 디렉터리 구조

```
network_v2/
├── Makefile
├── DESIGN_SPEC.md           # 시스템 설계 문서
├── UI_DESIGN_SPEC.md        # TUI 설계 문서
├── README.md
├── common/
│   ├── config.h             # 영역·타이밍 상수
│   ├── protocol.{h,c}       # 메시지 프레이밍 (TYPE 1B + LEN 2B + payload)
│   ├── log.{h,c}            # 쓰레드 안전 로깅 (분류 이벤트 지원)
│   └── event_bus.{h,c}      # UI 용 이벤트 링버퍼 (200건, mutex)
├── server/
│   ├── drone_table.{h,c}    # 드론 테이블 + 동기화/송신 락
│   ├── mission.{h,c}        # 목표 좌표 계산, phase1 전역 검증
│   ├── ui.{h,c}             # ncurses TUI (맵·테이블·로그)
│   └── server_main.c        # accept · worker · 미션 컨트롤러
├── client/
│   ├── drone_ui.{h,c}       # 드론용 간소 TUI (선택)
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

요구 사항: `gcc`, GNU Make, POSIX Threads, libm, **ncurses** (`-lncurses`).
macOS 는 기본 제공, Linux 는 `libncurses-dev` 등 패키지 필요할 수 있음.

## 실행

서버 먼저 띄우고 드론 3대를 각각 다른 터미널에서 실행:

```bash
# 터미널 1 — 서버 (포트 9000, 드론 3대 대기, 기본 TUI)
./bin/server 9000 3

# 터미널 2~4 — 드론 (서버주소, 포트, 드론ID)
./bin/drone 127.0.0.1 9000 1
./bin/drone 127.0.0.1 9000 2
./bin/drone 127.0.0.1 9000 3
```

| 인자 | 설명 |
|------|------|
| `./bin/server [port] [expected_drones] [--no-tui]` | 기본 9000 / 3. `--no-tui` 는 stdout 모드 (디버깅·CI 용) |
| `./bin/drone <ip> <port> <id> [--tui]` | `--tui` 추가 시 드론도 TUI 화면 표시 |

> 서버 TUI 권장 터미널 크기는 **100 cols × 40 rows** (최소 80×24).
> TTY 가 없거나 `TERM` 미설정인 경우 자동으로 stdout 모드로 fallback.

## TUI (관제 화면)

서버 실행 시 ncurses 기반 TUI 가 뜬다. 데모 동영상 / 결과보고서 캡쳐 친화적으로 설계됨.

```
┌────────────────────────────────────────────────────────────────┐
│ DRONE CONTROL CENTER   Phase: PHASE1_GO   Elapsed: 00:23  3/3  │
├────────────────────────────────────────────────────────────────┤
│   y=200m  ┌──────────────────────────────────────────────────┐ │
│           │                · · ·                              │ │
│           │             ·  · 2 ·  ·     ← phase1 목표 원      │ │
│           │           ·  +     +  ·                           │ │
│           │           ·  1●        ·                          │ │
│           │             ·   3   ·                             │ │
│   y= 20m  │                  ^                                │ │
│           └──────────────────────────────────────────────────┘ │
│   x=-100m                Base Station                  x=+100m │
├────────────────────────────────────────────────────────────────┤
│ DRONES                                                         │
│  ID | Mode |   X(m)   |   Y(m)  |    Target     |Last Seen|... │
│   1 | GOTO |   -3.21  |  95.40  | ( -7.5,113.0) | 0.4s ago| .. │
│   2 | HOLD |   15.00  | 100.00  | ( 15.0,100.0) | 0.1s ago| .. │
├────────────────────────────────────────────────────────────────┤
│ MESSAGE LOG  [tail]                                            │
│ 14:05:21.103  TX    MOVE_CMD   drone=1 -> (-7.50, 113.00)      │
│ 14:05:21.205  RX    POS_REPORT drone=2 (12.41, 99.13)          │
│ 14:05:21.412  PHS   *** PHASE1_OK ***                          │
└────────────────────────────────────────────────────────────────┘
[q] quit   [p] pause log   [c] clear log
```

### 시각 요소
| 요소 | 표시 | 비고 |
|------|------|------|
| 드론 본체 | `1` `2` `3` …, 색상은 ID별 (CYAN / YELLOW / MAGENTA) | bold |
| 드론 목표점 | `+` (드론과 동일 색, 옅게) | — |
| Phase1 목표 원 (반경 15m) | `*` | — |
| Phase1 검증 원 (반경 30m) | `.` | dim |
| 기지국 | `^` (0, 20m) | — |
| TX / RX / PHASE / WARN | GREEN / BLUE / YELLOW / RED | 분류 색상 |

### 키 바인딩
| 키 | 동작 |
|----|------|
| `q` | 종료 (서버 본체와 함께 정리) |
| `p` | 로그 스크롤 일시정지 / 재개 (이벤트 버퍼는 계속 적재) |
| `c` | 로그 화면 클리어 (버퍼 유지) |

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

- **서버**: `accept` 쓰레드 1 + 드론별 `worker` 쓰레드 N + 메인(미션 컨트롤러) + **UI 쓰레드 1** (10 fps 렌더).
- **클라이언트**: `move` 쓰레드 + `recv` 쓰레드 + (옵션) UI 쓰레드.
- 드론 테이블은 `pthread_mutex_t` + `pthread_cond_t`로 보호.
- UI 쓰레드는 **읽기 전용 옵저버** — `table_snapshot()` 으로 락 잡고 복사만.
- **send()는 절대 테이블 락을 잡은 채 호출하지 않는다** — 소켓별 송신 락만 잡고 송신.
- `event_bus` 는 자체 mutex 로 보호되는 ring buffer; `log_event()` 가 자동으로 push.

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

UI 관련 상수는 `server/ui.c` 상단에서 변경 가능 (`LOG_BUF_MAX`, 컬러 페어 등).

## 평가 항목 매핑

| 평가 항목 | 본 구현의 기여 |
|-----------|----------------|
| 동시성 / 다중 드론 처리 | accept + worker + 미션 컨트롤러 + UI 쓰레드, 테이블/송신 분리 락 |
| 메시지 종류 다양성 | HELLO / HELLO_ACK / POS_REPORT / MOVE_CMD / ARRIVED / PHASE_ACK / TERMINATE |
| UI 우수성 (드론 움직임 가시화) | TUI 맵 영역에 ID 글리프 + 목표점 + phase 원 실시간 표시 |
| UI 우수성 (메시지 송수신 가시화) | TUI 로그 영역에 TX/RX/PHASE 분류 컬러로 흐름 표시 |
| 데모 동영상 / 결과보고서 | TUI 화면 캡쳐 + `--no-tui` 회귀 로그 양쪽 모두 확보 가능 |
