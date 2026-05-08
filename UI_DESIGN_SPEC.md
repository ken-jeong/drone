# 드론 관제 시스템 UI 디자인 스펙

> 본 문서는 [DESIGN_SPEC.md](DESIGN_SPEC.md) 의 후속 문서로,
> 서버-드론 간 **메시지 송수신 상황**과 **드론의 실시간 움직임**을 시각화하는 UI 계층을 다룬다.

---

## 1. 개요

### 1.1 목표
- 데모 동영상에서 시청자가 **시스템 동작을 한눈에 이해**할 수 있도록 시각화한다.
- 평가 기준 중 "UI 우수성"(드론 움직임 + 메시지 송수신 상황 가시화) 항목을 충족한다.

### 1.2 비목표 (Non-goals)
- 그래픽 GUI(GTK/Qt/SDL) 미사용 — 추가 의존성 회피.
- 마우스 인터랙션, 사용자 입력 기반 명령 제어 미포함 (UI는 관찰 전용).
- 네트워크로 분리된 별도 뷰어 프로세스 미구현 (서버 프로세스 내부 통합).

### 1.3 기술 선택
| 후보 | 채택 여부 | 사유 |
|------|----------|------|
| **ncurses TUI** | ✅ 채택 | C 표준 환경, macOS/Linux 기본 제공, 데모 녹화 친화적 |
| SDL/raylib (그래픽) | ❌ | 외부 의존성, 빌드 복잡도 증가 |
| 별도 웹 뷰어(WebSocket) | ❌ | 프로토콜 추가 설계 필요, 과제 범위 초과 |
| 단순 stdout 갱신 | ❌ | 화면 리프레시·레이아웃 제어 불가 |

> ncurses 는 macOS 에 기본 설치되어 있으며 (`-lncurses`), POSIX 환경에서 표준에 가깝다.

---

## 2. 시스템 통합 관점

### 2.1 컴포넌트 변경 사항
```
┌──────────── server (process) ────────────┐
│                                          │
│  ┌─────────────┐    ┌─────────────────┐  │
│  │  Mission    │    │   UI Renderer    │  │
│  │  Controller │    │  (new thread)    │  │
│  └─────┬───────┘    └────────▲─────────┘  │
│        │   ┌────────────┐    │            │
│        └──►│ DroneTable ├────┘ snapshot   │
│            └────────────┘                 │
│                ▲                          │
│                │ update                   │
│        ┌───────┴────────┐                 │
│        │ Worker Threads │                 │
│        └────────────────┘                 │
│                                           │
│  ┌────────────────────────────────────┐   │
│  │  Event Bus (ring buffer + mutex)   │◄──┼── log_msg()
│  └────────────────────────────────────┘   │   (TX/RX/Phase events)
└───────────────────────────────────────────┘
```

### 2.2 기존 코드와의 결합
- 기존 `log_msg()` 는 **stdout 출력 + 이벤트 버퍼 push** 두 가지를 수행하도록 확장.
- 기존 `drone_table_t` 는 변경 없이 스냅샷 인터페이스만 활용.
- UI 모듈은 **읽기 전용 옵저버** — 미션 로직에 영향 없음.

---

## 3. 화면 레이아웃

### 3.1 서버 TUI (Primary)

권장 터미널 크기: **100 cols × 40 rows** (최소 80×30).

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│ DRONE CONTROL CENTER         Phase: PHASE1_GO        Elapsed: 00:23   Drones: 3 │ ← 헤더 (1행)
├──────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│   y=200m ┌────────────────────────────────────────────────────────────────┐     │ ← 맵
│          │                                                                │     │   (16~20행)
│          │                          · · ·                                 │     │
│          │                       ·         ·                              │     │
│          │                     ·   ② ───    ·     ← phase1 target circle │     │
│          │                    ·       \      ·                            │     │
│          │                    ·   ① • ●  ·                               │     │   ① = drone 1
│          │                     ·    /     ·                               │     │   ● = base proj
│          │                       ·  ③    ·                                │     │
│          │                          · · ·                                 │     │
│          │                                                                │     │
│   y= 20m │                              ▲                                 │     │
│          └──────────────────────────────│─────────────────────────────────┘     │
│   x=-100m                          Base Station                       x=+100m   │ ← 축 라벨
├──────────────────────────────────────────────────────────────────────────────────┤
│ DRONES                                                                           │ ← 테이블
│  ID │ Mode  │    X    │    Y    │ Target           │ Last Seen │ State          │   (5~7행)
│  ───┼───────┼─────────┼─────────┼──────────────────┼───────────┼─────           │
│   1 │ GOTO  │   -3.21 │   95.40 │ ( -7.50, 113.00) │  0.4s ago │ moving         │
│   2 │ HOLD  │   15.00 │  100.00 │ ( 15.00, 100.00) │  0.1s ago │ arrived ✓      │
│   3 │ GOTO  │   -8.10 │   82.70 │ ( -7.50,  87.00) │  0.3s ago │ moving         │
├──────────────────────────────────────────────────────────────────────────────────┤
│ MESSAGE LOG                                                              [↓ tail]│ ← 로그
│  14:05:21.103  TX  MOVE_CMD     drone=1  →  (-7.50, 113.00)                     │   (10~14행)
│  14:05:21.205  RX  POS_REPORT   drone=2     (12.41,  99.13)                     │
│  14:05:21.302  RX  POS_REPORT   drone=3     ( -6.20,  84.50)                    │
│  14:05:21.400  RX  ARRIVED      drone=2     (15.00, 100.00)                     │
│  14:05:21.412  *** PHASE1_OK ***                                                 │
│  14:05:21.420  TX  MOVE_CMD     drone=1  →  (-57.50, 113.00)                    │
│  ...                                                                             │
└──────────────────────────────────────────────────────────────────────────────────┘
[q] quit   [p] pause log scroll   [c] clear log
```

### 3.2 드론 TUI (Secondary, 간소화)

```
┌──── Drone 1 ────────────────────────────────────────────────────┐
│ Mode: GOTO       Pos: (-3.21,  95.40)       Target: (-7.50, 113)│
│ Distance to target: 18.42 m                                     │
├─────────────────────────────────────────────────────────────────┤
│ MESSAGE LOG                                                     │
│ 14:05:20.500  TX  HELLO                                         │
│ 14:05:20.510  RX  HELLO_ACK                                     │
│ 14:05:21.012  TX  POS_REPORT  (-15.20,  88.30)                  │
│ 14:05:21.103  RX  MOVE_CMD    →  (-7.50, 113.00)                │
│ 14:05:21.512  TX  POS_REPORT  (-13.80,  90.15)                  │
│ ...                                                             │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. 좌표 변환 (World ↔ Screen)

### 4.1 영역 정의
- 월드: `x ∈ [-100, +100]`, `y ∈ [20, 200]` (m)
- 맵 화면: `MAP_W` 컬럼 × `MAP_H` 행 (예: 64 × 18)

### 4.2 변환식
```c
int world_to_col(double x) {
    return (int)round((x - AREA_X_MIN) / (AREA_X_MAX - AREA_X_MIN)
                      * (MAP_W - 1));
}
int world_to_row(double y) {
    /* y가 클수록 위쪽 → row 번호는 작아짐 */
    return (int)round((AREA_Y_MAX - y) / (AREA_Y_MAX - AREA_Y_MIN)
                      * (MAP_H - 1));
}
```

### 4.3 종횡비 주의
- 셀 비율(폭:높이 ≈ 1:2)로 인해 원이 타원으로 보임 → 시각적 보정으로
  반경 그릴 때 `dx_per_col`, `dy_per_row` 분리 계산.

---

## 5. 시각 요소 (Glyphs & Colors)

### 5.1 심볼 가이드

| 요소 | 심볼 (ASCII) | 우선 순위 | 비고 |
|------|--------------|----------|------|
| 드론 | `①` `②` `③` … (또는 `1` `2` `3`) | 최상위 | ID 식별 |
| 드론 목표점 | `+` 또는 `×` | 중간 | 옅은 색 |
| Phase1 집결 원 (반경 30m) | `·` (점선 원주) | 하위 | 가이드 |
| Phase1 목표 원 (반경 15m) | `· · ·` (얇은 점선) | 하위 | 가이드 |
| 기지국 | `▲` 또는 `📡` | 고정 | 항상 (0, 0) |
| 맵 테두리 | `┌─┐│└─┘` | 고정 | 박스 |
| 빈 셀 | ` ` (스페이스) | — | — |

### 5.2 색상 팔레트

| 항목 | ncurses 색상 | 의미 |
|------|--------------|------|
| 드론 1 / 2 / 3 | CYAN / YELLOW / MAGENTA | 식별 구분 |
| 드론 (이동 중) | bold | 강조 |
| 드론 (도착) | dim | 비활성 |
| 목표 원·점 | DEFAULT (흰색) | 보조 |
| TX 로그 | GREEN | 송신 |
| RX 로그 | BLUE | 수신 |
| Phase 전이 | YELLOW + bold | 미션 이벤트 |
| 경고/오류 | RED + bold | 이상 상황 |
| 헤더 / 축 라벨 | WHITE + dim | 정보 |

> 256색 미지원 터미널 대비 fallback: `has_colors()` 가 false 면 모두 monochrome.

---

## 6. UI 모듈 구조

```
server/
├── ui.h              # 공개 API
├── ui.c              # ncurses 렌더링, UI 쓰레드 진입점
└── event_bus.h/c     # 로그 이벤트 링버퍼 (UI ↔ log_msg 통합)
client/
└── drone_ui.h/c      # 드론 TUI (선택)
```

### 6.1 `event_bus.h` (공통)
```c
typedef enum {
    EV_TX, EV_RX, EV_PHASE, EV_INFO, EV_WARN
} ev_kind_t;

typedef struct {
    long      ts_ms;          /* epoch ms */
    ev_kind_t kind;
    char      text[160];
} log_event_t;

void event_bus_init(int capacity);
void event_bus_push(ev_kind_t kind, const char *fmt, ...);
int  event_bus_snapshot(log_event_t *out, int max);  /* 최신 N건 복사 */
```

### 6.2 `ui.h`
```c
typedef struct {
    drone_table_t  *table;
    mission_phase_t *phase_ptr;   /* 미션 컨트롤러가 갱신 */
    long            start_ts_ms;
} ui_ctx_t;

void  ui_init(void);
void  ui_shutdown(void);
void *ui_thread(void *arg);       /* arg = ui_ctx_t* */
```

### 6.3 통합 포인트
- `log_msg()` 내부에서 `event_bus_push(...)` 호출 추가.
- `main()` 에서 ncurses 초기화 → UI 쓰레드 생성 → 미션 진행 → 종료 시 정리.
- 기존 stdout 로그는 파일(`server.log`)로 리다이렉트 (TUI 화면과 충돌 방지).

---

## 7. UI 쓰레드 모델

```
┌── main thread ───────────────────────┐
│  ui_init() (ncurses setup)            │
│  pthread_create(ui_tid, ui_thread)    │
│  ... 미션 진행 ...                    │
│  pthread_join(ui_tid)                 │
│  ui_shutdown()                        │
└──────────────────────────────────────┘
        │
        ▼
┌── ui_thread ─────────────────────────┐
│ loop @ 10 Hz:                         │
│   1) snapshot = table_snapshot()      │
│   2) events   = event_bus_snapshot()  │
│   3) phase    = *phase_ptr            │
│   4) draw_header(phase, elapsed)      │
│   5) draw_map(snapshot)               │
│   6) draw_drone_table(snapshot)       │
│   7) draw_log(events)                 │
│   8) refresh()                        │
│   9) check key (q → set g_shutdown)   │
└──────────────────────────────────────┘
```

### 7.1 갱신 주기
- **10 fps** (100ms 주기). 너무 빠르면 깜빡임/CPU 낭비, 느리면 끊김.
- `nodelay(stdscr, TRUE)` + `wgetch()` 로 논블로킹 키 입력.

### 7.2 동기화
- UI 쓰레드는 **읽기만 함**. 락 경합 최소화 위해:
  - drone_table 의 기존 `table_snapshot()` (락 잡고 복사) 그대로 사용.
  - event_bus 도 push/snapshot 내부에서 락 처리.

---

## 8. 이벤트 소싱 (Event Sourcing)

### 8.1 메시지 송수신 이벤트
| 발생 위치 | 호출 | 카테고리 |
|----------|------|---------|
| `worker_thread`: POS_REPORT 수신 시 | `event_bus_push(EV_RX, "POS_REPORT drone=%d (%.2f, %.2f)", ...)` | RX (BLUE) |
| `worker_thread`: ARRIVED 수신 시 | `event_bus_push(EV_RX, "ARRIVED drone=%d (...)" , ...)` | RX (BLUE) |
| `send_move_cmd()` | `event_bus_push(EV_TX, "MOVE_CMD drone=%d → (...)", ...)` | TX (GREEN) |
| `send_terminate()` | `event_bus_push(EV_TX, "TERMINATE drone=%d", ...)` | TX (GREEN) |
| `wait_for_arrival` 완료 시 | `event_bus_push(EV_PHASE, "PHASE1_OK")` | PHASE (YELLOW) |

### 8.2 링버퍼 정책
- 용량: **최근 200건** 유지 (메모리 ~32KB).
- 화면에는 최신 12~14건만 표시 (가시 영역 기준).
- 일시정지 모드(키 `p`)에서는 push 는 계속, 표시는 동결.

---

## 9. 키 바인딩

### 9.1 서버 TUI
| 키 | 동작 |
|----|------|
| `q` | 강제 종료 (g_shutdown = 1) |
| `p` | 로그 스크롤 일시정지 / 재개 |
| `c` | 로그 화면만 클리어 (이벤트 버퍼는 유지) |
| `+` / `-` | 로그 표시 줄 수 증감 (선택) |

### 9.2 드론 TUI
| 키 | 동작 |
|----|------|
| `q` | 종료 |
| `c` | 로그 클리어 |

---

## 10. 렌더링 알고리즘

### 10.1 맵 그리기 (`draw_map`)
```
1. 맵 영역 박스 외곽 출력
2. 기지국 위치 (x=0, y=20 가까이) 에 ▲ 출력
3. Phase1 목표 원 (반경 15m) 점선으로 출력
   for θ in [0, 2π) step 0.1:
     wx = 0 + 15·cos θ
     wy = 100 + 15·sin θ
     plot('·', world_to_col(wx), world_to_row(wy))
4. (선택) 드론 i 의 현재 → 목표 직선 (옅은 색 점선)
5. 각 드론을 ID 글리프로 plot, 색상은 drone_id_to_color(id)
   - 같은 셀에 둘 이상 → 마지막에 덮어쓰는 대신 '*' 로 표시
```

### 10.2 드론 테이블 (`draw_drone_table`)
- ID 정렬 후 행별 출력.
- "Last Seen" 은 현재 시각 - 마지막 POS_REPORT ts (ms 단위로 보관).
- 도착 여부는 `|pos - target| < tolerance` 로 표시.

### 10.3 로그 (`draw_log`)
- 최신 이벤트가 아래로 가도록 (tail-style).
- 각 이벤트의 `ev_kind_t` 에 따라 색상 attr 적용.
- 타임스탬프는 `HH:MM:SS.mmm` 형식.

---

## 11. 데이터 흐름 예시

```
worker_thread receives POS_REPORT
  → table_update_pos()           (테이블 갱신)
  → log_msg("RX POS_REPORT ...")
       └─► event_bus_push(EV_RX, ...)
                 └─► (10ms 후 다음 UI tick) draw_log() 가 가져가 출력

ui_thread tick
  ├─ table_snapshot() → 드론들의 현재 위치
  ├─ event_bus_snapshot() → 최근 이벤트 14건
  └─ erase() → draw_*() → refresh()
```

---

## 12. 빌드 / 실행 변경

### 12.1 Makefile 변경
```makefile
LDFLAGS = -pthread -lm -lncurses
SERVER_OBJ += server/ui.o server/event_bus.o
```
- macOS: `-lncurses` 그대로 동작.
- Linux: 일부 배포판은 `-lncursesw` 필요할 수 있음 (UTF-8 기호 사용 시).

### 12.2 실행
```bash
./bin/server 9000 3            # 기본 (TUI)
./bin/server 9000 3 --no-tui   # 기존 stdout 모드 (디버깅용 옵션)
```
- 디버깅용 stdout 모드는 평가/데모와 별개로 유지 → 회귀 테스트 용이.

---

## 13. 위험 요소 및 완화

| 위험 | 영향 | 완화 |
|------|------|------|
| 작은 터미널(80×24) | 레이아웃 깨짐 | `getmaxyx()` 로 크기 확인, 최소치 미만이면 안내 메시지 |
| stdout 출력과 ncurses 화면 충돌 | 화면 깨짐 | TUI 모드에서는 `printf` 차단, `log_msg` 가 stdout 미사용 |
| UI 쓰레드 락 경합 | 미션 지연 | UI 는 읽기만, 짧은 락 구간 |
| 종료 시 ncurses 정리 미흡 | 터미널 깨짐 | `atexit(ui_shutdown)` 등록, SIGINT 핸들러에서 endwin() |
| 한글/UTF-8 글리프 깨짐 | 가독성 저하 | `setlocale(LC_ALL, "")` + `ncursesw` 또는 ASCII fallback |
| 색상 미지원 환경 | 식별 어려움 | `has_colors()` 검사 후 attr 미적용 |
| 데모 녹화 시 폰트 가변폭 | 정렬 깨짐 | 등폭 폰트(Menlo, Consolas) 권장 가이드 |

---

## 14. 데모 시나리오 활용

UI 가 있을 때 데모 동영상 흐름이 더 명확해진다:

1. 서버 실행 — 빈 맵 + "Phase: INIT, Drones: 0"
2. 드론 1, 2, 3 순차 실행 — 맵에 드론 글리프 등장, 상태 테이블 채워짐
3. 모두 접속 → "Phase: PHASE1_GO" 전이 (헤더 색상 변화)
4. MOVE_CMD 송신 라인이 GREEN 으로 로그에 흐름
5. 드론들이 맵에서 phase1 목표 원으로 수렴하는 모습 시각화
6. "Phase: PHASE1_OK" 전이 (yellow flash)
7. PHASE2_GO → 좌측 50m 평행 이동이 맵에서 시각적으로 슬라이드
8. PHASE2_OK → DONE → TERMINATE → 드론 글리프가 맵에서 사라짐

---

## 15. 평가 기준 매핑

| 평가 항목 | UI 의 기여 |
|-----------|-----------|
| UI 우수성 (드론 움직임 가시화) | §3 맵 + §10.1 알고리즘 |
| UI 우수성 (메시지 송수신 가시화) | §3 로그 영역 + §5.2 TX/RX 색상 |
| 제어 난이도(메시지 종류) | §8 이벤트 카테고리별 색상 → 시각적으로 다양성 부각 |
| 데모 동영상 내용 | §14 시나리오 활용 |
| 결과보고서 내용 | §3 캡쳐 이미지 다수 확보 가능 |

---

## 16. 후속 작업 (Open Items)

- [ ] `event_bus.h/c` 구현 (ring buffer + mutex)
- [ ] `ui.h/c` 구현 (ncurses 초기화, draw_* 함수, ui_thread)
- [ ] `log_msg()` 에 `event_bus_push()` 통합
- [ ] `Makefile` 에 `-lncurses` 추가
- [ ] `--no-tui` 옵션 처리
- [ ] (선택) 드론 측 `drone_ui.c` 구현
- [ ] 80×24 미만 터미널 안내 메시지
- [ ] SIGINT 시 `endwin()` 정리 보장
- [ ] 데모 녹화용 컬러 테마 점검 (어두운 배경 vs 밝은 배경)
