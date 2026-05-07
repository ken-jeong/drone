# 드론 관제 시스템 디자인 스펙

> 네트워크 프로그래밍(12) 2026-1학기 프로젝트
> 기지국 서버 ↔ 다중 드론 클라이언트 (TCP, 다중 쓰레드, C언어)

---

## 1. 개요 (Overview)

### 1.1 목적
TCP 기반 다중 쓰레드 클라이언트-서버 구조를 활용하여, 1대의 기지국 서버가 3대 이상의 드론 클라이언트의 위치를 추적하고, 단계적 이동 명령(1차: 집결 / 2차: 좌측 50m 이동)을 통해 협응 동작을 수행하는 시스템을 구현한다.

### 1.2 범위
- **포함**: TCP 통신, 다중 쓰레드 서버, 드론 자율 이동 시뮬레이션, 단계별 미션 제어, 콘솔 기반 UI.
- **제외**: 실제 무선 통신, GUI, 영속 저장소, 인증/암호화.

### 1.3 제약
| 항목 | 값 |
|------|----|
| 언어 | C (C99 이상) |
| 전송 프로토콜 | TCP |
| 동시성 모델 | POSIX Threads (`pthread`) |
| 드론 수 | ≥ 3 |
| 영역(폭) | x ∈ [-100, +100] m (서버 기준 좌우) |
| 영역(높이) | y ∈ [20, 200] m |
| 1차 집결점 | 중심 (0, 100), 반경 ≤ 30 m, 드론 간 간격 ≥ 10 m |
| 2차 이동 | 각 드론이 좌측(-x)으로 50 m 이동 |

---

## 2. 시스템 아키텍처 (System Architecture)

### 2.1 컴포넌트 구성도
```
┌─────────────────────────────────────────────────┐
│                   기지국 서버                     │
│  ┌──────────────┐    ┌──────────────────────┐   │
│  │ Mission Ctrl │◄──►│   Drone Table        │   │
│  │  (메인 쓰레드)│    │  (mutex 보호)        │   │
│  └──────┬───────┘    └──────────────────────┘   │
│         │                  ▲                    │
│         │                  │ update             │
│         ▼                  │                    │
│  ┌──────────────┬──────────┴──────┬──────────┐  │
│  │ Worker T#1   │ Worker T#2      │ Worker T#3│  │
│  │ (Drone1 sock)│ (Drone2 sock)   │ (Drone3..)│ │
│  └──────┬───────┴──────┬──────────┴────┬─────┘  │
└─────────┼──────────────┼───────────────┼────────┘
          │ TCP          │ TCP           │ TCP
          ▼              ▼               ▼
     ┌─────────┐    ┌─────────┐    ┌─────────┐
     │ Drone 1 │    │ Drone 2 │    │ Drone 3 │
     │ ┌─────┐ │    │ ┌─────┐ │    │ ┌─────┐ │
     │ │Move │ │    │ │Move │ │    │ │Move │ │
     │ │Recv │ │    │ │Recv │ │    │ │Recv │ │
     │ └─────┘ │    │ └─────┘ │    │ └─────┘ │
     └─────────┘    └─────────┘    └─────────┘
```

### 2.2 책임 분담
| 컴포넌트 | 책임 |
|----------|------|
| **Mission Controller** (서버 메인) | 단계 전이 판정, broadcast 명령 발행, 종료 결정 |
| **Worker Thread** (서버, 클라이언트당 1개) | 해당 드론으로부터 `POS_REPORT` 수신, 테이블 갱신 |
| **Drone Table** (서버) | 드론 ID → {소켓, 위치, 상태} 맵, 뮤텍스 보호 |
| **Drone Move Thread** (클라이언트) | 주기적으로 자기 위치 갱신 (random walk 또는 goto) |
| **Drone Recv Thread** (클라이언트) | 서버로부터 명령 수신, 이동 모드 변경 |

---

## 3. 도메인 모델 (Domain Model)

### 3.1 핵심 자료구조

```c
typedef enum {
    DRONE_RANDOM,     // 초기: 랜덤 워크
    DRONE_GOTO,       // 목표 좌표로 이동 중
    DRONE_HOLD,       // 목표 도착 후 대기
    DRONE_TERMINATED  // 종료
} drone_mode_t;

typedef struct {
    int     id;
    double  x, y;            // 현재 위치 (m)
    double  target_x, target_y;
    drone_mode_t mode;
} drone_state_t;

// 서버 측 드론 엔트리
typedef struct {
    int             id;
    int             socket_fd;
    double          x, y;
    drone_mode_t    mode;
    int             phase1_done;
    int             phase2_done;
    pthread_mutex_t send_lock;  // 소켓별 송신 직렬화
} drone_entry_t;

typedef struct {
    drone_entry_t   drones[MAX_DRONES];
    int             count;
    pthread_mutex_t table_lock;
    pthread_cond_t  state_changed;
} drone_table_t;
```

### 3.2 미션 단계
```c
typedef enum {
    MISSION_INIT,         // 클라이언트 접속 대기
    MISSION_PHASE1_GO,    // 1차 이동 명령 발행됨
    MISSION_PHASE1_OK,    // 모든 드론 집결 검증 완료
    MISSION_PHASE2_GO,    // 2차 이동 명령 발행됨
    MISSION_PHASE2_OK,    // 모든 드론 좌측 50m 이동 완료
    MISSION_DONE
} mission_phase_t;
```

---

## 4. 메시지 프로토콜 (Wire Protocol)

### 4.1 프레임 포맷
모든 메시지는 **고정 헤더 + 페이로드** 구조로 송수신한다. TCP 스트림에서 메시지 경계를 식별하기 위해 길이 prefix를 사용한다.

```
 0       1       3                                   N
 +-------+-------+-----------------------------------+
 | TYPE  | LEN   |          PAYLOAD (LEN bytes)      |
 | (1B)  | (2B)  |                                   |
 +-------+-------+-----------------------------------+
```
- `TYPE` (1 byte): 메시지 종류
- `LEN`  (2 bytes, network byte order): 페이로드 길이
- 모든 수치 필드는 네트워크 바이트 오더(`htons`/`htonl`) 사용

### 4.2 메시지 타입

| 코드 | 이름 | 방향 | 페이로드 | 의미 |
|------|------|------|----------|------|
| 0x01 | `MSG_HELLO` | C→S | `int32 drone_id` | 접속 신고 |
| 0x02 | `MSG_HELLO_ACK` | S→C | `int32 assigned_id` | 접속 승인 |
| 0x10 | `MSG_POS_REPORT` | C→S | `int32 id, int32 x_mm, int32 y_mm` | 주기 위치 보고 |
| 0x20 | `MSG_MOVE_CMD` | S→C | `int32 target_x_mm, int32 target_y_mm` | 목표 좌표 이동 명령 |
| 0x21 | `MSG_ARRIVED` | C→S | `int32 id, int32 x_mm, int32 y_mm` | 목표 도착 통지 |
| 0x30 | `MSG_PHASE_ACK` | S→C | `int32 phase` | 단계 완료 통지 (정보용) |
| 0xFF | `MSG_TERMINATE` | S→C | (empty) | 종료 명령 |

> 좌표는 정수(밀리미터 단위)로 송신하여 부동소수점 직렬화 이슈를 회피한다.

### 4.3 메시지 흐름 (Sequence)

```
Client                              Server
  │                                   │
  │ ──── HELLO(id=1) ──────────────► │
  │ ◄──── HELLO_ACK ──────────────── │
  │                                   │
  │ ──── POS_REPORT ────────────────►│  (every 1s, RANDOM 모드)
  │ ──── POS_REPORT ────────────────►│
  │ ──── POS_REPORT ────────────────►│
  │                                   │ [모든 드론 접속 확인 →
  │                                   │  Mission: PHASE1_GO]
  │ ◄──── MOVE_CMD(target1) ─────────│
  │ ──── POS_REPORT ────────────────►│  (이동 중, GOTO 모드)
  │ ──── POS_REPORT ────────────────►│
  │ ──── ARRIVED ───────────────────►│
  │                                   │ [반경/간격 검증 →
  │                                   │  Mission: PHASE1_OK → PHASE2_GO]
  │ ◄──── MOVE_CMD(target1.x-50) ────│
  │ ──── POS_REPORT ────────────────►│
  │ ──── ARRIVED ───────────────────►│
  │                                   │ [모두 도착 → PHASE2_OK]
  │ ◄──── TERMINATE ─────────────────│
  │ (close)                           │
```

---

## 5. 상태 기계 (State Machine)

### 5.1 서버 미션 상태 전이
```
            모든 드론 접속 완료
   INIT ─────────────────────► PHASE1_GO
                                  │
                  모든 드론이 (0,100) 반경 30m 내 +
                  드론 간 간격 ≥ 10m 충족
                                  ▼
                              PHASE1_OK
                                  │ (자동)
                                  ▼
                              PHASE2_GO
                                  │
                  모든 드론이 1차 위치 -50m 도달
                                  ▼
                              PHASE2_OK
                                  │ (TERMINATE 발송)
                                  ▼
                                DONE
```

### 5.2 드론 동작 상태 전이
```
   START ──► RANDOM ──(MOVE_CMD 수신)──► GOTO ──(target 도달)──► HOLD
                                          ▲                       │
                                          │                       │
                                          └──(MOVE_CMD 재수신)────┘
                                                                  │
                                          (TERMINATE 수신)        │
                                                ▼                 ▼
                                            TERMINATED ◄──────────┘
```

---

## 6. 동시성 설계 (Concurrency Design)

### 6.1 쓰레드 구성

**서버 측**
- `main thread`: 미션 컨트롤러. 드론 테이블을 폴링(또는 cond_wait)하여 단계 판정 및 broadcast.
- `accept thread` (선택): listen 소켓에서 신규 연결을 수락하고 worker 쓰레드 생성.
- `worker thread × N`: 각 드론 소켓에서 `recv()` 루프 수행. 수신한 `POS_REPORT`/`ARRIVED`를 테이블에 반영.

**클라이언트 측**
- `main / move thread`: 100~500ms 주기로 좌표 갱신 후 `POS_REPORT` 송신.
- `recv thread`: 서버 명령 수신 및 모드/목표 좌표 갱신.

### 6.2 동기화 정책

| 자원 | 락 | 보호 대상 |
|------|----|-----------|
| `drone_table_t.table_lock` | `pthread_mutex_t` | drones 배열, count, 각 엔트리의 위치/상태 |
| `drone_entry_t.send_lock` | `pthread_mutex_t` | 해당 드론 소켓 `send()` 직렬화 |
| `drone_table_t.state_changed` | `pthread_cond_t` | 미션 컨트롤러가 상태 변화 대기 |

**클라이언트 측**: `current_state` 보호용 단일 뮤텍스 (move/recv 쓰레드 공용).

### 6.3 락킹 규약 (중요)
> **send() 호출은 절대 `table_lock`을 잡은 상태에서 하지 않는다.**
> 이유: 네트워크 블로킹으로 인한 전체 서버 정지 방지.

권장 패턴:
```c
pthread_mutex_lock(&table.table_lock);
// 1) 송신 대상 목록을 로컬 배열로 복사
snapshot = copy_targets(&table);
pthread_mutex_unlock(&table.table_lock);

// 2) 락 밖에서, 각 entry의 send_lock만 잡고 송신
for (each target in snapshot) {
    pthread_mutex_lock(&target->send_lock);
    send_message(target->socket_fd, ...);
    pthread_mutex_unlock(&target->send_lock);
}
```

---

## 7. 검증 로직 (Validation)

### 7.1 1차 이동 완료 판정
```
모든 드론 i에 대해:
  dist((x_i, y_i), (0, 100)) ≤ 30 + ε

모든 드론 쌍 (i, j), i ≠ j에 대해:
  dist((x_i, y_i), (x_j, y_j)) ≥ 10 - ε
```
- ε(허용 오차): 0.5 m 권장 (정수 mm 좌표라면 500)

### 7.2 목표 좌표 할당 알고리즘 (1차)
중심 (0, 100), 반경 약 15m의 원주에 균등 배치:
```
N = 드론 수
for i in 0..N-1:
    θ = 2π · i / N
    target_x[i] = 0   + 15 · cos(θ)
    target_y[i] = 100 + 15 · sin(θ)
```
N=3일 때 인접 거리 ≈ 25.98 m로 10m 제약 충족.

### 7.3 2차 이동 목표
```
target2_x[i] = target1_x[i] - 50
target2_y[i] = target1_y[i]
```

### 7.4 도착 판정 (클라이언트)
```
|x - target_x| < 0.5  AND  |y - target_y| < 0.5
```

### 7.5 점진적 이동
한 틱당 최대 이동 거리 `STEP = 2.0 m`로 제한:
```
dx = target_x - x;  dy = target_y - y;
d  = sqrt(dx² + dy²);
if (d < STEP) { x = target_x; y = target_y; }
else          { x += STEP·dx/d; y += STEP·dy/d; }
```

---

## 8. 모듈 / 파일 구조

```
network_v2/
├── DESIGN_SPEC.md          # 본 문서
├── Makefile
├── common/
│   ├── protocol.h          # 메시지 타입, 프레임 함수 시그니처
│   ├── protocol.c          # send_msg / recv_msg (프레이밍 처리)
│   └── config.h            # 상수 (포트, 영역, 임계값)
├── server/
│   ├── server_main.c       # main, accept loop, 미션 컨트롤러
│   ├── drone_table.h/c     # 테이블 자료구조 + 동기화
│   └── mission.h/c         # 단계 전이 로직, 검증 함수
└── client/
    └── drone_main.c        # 드론 클라이언트 (move + recv 쓰레드)
```

---

## 9. 핵심 API 시그니처

### 9.1 protocol.c
```c
// 한 번의 send로 한 메시지 전체를 송신 (write-all)
int send_msg(int fd, uint8_t type, const void *payload, uint16_t len);

// 헤더 → 페이로드 순서로 정확히 한 메시지를 수신 (read-all)
// 반환: 0 정상, -1 오류, -2 상대측 종료
int recv_msg(int fd, uint8_t *type, void *buf, uint16_t bufsize, uint16_t *len);
```

### 9.2 drone_table.c
```c
int  table_add(drone_table_t *t, int id, int sock);
int  table_remove(drone_table_t *t, int id);
int  table_update_pos(drone_table_t *t, int id, double x, double y);
int  table_snapshot(drone_table_t *t, drone_entry_t *out, int max);
```

### 9.3 mission.c
```c
int  check_phase1_complete(const drone_entry_t *snap, int n);
int  check_phase2_complete(const drone_entry_t *snap, int n,
                            const double *phase1_target_x);
void compute_phase1_targets(int n, double *tx, double *ty);
```

---

## 10. UI 설계 (콘솔)

### 10.1 서버 화면 (주기 출력 권장)
```
========================================================
[Mission Phase] PHASE1_GO   (elapsed: 12.3s)
--------------------------------------------------------
 ID | Mode    |   X     |   Y     | Target          | Phase
----+---------+---------+---------+-----------------+------
  1 | GOTO    |  -3.2   |   95.4  | (-7.5,  113.0) |  -
  2 | GOTO    |   8.1   |  102.7  | ( 15.0, 100.0) |  -
  3 | HOLD    |  -7.5   |  113.0  | (-7.5,  113.0) | P1✓
========================================================
[LOG] 2026-05-07 14:23:01  RX  POS_REPORT  drone=1  (-3.2, 95.4)
[LOG] 2026-05-07 14:23:01  TX  MOVE_CMD    drone=2  -> (15.0, 100.0)
```

### 10.2 클라이언트 화면
```
[Drone 1] mode=GOTO  pos=(-3.2, 95.4)  target=(-7.5, 113.0)
[Drone 1] TX POS_REPORT
[Drone 1] RX MOVE_CMD -> (-7.5, 113.0)
[Drone 1] arrived. switching to HOLD.
```

---

## 11. 위험 요소 및 완화 전략

| 위험 | 영향 | 완화 |
|------|------|------|
| TCP 메시지 단편화 | 잘못된 파싱 | 길이 prefix 프레이밍 + read-all 루프 |
| 락을 잡은 채 send() | 데드락/멈춤 | 스냅샷 패턴 (§6.3) |
| 동일 소켓 동시 send | 데이터 깨짐 | 엔트리별 `send_lock` |
| 부동소수 비교 | 무한 대기 | 톨러런스 ε 도입 |
| 비정상 종료 | 자원 누수 | SIGINT 핸들러에서 close + join |
| accept 블로킹 중 종료 | 깨끗한 종료 어려움 | listen 소켓에 `SO_REUSEADDR`, shutdown 패턴 |
| 좌표 영역 이탈 | 요구사항 위반 | 클라이언트가 매 틱 clamp |

---

## 12. 빌드 / 실행 (예정)

```bash
# 빌드
make

# 서버 실행 (포트 9000)
./server 9000

# 드론 클라이언트 실행 (서버주소, 포트, 드론ID)
./drone 127.0.0.1 9000 1
./drone 127.0.0.1 9000 2
./drone 127.0.0.1 9000 3
```

---

## 13. 평가 기준 매핑

| 평가 항목 | 본 설계의 대응 |
|-----------|---------------|
| 요구사항 이행 | §1.3 제약 전수 반영, §7 검증 로직, §5 단계 전이 |
| UI 우수성 | §10 테이블/로그 분리 출력 |
| 제어 난이도 | §4 7종 메시지, §5 단계 전이 프로토콜, ACK 기반 동기화 |
| 실행 안정성 | §6.3 락킹 규약, §11 위험 완화 |
| 데모/보고서 | §4.3 시퀀스 다이어그램, §10 UI 설계 그대로 활용 가능 |

---

## 14. 후속 작업 (Open Items)
- [ ] `Makefile` 및 디렉터리 스켈레톤 생성
- [ ] `protocol.c` (프레이밍) 구현
- [ ] 서버 accept/worker 루프 구현
- [ ] 미션 컨트롤러 구현
- [ ] 드론 클라이언트 구현
- [ ] 통합 테스트 (드론 3대)
- [ ] 데모 시나리오 스크립트 작성
- [ ] 결과보고서 초안
