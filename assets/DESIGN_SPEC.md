# 드론 군집 제어 시스템 — 소프트웨어 디자인 스펙 (SDS)

> 네트워크프로그래밍(12) 프로젝트 / 2026학년도 1학기
> 기준 예제: 『윤성우의 열혈 TCP/IP 소켓 프로그래밍』 Chapter20 `chat_serv_win.c` · `chat_clnt_win.c`
> 플랫폼: Windows / Winsock 2.2 (`ws2_32.lib`) / 언어: C / 전송: TCP / 동시성: Win32 멀티쓰레드(`_beginthreadex` + `CreateMutex`)
> 작성일: 2026-06-07

---

## 1. 목적 및 범위

기지국 서버(1대)가 TCP로 접속한 **N(≥3)대의 드론 클라이언트**를 제어한다.
드론은 주기적으로 자신의 위치를 보고하고, 서버는 군집 제어 명령을 내려
**1차 이동(집결)** → **2차 이동(좌측 50m)** → **종료**의 페이즈를 진행한다.

본 스펙은 기준 채팅 예제를 어떻게 **재사용·변형**하여 요구사항을 충족하는지를 정의하며,
**(A) Winsock API 계층**과 **(B) 실시간 시각화**를 구현 수준까지 구체화한다.

---

## 2. 기준 예제와의 매핑 (재사용 전략)

| 채팅 예제 요소 | 본 프로젝트로의 변형 |
|----------------|----------------------|
| `SOCKET clntSocks[MAX_CLNT]` 배열 | `DroneState drones[MAX_DRONE]` 상태 테이블로 확장 (소켓 + 위치 + 상태) |
| `hMutex` (전역 1개) | 그대로 사용. 드론 상태 테이블의 임계구역 보호 |
| `_beginthreadex(HandleClnt)` 연결당 쓰레드 | `DroneHandler` 쓰레드로 변형: 위치 보고 수신 전담 |
| `SendMsg()` 전체 브로드캐스트 | `SendCmd(droneId, ...)` 개별 유니캐스트 명령으로 변형 |
| 클라이언트 송/수신 2쓰레드 | 그대로 유지: 송신=위치보고, 수신=명령처리 |
| 텍스트 라인 메시지 | **고정 길이 구조체 메시지**로 변형 (framing 해결) |
| (없음) | **서버에 제어 쓰레드(ControlLoop) 신규 추가** — 페이즈 관리 |
| (없음) | **서버에 렌더링 쓰레드(RenderLoop) 신규 추가** — 실시간 시각화 |

> 신규 요소는 둘: **중앙 제어 쓰레드(ControlLoop)** 와 **시각화 쓰레드(RenderLoop)**. 나머지는 기존 골격의 확장.

---

## 3. 시스템 아키텍처

```
                      기지국 서버 (server.c)
   ┌────────────────────────────────────────────────────────────────┐
   │  main: accept 루프  ── 연결당 ──▶  DroneHandler 쓰레드 ×N          │
   │                                      (POSITION_REPORT 수신)       │
   │                                            │ g_mutex             │
   │                                            ▼                     │
   │                    ┌─────────────────────────────────┐          │
   │                    │  공유 상태: DroneState drones[]   │          │
   │                    │            SystemPhase phase      │          │
   │                    └─────────────────────────────────┘          │
   │                          ▲ g_mutex          │ g_mutex            │
   │                          │                  ▼                    │
   │  ControlLoop 쓰레드(신규): 평가→MOVE_CMD    RenderLoop 쓰레드(신규):│
   │                          유니캐스트          상태 읽어 레이더 렌더 │
   └────────────────────────────────────────────────────────────────┘
          ▲ TCP            ▲ TCP            ▲ TCP
   ┌──────┴─────┐   ┌──────┴─────┐   ┌──────┴─────┐
   │ drone.c #1 │   │ drone.c #2 │   │ drone.c #3 │  (각: 송신=보고, 수신=명령)
   └────────────┘   └────────────┘   └────────────┘
```

### 3.1 서버 쓰레드 구성
- **Acceptor (main)**: `accept()` 후 드론 등록, `DroneHandler` 생성. (예제 그대로)
- **DroneHandler[i]**: 드론 i의 `POSITION_REPORT`를 루프 수신 → 상태 테이블 갱신. (`HandleClnt` 변형)
- **ControlLoop (신규)**: 주기적으로 전체 위치를 평가, 페이즈 전이 판정, 목표 명령 송신.
- **RenderLoop (신규)**: 주기적으로 상태를 스냅샷 떠 콘솔 레이더/표/로그를 다시 그림.

### 3.2 클라이언트 쓰레드 구성 (예제 그대로 2쓰레드)
- **ReportThread**: 주기 T마다 현재 위치를 `POSITION_REPORT`로 송신 + 위치 시뮬레이션 1틱 진행.
- **CommandThread**: `MOVE_CMD`/`TERMINATE` 수신 → 목표 좌표 갱신.

---

## 4. 좌표계 및 운동 모델 (명시적 가정)

> 명세가 열어둔 변수 → 본 스펙에서 확정. 결과보고서의 Assumptions로 그대로 사용.

| 항목 | 정의 |
|------|------|
| 좌표계 | 기지국을 원점(0,0). X=너비(좌우), Y=고도. 단위 = m (float) |
| X 범위 | −100 ≤ X ≤ +100 (좌우 폭 100m 해석: 좌우 각 100m → ±100) |
| Y 범위 | 20 ≤ Y ≤ 200 |
| 위치 보고 주기 | 200 ms |
| 제어 루프 주기 | 200 ms |
| 렌더 주기 | 100 ms (10 FPS) |
| 이동 속도(틱당) | 2 m/tick (점진적 이동) |
| 완료 판정 오차 | 목표와의 거리 ≤ 1.0 m 이면 도달로 간주 |
| 좌표 클램핑 | 매 틱 이동 후 범위 밖이면 경계값으로 보정 |

---

# A. Winsock 계층 설계

## 5. Winsock 초기화 / 정리 (서버·클라 공통)

```c
WSADATA wsaData;
if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0)   // 2.2 버전 요청
    ErrorHandling("WSAStartup() error");
... // 소켓 사용
WSACleanup();                                    // 종료 시 1회
```
- 모든 소켓 핸들 타입은 `SOCKET`, 실패 비교는 `INVALID_SOCKET`/`SOCKET_ERROR` 사용.
- 빌드: 프로젝트 링커에 `ws2_32.lib` 추가. `#include <winsock2.h>`(반드시 `windows.h`보다 먼저) + `#include <ws2tcpip.h>`.
- 쓰레드: `#include <process.h>` 후 `_beginthreadex` 사용(예제와 동일).

## 6. 서버 측 Winsock 호출 순서

```c
// (1) 리슨 소켓 생성
hServSock = socket(PF_INET, SOCK_STREAM, 0);     // TCP

// (2) 주소 바인딩
SOCKADDR_IN servAdr = {0};
servAdr.sin_family      = AF_INET;
servAdr.sin_addr.s_addr = htonl(INADDR_ANY);     // 모든 인터페이스
servAdr.sin_port        = htons(atoi(argv[1]));  // 포트 인자
bind(hServSock, (SOCKADDR*)&servAdr, sizeof(servAdr));

// (3) 대기열 오픈
listen(hServSock, 5);

// (4) accept 루프 (예제 골격 그대로)
while (running) {
    int sz = sizeof(clntAdr);
    hClntSock = accept(hServSock, (SOCKADDR*)&clntAdr, &sz);

    WaitForSingleObject(g_mutex, INFINITE);
    int id = RegisterDrone(hClntSock, clntAdr);   // 빈 슬롯에 등록, droneId 부여
    ReleaseMutex(g_mutex);

    _beginthreadex(NULL, 0, DroneHandler, (void*)(intptr_t)id, 0, NULL);
}
```

> 채팅 예제는 `accept` 후 곧장 소켓을 배열에 넣었다. 본 설계는 **위치/상태 필드까지 포함한
> 슬롯에 등록**하고 슬롯 인덱스(=droneId)를 쓰레드 인자로 넘긴다.
> ※ 예제의 `_beginthreadex(..., (void*)&hClntSock, ...)`는 지역변수 주소를 넘겨 **경합 위험**이 있다.
> 본 설계는 **값(슬롯 id)** 을 넘겨 그 결함을 제거한다.

## 7. 클라이언트 측 Winsock 호출 순서

```c
hSock = socket(PF_INET, SOCK_STREAM, 0);
SOCKADDR_IN servAdr = {0};
servAdr.sin_family      = AF_INET;
servAdr.sin_addr.s_addr = inet_addr(argv[1]);    // 서버 IP
servAdr.sin_port        = htons(atoi(argv[2]));  // 서버 포트
connect(hSock, (SOCKADDR*)&servAdr, sizeof(servAdr));

// 송신/수신 2쓰레드 (예제 그대로)
hSnd = (HANDLE)_beginthreadex(NULL, 0, ReportThread,  (void*)&hSock, 0, NULL);
hRcv = (HANDLE)_beginthreadex(NULL, 0, CommandThread, (void*)&hSock, 0, NULL);
WaitForSingleObject(hSnd, INFINITE);
WaitForSingleObject(hRcv, INFINITE);
closesocket(hSock);  WSACleanup();
```

## 8. 메시지 프레이밍 — 신뢰성 있는 송수신 래퍼

TCP는 바이트 스트림이라 한 번의 `recv()`가 메시지 절반/2개를 줄 수 있다.
채팅 예제는 텍스트라 경계를 무시했지만, 본 설계는 **고정 크기 구조체**이므로
`sizeof(Message)`를 **정확히 채울 때까지 누적**하는 래퍼를 둔다.

```c
// 정확히 len 바이트를 받을 때까지 반복 (부분 수신 대응)
int RecvFixed(SOCKET s, char *buf, int len) {
    int got = 0, r;
    while (got < len) {
        r = recv(s, buf + got, len - got, 0);
        if (r == 0 || r == SOCKET_ERROR) return r;   // 연결 종료/오류
        got += r;
    }
    return got;
}
// 정확히 len 바이트를 보낼 때까지 반복
int SendFixed(SOCKET s, const char *buf, int len) {
    int sent = 0, w;
    while (sent < len) {
        w = send(s, buf + sent, len - sent, 0);
        if (w == SOCKET_ERROR) return w;
        sent += w;
    }
    return sent;
}

// 사용 예
Message m;
int r = RecvFixed(sock, (char*)&m, sizeof(m));   // 한 메시지 완성 수신
SendFixed(sock, (const char*)&m, sizeof(m));     // 한 메시지 송신
```

**바이트 순서(엔디안) 가정**: 서버·클라이언트가 동일 PC(또는 동일 아키텍처)에서 데모되므로
구조체 raw 송수신을 사용한다. (보고서 Assumptions에 명시; 이기종 확장 시 `htonl`/`ntohl` + `float` 직렬화 필요.)

## 9. 동기화 (Mutex)

예제의 단일 `hMutex`를 그대로 사용. 보호 대상 = `g_drones[]`, `g_droneCnt`, `g_phase`.
교착 방지 규칙: **임계구역 안에서 `send()`/블로킹 호출을 하지 않는다.**
→ ControlLoop은 잠금 구간에서 "보낼 목표 목록"만 **스냅샷 복사**하고, 잠금 해제 후 `SendFixed`.

```c
// ControlLoop 한 주기
WaitForSingleObject(g_mutex, INFINITE);
SnapshotTargets(localCmd, &n);   // 보낼 (sock,target) 목록만 복사
g_phase = nextPhase;             // 페이즈 갱신
ReleaseMutex(g_mutex);
for (i=0;i<n;i++) SendFixed(localCmd[i].sock, ...);  // 잠금 밖에서 송신
```

---

# B. 통신 프로토콜 / 상태

## 10. 메시지 구조 (server·client 공유 헤더 `protocol.h`)

```c
#define MSG_POSITION_REPORT  1   // 드론 → 서버: 현재 위치
#define MSG_MOVE_CMD         2   // 서버 → 드론: 목표 위치로 이동
#define MSG_ARRIVED          3   // 드론 → 서버: 목표 도달 통보 (선택)
#define MSG_TERMINATE        4   // 서버 → 드론: 종료

typedef struct {
    int   type;        // MSG_* 종류
    int   droneId;     // 드론 식별자
    float x;           // 위치/목표 X (좌우)
    float y;           // 위치/목표 Y (고도)
    int   seq;         // 순서 번호(디버깅/로깅용)
} Message;             // 고정 크기 — RecvFixed/SendFixed로 한 번에 처리
```

## 11. 메시지 시퀀스 (정상 시나리오)

```
드론                          서버
 │ ── POSITION_REPORT ───────▶ │  (접속 직후부터 주기적, random 위치)
 │                             │  [PHASE_INIT]
 │ ◀──── MOVE_CMD(집결목표) ─── │  1차 이동 명령 (드론별 개별 목표)
 │ ── POSITION_REPORT ───────▶ │  이동하며 계속 보고
 │                             │  서버: 전원 반경30m·간격10m 확인 → [PHASE1_DONE]
 │ ◀──── MOVE_CMD(현재X-50) ─── │  2차 이동 명령 (좌측 50m)
 │ ── POSITION_REPORT ───────▶ │  서버: 전원 도달 확인 → [PHASE_DONE]
 │ ◀──────── TERMINATE ─────── │  종료 명령
 │ (소켓 닫고 종료)             │  (소켓 닫고 종료)
```

## 12. 서버 상태 기계 (페이즈)

```
PHASE_INIT ──(드론 ≥3 접속·보고 시작)──▶ 1차 목표 산출·송신
    ▼
PHASE_GATHERING ──(전원 반경30m 내 & 상호간격≥10m)──▶
    ▼
PHASE1_DONE ──(2차 목표=각자 X−50 송신)──▶
    ▼
PHASE_SHIFTING ──(전원 2차목표 도달)──▶
    ▼
PHASE_DONE ──(TERMINATE 전송, 서버 종료)
```
페이즈 전이는 **ControlLoop 쓰레드**가 mutex로 상태 테이블을 읽어 판정한다(배리어 동기화).

## 13. 데이터 구조 (서버 전역, 공유 자원)

```c
typedef enum { PHASE_INIT, PHASE_GATHERING, PHASE1_DONE,
               PHASE_SHIFTING, PHASE_DONE } SystemPhase;
typedef enum { D_CONNECTED, D_GATHERING, D_GATHERED,
               D_SHIFTING, D_DONE } DroneStatus;

typedef struct {
    SOCKET      sock;
    int         droneId;
    float       x, y;          // 최신 보고 위치
    float       tx, ty;        // 서버가 지정한 목표
    DroneStatus status;
    int         active;        // 사용 중 슬롯 여부
} DroneState;

DroneState   g_drones[MAX_DRONE];   // ← 채팅의 clntSocks[] 확장판
int          g_droneCnt = 0;
SystemPhase  g_phase    = PHASE_INIT;
HANDLE       g_mutex;               // ← 예제의 hMutex 그대로
```

---

# C. 알고리즘

## 14. 핵심 알고리즘

### 14.1 1차 집결 목표 좌표 배치
중심 C=(0, 100). 반경 30m 원 내에 N개 드론을 **상호 간격 ≥ 10m**로 배치.
- 반경 R=15m 원주 위에 균등 각도 분할: `tx = R·cos(2πk/N)`, `ty = 100 + R·sin(2πk/N)`
- N=3: 인접 간격 ≈ 31m ≥ 10m ✅, 반경 15m ≤ 30m ✅
- N 증가로 간격이 좁아지면 R↑(≤30m) 또는 2중 원 배치로 확장.

### 14.2 점진적 이동 (드론 측, 매 틱)
```
dx = tx - x;  dy = ty - y;  dist = sqrt(dx²+dy²);
if (dist <= STEP) { x = tx; y = ty; }
else { x += STEP*dx/dist;  y += STEP*dy/dist; }
clamp(x, -100, 100);  clamp(y, 20, 200);
```

### 14.3 1차 완료 판정 (서버 측)
```
모든 active 드론: (1) 중심(0,100)과 거리 ≤ 30m  AND
                 (2) 임의 두 드론 간 거리 ≥ 10m   →  PHASE1_DONE
```

### 14.4 2차 목표
각 드론: `tx = (1차도달 x) − 50`, `ty = y`. 클램핑으로 X ≥ −100 보정.

---

# D. 시각화 설계 (평가 항목: UI 우수성)

## 15. 시각화 목표 및 전략

평가 기준의 "UI 우수성: 각 드론의 움직임과 메시지 송수신 상황을 잘 보여줄 것"을 충족하기 위해
서버 콘솔을 **3개 패널**(레이더 / 상태표 / 메시지 로그)로 구성하고,
독립 **RenderLoop 쓰레드**가 10 FPS로 재그린다.

- **기술**: Windows 콘솔. `system("cls")` 대신 **`SetConsoleCursorPosition`** 으로 좌상단 복귀 →
  깜빡임 없는 부분 갱신. 색상은 `SetConsoleTextAttribute`(페이즈/상태별 색).
- **렌더 분리**: 네트워크/제어 쓰레드는 데이터만 갱신, 그리기는 RenderLoop 전담 → 출력 경합 방지.
- 렌더는 잠금 구간에서 **스냅샷만 복사**한 뒤 잠금 밖에서 그린다(§9 원칙).

## 16. 화면 레이아웃 (목업)

```
╔══════════════════════════════════════════════════════════════════╗
║  DRONE SWARM — BASE STATION            PHASE: ▶ GATHERING (2/5)    ║
╠════════════════════════════════════╦═════════════════════════════╣
║  RADAR  (X:-100..100  Y:20..200)    ║  DRONES                     ║
║   200 ┤                             ║  ID  X      Y     상태  Δ중심 ║
║       │            · 30m ring       ║  01 -12.4 103.2  GATH  18m  ║
║   150 ┤        ╱‾‾‾‾‾‾╲             ║  02   8.1  97.5  GATH  14m  ║
║       │       │  ②  ① │            ║  03   3.3 110.0  DONE   9m  ║
║   100 ┤  ◎    │    ③  │ ← center    ║                             ║
║       │       ╲______╱             ║  연결: 3 / 최소간격: 21m     ║
║    50 ┤                             ║  목표달성: 1 / 3            ║
║    20 ┤────────────────────────     ╠═════════════════════════════╣
║       └──────────────────────────   ║  MESSAGE LOG                ║
║      -100      0(base)     +100      ║  ▸ RX #128 POS  D02(8.1,97) ║
║                                     ║  ◂ TX     MOVE D01→(0,115)  ║
║   범례: ① 드론  ◎ 기지국  · 30m경계  ║  ◂ TX     MOVE D02→(13,107) ║
╠════════════════════════════════════╩═════════════════════════════╣
║  [Q] 종료   목표: 100m±30m 집결 → 좌측 50m 이동                     ║
╚══════════════════════════════════════════════════════════════════╝
```

## 17. 패널별 설계

### 17.1 레이더 패널 (좌)
- 월드 좌표(X:−100~100, Y:20~200)를 콘솔 셀 격자(예: 폭 40 × 높이 20)로 매핑.
  - `col = round((x + 100) / 200 * (W-1))`, `row = round((200 - y) / 180 * (H-1))` (Y 위가 높이↑)
- 기호: 드론 = 숫자/`●`, 기지국 = `◎`(원점), 집결 중심(0,100)과 **반경 30m 링**을 `·`로 표시.
- 색상: GATHERING=노랑, GATHERED=초록, SHIFTING=청록, 충돌위험(간격<10m)=빨강 점멸.
- 같은 셀에 2드론이 겹치면 `*`로 표기(겹침 경고).

### 17.2 상태표 패널 (우상)
- 드론별 행: ID / X / Y / 상태 / 중심거리 Δ. 헤더에 연결 수·최소간격·목표달성 수.
- 1차/2차 완료 판정에 쓰는 수치(최소간격, 중심거리)를 그대로 노출 → 채점자가 제약 충족을 눈으로 확인.

### 17.3 메시지 로그 패널 (우하)
- 최근 N(=8)개 송수신 이벤트를 순환 버퍼로 표시. `▸ RX`=수신, `◂ TX`=송신.
- 각 줄: 방향 · seq · 타입(POS/MOVE/TERM) · 대상 드론 · 핵심 좌표.
- **요구사항의 "메시지 송수신 상황을 보여줄 것"** 을 직접 충족(데모·보고서 캡처에 활용).

### 17.4 헤더/푸터
- 헤더: 현재 페이즈 + 진행 단계 표시(예: `GATHERING (2/5)`).
- 푸터: 종료 키 안내 + 시나리오 목표 요약(데모 시청자 이해 보조).

## 18. 렌더링 의사코드

```c
unsigned WINAPI RenderLoop(void *arg) {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    HideCursor(h);
    DroneState snap[MAX_DRONE]; int n; SystemPhase ph;
    while (g_running) {
        WaitForSingleObject(g_mutex, INFINITE);
        n = SnapshotDrones(snap); ph = g_phase;        // 복사만
        ReleaseMutex(g_mutex);

        SetConsoleCursorPosition(h, (COORD){0,0});     // 좌상단 복귀(무깜빡)
        DrawHeader(ph);
        DrawRadar(snap, n);                            // 좌표→격자 매핑
        DrawTable(snap, n);                            // 상태표 + 최소간격
        DrawMsgLog();                                  // 순환 버퍼
        Sleep(100);                                    // 10 FPS
    }
    return 0;
}
```

### 드론 클라이언트 콘솔
- 1줄 상태 + 직전 송수신: `D02 pos(8.1,97.5)→tgt(13,107.5) | TX POS#128  RX MOVE`.

---

## 19. 에러 처리 / 안정성 (평가 항목: 실행 안정성)

| 상황 | 처리 |
|------|------|
| `recv()==0 또는 SOCKET_ERROR` | 해당 드론 슬롯 비활성화, 소켓 닫기 (예제의 disconnect 정리 변형) |
| 부분 수신 | §8 `RecvFixed`로 `sizeof(Message)` 채울 때까지 누적 |
| 부분 송신 | §8 `SendFixed`로 전량 전송 보장 |
| 드론 < 3 접속 | ControlLoop는 3대 이상 접속 전까지 PHASE_INIT 유지 |
| 공유 자원 접근 | 전 구간 mutex 보호, 임계구역 내 블로킹 송신 금지(§9) |
| 콘솔 출력 경합 | 그리기는 RenderLoop 단일 쓰레드로 일원화 |
| 종료 | TERMINATE 송신 → 소켓/뮤텍스/WSACleanup 정리 |

---

## 20. 모듈/파일 구성

| 파일 | 책임 |
|------|------|
| `protocol.h` | `Message` 구조체, MSG_* 상수, 영역/주기 상수 (서버·클라 공유) |
| `server.c` | Acceptor + DroneHandler + ControlLoop + **RenderLoop** + 군집 알고리즘 |
| `drone.c` | connect + ReportThread + CommandThread + 운동 시뮬레이션 |
| `console.c/.h` | (선택) `SetConsoleCursorPosition`/색상 래퍼, 레이더·표·로그 그리기 |

빌드(Visual Studio): 링커에 `ws2_32.lib`, 멀티쓰레드 CRT(`_beginthreadex`), 콘솔 응용.

---

## 21. 요구사항 ↔ 설계 추적 매트릭스

| 요구사항 | 충족 설계 |
|----------|-----------|
| 주기적 위치 보고 | §3.2 ReportThread + §10 POSITION_REPORT |
| 다중 드론 동시 처리 | §6 연결당 DroneHandler 쓰레드 (예제 골격) |
| 드론 ≥ 3 | §12 ControlLoop 게이트 |
| random 시작·자율 이동 | §3.2 + §4 (1차 명령 전까지 임의 이동) |
| 1차 집결(반경30·간격10) | §14.1 배치 + §14.3 판정 |
| 2차 좌측 50m | §14.4 |
| 완료 후 종료 | §12 PHASE_DONE + §10 TERMINATE |
| TCP·멀티쓰레드·C·예제기반 | §5~§9 (Chapter20 골격 재사용 + framing/동기화 보강) |
| UI 우수성(움직임·메시지 가시화) | §15~§18 레이더·상태표·메시지 로그 3패널 |
| 실행 안정성 | §8 framing, §9 동기화, §19 에러 처리 |

---

## 22. 잔여 리스크
1. **멀티쓰레드 race** — 가장 큰 데모 크래시 요인. mutex 누락·임계구역 내 송신 점검 필수.
2. **콘솔 렌더 성능/깜빡임** — `cls` 대신 커서 복귀 방식 사용, 10 FPS로 제한.
3. **간격 10m 유지** — 이동 중 일시 위반 가능. 판정은 "도달 후" 기준으로만 적용 권장.
4. **N 가변성** — N>7 시 §14.1 단일 원 배치 한계 → 2중 원 확장 필요.
5. **엔디안** — 이기종 환경 데모 시 구조체 raw 송수신 깨짐 → 직렬화 필요(현재 동일 PC 가정).
```