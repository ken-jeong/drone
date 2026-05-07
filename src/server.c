/*
    server.c - 기지국 서버 (드론 군집 제어)
    - Acceptor(main) : accept 루프, 연결당 DroneHandler 쓰레드 생성
    - DroneHandler   : 드론별 POSITION_REPORT 수신
    - ControlLoop    : 페이즈 관리 + MOVE_CMD 유니캐스트
    - RenderLoop     : 콘솔 레이더/상태표/메시지로그 렌더
*/

#define _CRT_SECURE_NO_WARNINGS

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "protocol.h"

#pragma comment(lib, "ws2_32.lib")

#define MAX_DRONE 64

/* ---- 서버 페이즈/드론 상태 ---- */
typedef enum {
    PHASE_INIT, PHASE_GATHERING, PHASE1_DONE, PHASE_SHIFTING, PHASE_DONE
} SystemPhase;

typedef enum {
    D_CONNECTED, D_GATHERING, D_GATHERED, D_SHIFTING, D_DONE
} DroneStatus;

typedef struct {
    SOCKET      sock;
    int         droneId;
    float       x, y;       /* 최신 보고 위치 */
    float       tx, ty;     /* 서버 지정 목표 */
    DroneStatus status;
    int         active;     /* 슬롯 사용 여부 */
    int         reported;   /* 1회 이상 위치 보고를 받았는가 */
} DroneState;

/* ---- 전역 공유 상태 (g_mutex 로 보호) ---- */
static DroneState  g_drones[MAX_DRONE];
static int         g_droneCnt = 0;
static SystemPhase g_phase    = PHASE_INIT;
static HANDLE      g_mutex;
static volatile LONG g_running = 1;
static SOCKET      g_servSock = INVALID_SOCKET;

/* ---- 메시지 로그 (별도 mutex) ---- */
#define LOG_CAP 8
static char   g_log[LOG_CAP][96];
static int    g_logHead = 0, g_logCount = 0;
static HANDLE g_logMutex;

/* ===================== 유틸 ===================== */
static void ErrorHandling(const char *msg) {
    fputs(msg, stderr); fputc('\n', stderr);
    exit(1);
}

static float fclampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
static float dist2f(float ax, float ay, float bx, float by) {
    float dx = ax - bx, dy = ay - by;
    return (float)sqrt(dx * dx + dy * dy);
}

/* 정확히 len 바이트 송신 (부분 송신 대응) */
static int SendFixed(SOCKET s, const char *buf, int len) {
    int sent = 0, w;
    while (sent < len) {
        w = send(s, buf + sent, len - sent, 0);
        if (w == SOCKET_ERROR) return SOCKET_ERROR;
        sent += w;
    }
    return sent;
}
/* 정확히 len 바이트 수신 (부분 수신 대응) */
static int RecvFixed(SOCKET s, char *buf, int len) {
    int got = 0, r;
    while (got < len) {
        r = recv(s, buf + got, len - got, 0);
        if (r == 0 || r == SOCKET_ERROR) return r;
        got += r;
    }
    return got;
}

static void AddLog(const char *line) {
    WaitForSingleObject(g_logMutex, INFINITE);
    int idx = (g_logHead + g_logCount) % LOG_CAP;
    if (g_logCount < LOG_CAP) g_logCount++;
    else g_logHead = (g_logHead + 1) % LOG_CAP;
    strncpy(g_log[idx], line, sizeof(g_log[0]) - 1);
    g_log[idx][sizeof(g_log[0]) - 1] = 0;
    ReleaseMutex(g_logMutex);
}

/* ===================== 드론 등록 ===================== */
/* g_mutex 잠금 상태에서 호출. 빈 슬롯에 등록하고 슬롯 인덱스를 반환(-1=실패). */
static int RegisterDrone(SOCKET sock) {
    int i;
    for (i = 0; i < MAX_DRONE; i++) {
        if (!g_drones[i].active) {
            memset(&g_drones[i], 0, sizeof(DroneState));
            g_drones[i].sock     = sock;
            g_drones[i].droneId  = i + 1;
            g_drones[i].status   = D_CONNECTED;
            g_drones[i].active   = 1;
            g_drones[i].reported = 0;
            g_droneCnt++;
            return i;
        }
    }
    return -1;
}

// ==================================================
// 2. DroneHandler 쓰레드
// -> 해당 드론의 위치 보고를 계속 받아서, 공유 상태 테이블의 위치를 갱신
// ==================================================
static unsigned WINAPI DroneHandler(void *arg) { // arg = 슬롯 인덱스(값 전달)
    int     slot = (int)(intptr_t)arg;
    SOCKET  sock = g_drones[slot].sock;
    Message m;
    char    buf[96];

    while (g_running) {
        int r = RecvFixed(sock, (char *)&m, sizeof(m));

        if (r == 0 || r == SOCKET_ERROR) { // 연결 종료/오류
            break;
        }

        if (m.type == MSG_POSITION_REPORT) {
            // 공유 자원에 접근할 때는 항상 뮤텍스로 보호
            WaitForSingleObject(g_mutex, INFINITE);
            g_drones[slot].x = m.x;
            g_drones[slot].y = m.y;
            g_drones[slot].reported = 1;
            ReleaseMutex(g_mutex);

            sprintf(
                buf, "RX #%-4d POS  D%02d (%.1f,%.1f)",
                m.seq, g_drones[slot].droneId, m.x, m.y
            );
            AddLog(buf);
        }
    }

    // 연결 정리
    WaitForSingleObject(g_mutex, INFINITE);
    g_drones[slot].active = 0;
    g_droneCnt--;
    ReleaseMutex(g_mutex);
    closesocket(sock);

    sprintf(buf, "-- D%02d disconnected", slot + 1);
    AddLog(buf);
    return 0;
}

/* g_mutex 잠금 상태에서 호출: active 슬롯 인덱스 수집 */
static int CollectActive(int *idx) {
    int i, n = 0;
    for (i = 0; i < MAX_DRONE; i++)
        if (g_drones[i].active && g_drones[i].reported) idx[n++] = i;
    return n;
}

/* 보낼 명령을 잠금 밖에서 전송하기 위한 임시 구조 */
typedef struct { SOCKET sock; Message msg; } OutCmd;

static int  g_seqTx = 0;

/* 1차 집결 목표 배치 (반경 RING_RADIUS 원주 균등분할) */
static void AssignGatherTargets(int *idx, int n) {
    int k;
    for (k = 0; k < n; k++) {
        float ang = (float)(2.0 * 3.14159265358979 * k / (n > 0 ? n : 1));
        g_drones[idx[k]].tx = CENTER_X + RING_RADIUS * (float)cos(ang);
        g_drones[idx[k]].ty = CENTER_Y + RING_RADIUS * (float)sin(ang);
        g_drones[idx[k]].status = D_GATHERING;
    }
}

/* 1차 완료 판정 */
static int GatheringComplete(int *idx, int n) {
    int i, j;
    if (n < MIN_DRONES) return 0;
    for (i = 0; i < n; i++) {
        DroneState *d = &g_drones[idx[i]];
        if (dist2f(d->x, d->y, CENTER_X, CENTER_Y) > GATHER_RADIUS) return 0;
    }
    for (i = 0; i < n; i++)
        for (j = i + 1; j < n; j++) {
            DroneState *a = &g_drones[idx[i]], *b = &g_drones[idx[j]];
            if (dist2f(a->x, a->y, b->x, b->y) < MIN_SEPARATION) return 0;
        }
    return 1;
}

/* 2차(좌측 50m) 도달 판정 */
static int ShiftComplete(int *idx, int n) {
    int i;
    if (n < MIN_DRONES) return 0;
    for (i = 0; i < n; i++) {
        DroneState *d = &g_drones[idx[i]];
        if (dist2f(d->x, d->y, d->tx, d->ty) > ARRIVE_EPS) return 0;
    }
    return 1;
}

// ==================================================
// 3. ControlLoop 쓰레드
// [핵심] 시스템을 페이즈 상태기계로 운영
// ==================================================
static unsigned WINAPI ControlLoop(void *arg) {
    int idx[MAX_DRONE], n, i;
    OutCmd out[MAX_DRONE]; int outN;
    char buf[96];
    (void)arg;

    while (g_running) {
        outN = 0;
        WaitForSingleObject(g_mutex, INFINITE);
        n = CollectActive(idx);

        switch (g_phase) {
        // 3.1. 임의 배치/배회
        case PHASE_INIT:
            if (n >= MIN_DRONES) {
                AssignGatherTargets(idx, n);
                for (i = 0; i < n; i++) {
                    DroneState *d = &g_drones[idx[i]];
                    out[outN].sock = d->sock;
                    out[outN].msg.type    = MSG_MOVE_CMD;
                    out[outN].msg.droneId = d->droneId;
                    out[outN].msg.x = d->tx; out[outN].msg.y = d->ty;
                    out[outN].msg.seq = ++g_seqTx;
                    outN++;
                }
                g_phase = PHASE_GATHERING;
            }
            break;
        
        // 3.2. 1차 집결 진행
        case PHASE_GATHERING:
            if (GatheringComplete(idx, n)) {
                for (i = 0; i < n; i++) {
                    g_drones[idx[i]].status = D_GATHERED;
                }
                g_phase = PHASE1_DONE;
            }
            break;
        
        // 3.3. 1차 완료
        case PHASE1_DONE: // 2차 목표(좌측 50m) 산출·송신
            for (i = 0; i < n; i++) {
                DroneState *d = &g_drones[idx[i]];
                d->tx = fclampf(d->x - SHIFT_LEFT, WORLD_X_MIN, WORLD_X_MAX);
                d->ty = d->y;
                d->status = D_SHIFTING;
                out[outN].sock = d->sock;
                out[outN].msg.type    = MSG_MOVE_CMD;
                out[outN].msg.droneId = d->droneId;
                out[outN].msg.x = d->tx; out[outN].msg.y = d->ty;
                out[outN].msg.seq = ++g_seqTx;
                outN++;
            }
            g_phase = PHASE_SHIFTING;
            break;
        
        // 3.4. 2차 좌측 이동
        case PHASE_SHIFTING:
            if (ShiftComplete(idx, n)) {
                for (i = 0; i < n; i++) {
                    DroneState *d = &g_drones[idx[i]];
                    d->status = D_DONE;
                    out[outN].sock = d->sock;
                    out[outN].msg.type    = MSG_TERMINATE;
                    out[outN].msg.droneId = d->droneId;
                    out[outN].msg.x = d->x; out[outN].msg.y = d->y;
                    out[outN].msg.seq = ++g_seqTx;
                    outN++;
                }
                g_phase = PHASE_DONE;
            }
            break;
        
        // 3.5. 종료
        case PHASE_DONE:
        default:
            break;
        }
        SystemPhase phaseNow = g_phase;
        ReleaseMutex(g_mutex);

        // 잠금 밖에서 송신 (교착 방지)
        for (i = 0; i < outN; i++) {
            SendFixed(out[i].sock, (const char *)&out[i].msg, sizeof(Message));
            const char *t = (out[i].msg.type == MSG_TERMINATE) ? "TERM" : "MOVE";
            sprintf(buf, "TX #%-4d %s D%02d ->(%.1f,%.1f)",
                    out[i].msg.seq, t, out[i].msg.droneId,
                    out[i].msg.x, out[i].msg.y);
            AddLog(buf);
        }

        if (phaseNow == PHASE_DONE) {
            Sleep(800); // 마지막 프레임을 잠깐 보여줌
            InterlockedExchange(&g_running, 0);
            if (g_servSock != INVALID_SOCKET) closesocket(g_servSock); // accept 해제
            break;
        }
        Sleep(CONTROL_PERIOD_MS);
    }
    return 0;
}

/* ===================== RenderLoop 쓰레드 ===================== */
#define RAD_W 41
#define RAD_H 19

static const char *PhaseName(SystemPhase p) {
    switch (p) {
    case PHASE_INIT:      return "INIT (대기)";
    case PHASE_GATHERING: return "GATHERING (1차 집결)";
    case PHASE1_DONE:     return "GATHERED (1차 완료)";
    case PHASE_SHIFTING:  return "SHIFTING (2차 좌측이동)";
    case PHASE_DONE:      return "DONE (종료)";
    default:              return "?";
    }
}
static const char *StatusName(DroneStatus s) {
    switch (s) {
    case D_CONNECTED: return "CONN";
    case D_GATHERING: return "GATH";
    case D_GATHERED:  return "GTHD";
    case D_SHIFTING:  return "SHFT";
    case D_DONE:      return "DONE";
    default:          return "?";
    }
}

static void SetColor(HANDLE h, WORD attr) { SetConsoleTextAttribute(h, attr); }

static void DrawRadar(DroneState *snap, int n) {
    char grid[RAD_H][RAD_W + 1];
    int r, c, i;
    for (r = 0; r < RAD_H; r++) {
        for (c = 0; c < RAD_W; c++) grid[r][c] = ' ';
        grid[r][RAD_W] = 0;
    }
    /* 30m 집결 링 (점선) */
    for (i = 0; i < 72; i++) {
        float ang = (float)(2.0 * 3.14159265 * i / 72.0);
        float wx = CENTER_X + GATHER_RADIUS * (float)cos(ang);
        float wy = CENTER_Y + GATHER_RADIUS * (float)sin(ang);
        c = (int)((wx - WORLD_X_MIN) / (WORLD_X_MAX - WORLD_X_MIN) * (RAD_W - 1) + 0.5f);
        r = (int)((WORLD_Y_MAX - wy) / (WORLD_Y_MAX - WORLD_Y_MIN) * (RAD_H - 1) + 0.5f);
        if (r >= 0 && r < RAD_H && c >= 0 && c < RAD_W && grid[r][c] == ' ')
            grid[r][c] = '.';
    }
    /* 집결 중심 '+' */
    c = (int)((CENTER_X - WORLD_X_MIN) / (WORLD_X_MAX - WORLD_X_MIN) * (RAD_W - 1) + 0.5f);
    r = (int)((WORLD_Y_MAX - CENTER_Y) / (WORLD_Y_MAX - WORLD_Y_MIN) * (RAD_H - 1) + 0.5f);
    if (r >= 0 && r < RAD_H && c >= 0 && c < RAD_W) grid[r][c] = '+';
    /* 기지국 'B' (지상 중앙) */
    grid[RAD_H - 1][(RAD_W - 1) / 2] = 'B';
    /* 드론: id 숫자 (겹치면 '*') */
    for (i = 0; i < n; i++) {
        c = (int)((snap[i].x - WORLD_X_MIN) / (WORLD_X_MAX - WORLD_X_MIN) * (RAD_W - 1) + 0.5f);
        r = (int)((WORLD_Y_MAX - snap[i].y) / (WORLD_Y_MAX - WORLD_Y_MIN) * (RAD_H - 1) + 0.5f);
        if (r < 0) r = 0; if (r >= RAD_H) r = RAD_H - 1;
        if (c < 0) c = 0; if (c >= RAD_W) c = RAD_W - 1;
        if (grid[r][c] >= '0' && grid[r][c] <= '9') grid[r][c] = '*';
        else grid[r][c] = (char)('0' + (snap[i].droneId % 10));
    }
    /* 출력: y축 눈금과 함께 */
    for (r = 0; r < RAD_H; r++) {
        int ylab = (int)(WORLD_Y_MAX - (float)r / (RAD_H - 1) * (WORLD_Y_MAX - WORLD_Y_MIN) + 0.5f);
        printf("  %3d |%s|\n", ylab, grid[r]);
    }
    printf("       +");
    for (c = 0; c < RAD_W; c++) putchar('-');
    printf("+\n");
    printf("       %-4d         0(base)        %4d\n",
           (int)WORLD_X_MIN, (int)WORLD_X_MAX);
}

static void DrawFrame(HANDLE h) {
    DroneState snap[MAX_DRONE];
    int i, j, n = 0;
    SystemPhase ph;

    /* 스냅샷만 잠금 안에서 복사 */
    WaitForSingleObject(g_mutex, INFINITE);
    ph = g_phase;
    for (i = 0; i < MAX_DRONE; i++)
        if (g_drones[i].active) snap[n++] = g_drones[i];
    ReleaseMutex(g_mutex);

    /* 최소간격/목표달성 계산 */
    float minSep = 1e9f; int arrived = 0;
    for (i = 0; i < n; i++) {
        if (dist2f(snap[i].x, snap[i].y, CENTER_X, CENTER_Y) <= GATHER_RADIUS &&
            (ph == PHASE_GATHERING || ph == PHASE1_DONE)) arrived++;
        for (j = i + 1; j < n; j++) {
            float d = dist2f(snap[i].x, snap[i].y, snap[j].x, snap[j].y);
            if (d < minSep) minSep = d;
        }
    }
    if (n < 2) minSep = 0.0f;

    COORD home = { 0, 0 };
    SetConsoleCursorPosition(h, home);

    SetColor(h, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
    printf("==================== DRONE SWARM - BASE STATION ====================\n");
    SetColor(h, FOREGROUND_GREEN | FOREGROUND_INTENSITY);
    printf("  PHASE: %-28s  연결 드론: %d        \n", PhaseName(ph), n);
    SetColor(h, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
    printf("--------------------------------------------------------------------\n");

    /* 레이더 */
    DrawRadar(snap, n);

    /* 상태표 */
    printf("\n  DRONES                                                      \n");
    printf("  ID    X        Y      목표X    목표Y    상태   중심거리      \n");
    for (i = 0; i < n; i++) {
        DroneState *d = &snap[i];
        WORD col;
        switch (d->status) {
        case D_GATHERING: col = FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY; break; /* yellow */
        case D_GATHERED:  col = FOREGROUND_GREEN | FOREGROUND_INTENSITY; break;                  /* green  */
        case D_SHIFTING:  col = FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY; break;/* cyan   */
        case D_DONE:      col = FOREGROUND_GREEN; break;
        default:          col = FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE;
        }
        SetColor(h, col);
        printf("  %02d  %7.1f  %7.1f  %7.1f  %7.1f   %-4s   %6.1fm    \n",
               d->droneId, d->x, d->y, d->tx, d->ty, StatusName(d->status),
               dist2f(d->x, d->y, CENTER_X, CENTER_Y));
    }
    SetColor(h, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
    for (i = n; i < MIN_DRONES; i++) printf("  --  (대기 중: 최소 %d대 필요)                              \n", MIN_DRONES);
    printf("  최소간격: %.1fm   (목표: >=%.0fm)   집결중심 반경내: %d/%d        \n",
           minSep, MIN_SEPARATION, arrived, n);

    /* 메시지 로그 */
    printf("\n  MESSAGE LOG (최근 %d건)                                     \n", LOG_CAP);
    WaitForSingleObject(g_logMutex, INFINITE);
    for (i = 0; i < LOG_CAP; i++) {
        if (i < g_logCount) {
            int idx = (g_logHead + i) % LOG_CAP;
            char dir = (g_log[idx][0] == 'T') ? '<' : '>';
            SetColor(h, (g_log[idx][0] == 'T')
                        ? (FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY)
                        : (FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY));
            printf("   %c %-60s\n", dir, g_log[idx]);
        } else {
            printf("                                                              \n");
        }
    }
    ReleaseMutex(g_logMutex);

    SetColor(h, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
    printf("--------------------------------------------------------------------\n");
    printf("  목표: 100m +-30m 집결(간격>=10m) -> 전원 좌측 50m 이동 -> 종료      \n");
}

static unsigned WINAPI RenderLoop(void *arg) {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_CURSOR_INFO ci; (void)arg;
    GetConsoleCursorInfo(h, &ci); ci.bVisible = FALSE;
    SetConsoleCursorInfo(h, &ci);
    system("cls");
    while (g_running) {
        DrawFrame(h);
        Sleep(RENDER_PERIOD_MS);
    }
    DrawFrame(h);   /* 마지막 상태 1프레임 */
    ci.bVisible = TRUE; SetConsoleCursorInfo(h, &ci);
    return 0;
}

// ==================================================
// 1. main (Acceptor)
// ==================================================
int main(int argc, char *argv[]) {
    WSADATA     wsa;
    SOCKADDR_IN servAdr, clntAdr;
    int         clntAdrSz;
    HANDLE      hCtrl, hRender;

    // 콘솔을 UTF-8로 설정 (한글 깨짐 방지)
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    if (argc != 2) {
        printf("Usage : %s <port>\n", argv[0]);
        exit(1);
    }

    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        ErrorHandling("WSAStartup() error");
    }

    g_mutex    = CreateMutex(NULL, FALSE, NULL);
    g_logMutex = CreateMutex(NULL, FALSE, NULL);

    // 1.1. socket()
    g_servSock = socket(PF_INET, SOCK_STREAM, 0);

    if (g_servSock == INVALID_SOCKET) {
        ErrorHandling("socket() error");
    }

    memset(&servAdr, 0, sizeof(servAdr));
    servAdr.sin_family      = AF_INET;
    servAdr.sin_addr.s_addr = htonl(INADDR_ANY);
    servAdr.sin_port        = htons((u_short)atoi(argv[1]));

    // 1.2. bind()
    if (bind(g_servSock, (SOCKADDR *)&servAdr, sizeof(servAdr)) == SOCKET_ERROR) {
        ErrorHandling("bind() error");
    }

    // 1.3. listen()
    if (listen(g_servSock, 5) == SOCKET_ERROR) {
        ErrorHandling("listen() error");
    }

    AddLog("-- 서버 시작, 드론 접속 대기");

    hCtrl   = (HANDLE)_beginthreadex(NULL, 0, ControlLoop, NULL, 0, NULL);
    hRender = (HANDLE)_beginthreadex(NULL, 0, RenderLoop,  NULL, 0, NULL);

    // 1.4. accept() 루프
    while (g_running) {
        clntAdrSz    = sizeof(clntAdr);
        SOCKET hClnt = accept(g_servSock, (SOCKADDR *)&clntAdr, &clntAdrSz);

        if (hClnt == INVALID_SOCKET) { // g_servSock 닫힘 → 종료
            break; 
        }

        WaitForSingleObject(g_mutex, INFINITE);
        int slot = RegisterDrone(hClnt); // 드론이 접속할 때마다 빈 슬롯에 등록
        ReleaseMutex(g_mutex);

        if (slot < 0) {
            closesocket(hClnt);
            continue;
        }

        char buf[96];
        sprintf(buf, "++ D%02d connected (%s)", slot + 1, inet_ntoa(clntAdr.sin_addr));
        AddLog(buf);

        _beginthreadex(NULL, 0, DroneHandler, (void *)(intptr_t)slot, 0, NULL);
    }

    WaitForSingleObject(hCtrl, 2000);
    WaitForSingleObject(hRender, 2000);

    closesocket(g_servSock);
    WSACleanup();
    return 0;
}
