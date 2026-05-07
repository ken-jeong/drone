/*
    drone.c - 드론 클라이언트
    - ReportThread  : 주기마다 1틱 이동 + POSITION_REPORT 송신
    - CommandThread : MOVE_CMD/TERMINATE 수신 → 목표 갱신/종료
    - 1차 명령 전까지는 영역 내 임의(random) 위치를 배회

    실행: drone <서버IP> <포트> [이름]
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
#include <time.h>

#include "protocol.h"

#pragma comment(lib, "ws2_32.lib")

/* ---- 드론 로컬 상태 (g_dmutex 로 보호) ---- */
static float  g_x, g_y;          /* 현재 위치 */
static float  g_tx, g_ty;        /* 목표 위치 */
static int    g_commanded = 0;   /* 서버 1차 명령 수신 여부 */
static int    g_myId = 0;        /* 서버가 알려준 id(표시용) */
static volatile int g_alive = 1;
static HANDLE g_dmutex;
static char   g_name[32] = "drone";

static void ErrorHandling(const char *msg) {
    fputs(msg, stderr); fputc('\n', stderr);
    exit(1);
}
static float fclampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
static float frand(float lo, float hi) {
    return lo + (hi - lo) * ((float)rand() / (float)RAND_MAX);
}

/* 정확히 len 바이트 송/수신 */
static int SendFixed(SOCKET s, const char *buf, int len) {
    int sent = 0, w;
    while (sent < len) {
        w = send(s, buf + sent, len - sent, 0);
        if (w == SOCKET_ERROR) return SOCKET_ERROR;
        sent += w;
    }
    return sent;
}
static int RecvFixed(SOCKET s, char *buf, int len) {
    int got = 0, r;
    while (got < len) {
        r = recv(s, buf + got, len - got, 0);
        if (r == 0 || r == SOCKET_ERROR) return r;
        got += r;
    }
    return got;
}

// ==================================================
// 1. 송신 쓰레드 ReportThread
//    1틱 점진 이동 후 현재 위치를 보고
// ==================================================
static unsigned WINAPI ReportThread(void *arg) {
    SOCKET  sock = *((SOCKET *)arg);
    Message m;
    int     seq = 0;

    while (g_alive) {
        WaitForSingleObject(g_dmutex, INFINITE);

        // 명령 전: 목표 도달 시 새 임의 목표(배회)
        if (!g_commanded) {
            float d = (float)sqrt(
                (g_tx - g_x) * (g_tx - g_x) + (g_ty - g_y) * (g_ty - g_y)
            );
            if (d <= MOVE_STEP) {
                g_tx = frand(WORLD_X_MIN, WORLD_X_MAX);
                g_ty = frand(WORLD_Y_MIN, WORLD_Y_MAX);
            }
        }

        // 점진 이동
        float dx   = g_tx - g_x, dy = g_ty - g_y;
        float dist = (float)sqrt(dx * dx + dy * dy);
        if (dist <= MOVE_STEP) {
            g_x = g_tx; g_y = g_ty;
        } else {
            g_x += MOVE_STEP * dx / dist;
            g_y += MOVE_STEP * dy / dist;
        }
        g_x = fclampf(g_x, WORLD_X_MIN, WORLD_X_MAX);
        g_y = fclampf(g_y, WORLD_Y_MIN, WORLD_Y_MAX);

        float cx  = g_x, cy = g_y, ctx = g_tx, cty = g_ty;
        int   cmd = g_commanded;
        ReleaseMutex(g_dmutex);

        m.type    = MSG_POSITION_REPORT;
        m.droneId = g_myId;
        m.x       = cx;
        m.y       = cy;
        m.seq     = ++seq;
        if (SendFixed(sock, (const char *)&m, sizeof(m)) == SOCKET_ERROR) {
            break;
        }

        printf(
            "[%s] pos(%6.1f,%6.1f) -> tgt(%6.1f,%6.1f) | TX POS #%-4d %s\n",
            g_name, cx, cy, ctx, cty, seq, cmd ? "(명령수행중)" : "(배회중)"
        );

        Sleep(REPORT_PERIOD_MS);
    }
    return 0;
}

// ==================================================
// 2. 수신 쓰레드 CommandThread
//    1틱 점진 이동 후 현재 위치를 보고
// ==================================================
static unsigned WINAPI CommandThread(void *arg) {
    SOCKET  sock = *((SOCKET *)arg);
    Message m;

    while (g_alive) {
        int r = RecvFixed(sock, (char *)&m, sizeof(m));
        if (r == 0 || r == SOCKET_ERROR) {
            g_alive = 0;
            break;
        }

        if (m.type == MSG_MOVE_CMD) { // MOVE_CMD를 받으면 목표 좌표를 갱신
            WaitForSingleObject(g_dmutex, INFINITE);
            g_tx        = m.x;
            g_ty        = m.y;
            g_commanded = 1;
            g_myId      = m.droneId;
            ReleaseMutex(g_dmutex);
            printf(
                "[%s] RX MOVE  -> 목표 (%6.1f,%6.1f)  (id=%d)\n",
                g_name, m.x, m.y, m.droneId
            );
        } else if (m.type == MSG_TERMINATE) { // TERMINATE를 받으면 프로그램을 종료
            printf("[%s] RX TERMINATE -> 종료\n", g_name);
            g_alive = 0;
            break;
        }
    }
    return 0;
}

// ==================================================
// main
// ==================================================
int main(int argc, char *argv[]) {
    WSADATA     wsa;
    SOCKET      sock;
    SOCKADDR_IN servAdr;
    HANDLE      hSnd, hRcv;

    // 콘솔을 UTF-8로 설정 (한글 깨짐 방지)
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    if (argc < 3) {
        printf("Usage : %s <serverIP> <port> [name]\n", argv[0]);
        exit(1);
    }

    if (argc >= 4) {
        strncpy(g_name, argv[3], sizeof(g_name) - 1);
    }

    srand((unsigned)time(NULL) ^ (unsigned)GetCurrentProcessId());

    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        ErrorHandling("WSAStartup() error");
    }

    g_dmutex = CreateMutex(NULL, FALSE, NULL);

    /* 영역 내 임의 시작 위치 (요구사항) */
    g_x  = frand(WORLD_X_MIN, WORLD_X_MAX);
    g_y  = frand(WORLD_Y_MIN, WORLD_Y_MAX);
    g_tx = frand(WORLD_X_MIN, WORLD_X_MAX);
    g_ty = frand(WORLD_Y_MIN, WORLD_Y_MAX);

    sock = socket(PF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCKET) {
        ErrorHandling("socket() error");
    }

    memset(&servAdr, 0, sizeof(servAdr));
    servAdr.sin_family      = AF_INET;
    servAdr.sin_addr.s_addr = inet_addr(argv[1]);
    servAdr.sin_port        = htons((u_short)atoi(argv[2]));

    if (connect(sock, (SOCKADDR *)&servAdr, sizeof(servAdr)) == SOCKET_ERROR) {
        ErrorHandling("connect() error");
    }

    printf("[%s] 서버 접속 완료. 시작 위치 (%.1f, %.1f)\n", g_name, g_x, g_y);

    hSnd = (HANDLE)_beginthreadex(NULL, 0, ReportThread,  (void *)&sock, 0, NULL);
    hRcv = (HANDLE)_beginthreadex(NULL, 0, CommandThread, (void *)&sock, 0, NULL);

    WaitForSingleObject(hSnd, INFINITE);
    WaitForSingleObject(hRcv, INFINITE);

    closesocket(sock);
    WSACleanup();
    printf("[%s] 프로그램 종료\n", g_name);
    return 0;
}
