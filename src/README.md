# 드론 군집 제어 시스템 — 구현체

[`../DESIGN_SPEC.md`](../DESIGN_SPEC.md) 설계를 그대로 구현한 Winsock(C) 코드입니다.
기준 예제: 『윤성우의 열혈 TCP/IP 소켓 프로그래밍』 Chapter20 멀티쓰레드 채팅 예제.

## 파일 구성
| 파일 | 책임 | 설계 매핑 |
|------|------|-----------|
| `protocol.h` | 메시지 구조체·상수 (서버/클라 공유) | §10, §4 |
| `server.c` | Acceptor + DroneHandler + ControlLoop + RenderLoop | §6, §9, §12, §14~§18 |
| `drone.c` | connect + ReportThread + CommandThread + 운동 시뮬레이션 | §3.2, §14.2 |

## 빌드 (Windows)

### 방법 1) MinGW gcc — 이 프로젝트에서 사용한 방법
```bat
gcc server.c -o server.exe -lws2_32
gcc drone.c  -o drone.exe  -lws2_32
```
> WinLibs gcc 설치: `winget install BrechtSanders.WinLibs.POSIX.UCRT`
> (설치 직후 같은 셸에는 PATH 미반영 → 새 터미널 사용)

### 방법 2) Developer Command Prompt (MSVC `cl`)
```bat
cl /nologo server.c /Fe:server.exe /link ws2_32.lib
cl /nologo drone.c  /Fe:drone.exe  /link ws2_32.lib
```

### 방법 3) Visual Studio
- 콘솔 응용(Console App) 프로젝트 2개 생성 → 각각 `server.c`, `drone.c` + `protocol.h` 추가.
- 프로젝트 속성 → 링커 → 입력 → 추가 종속성에 `ws2_32.lib` 추가.
  (소스의 `#pragma comment(lib, "ws2_32.lib")` 로 자동 링크되므로 보통 생략 가능.)

## 실행
```bat
REM 1) 서버 (포트 9000)
server.exe 9000

REM 2) 드론 3대 이상 (다른 콘솔에서)
drone.exe 127.0.0.1 9000 D1
drone.exe 127.0.0.1 9000 D2
drone.exe 127.0.0.1 9000 D3
```
> 서버·드론 모두 시작 시 콘솔을 UTF-8로 설정(`SetConsoleOutputCP(CP_UTF8)`)하므로
> 한글 레이더/상태표/로그가 깨지지 않습니다.

## 동작 흐름 (자동 진행)
1. **INIT** — 드론들이 영역 내 임의 위치에서 배회. 3대 이상 접속·보고되면 →
2. **GATHERING** — 서버가 각 드론에 집결 목표(중심 (0,100), 반경 15m 원주) `MOVE_CMD` 송신.
   전원이 반경 30m 내 + 상호 간격 ≥ 10m 가 되면 →
3. **GATHERED → SHIFTING** — 서버가 전원에게 "현재 X − 50m" 2차 목표 송신. 전원 도달하면 →
4. **DONE** — 서버가 `TERMINATE` 송신, 모든 프로그램 종료.

서버 콘솔에는 실시간 **레이더 / 상태표 / 메시지 로그** 3패널이 표시됩니다(§15~§18).

## 결과보고서용 캡처 포인트
- INIT(배회) / GATHERING / GATHERED / SHIFTING / DONE 각 페이즈의 서버 레이더 화면
- 상태표의 "최소간격 ≥ 10m", "중심거리 ≤ 30m" 수치 (제약 충족 증빙)
- MESSAGE LOG 패널의 POS(RX)/MOVE·TERM(TX) 송수신 내역
- 드론 콘솔의 위치 보고 / 명령 수신 로그

## 주요 가정 (DESIGN_SPEC §4, §8)
- 좌표계: 기지국 원점, X∈[−100,100], Y(고도)∈[20,200], 단위 m.
- 서버·드론이 동일 PC(동일 엔디안)에서 구동 → 구조체 raw 송수신.
  이기종 환경 확장 시 직렬화/`htonl` 필요.
- 이동 속도 2 m/tick, 보고·제어 주기 200 ms, 도달 오차 1 m.
