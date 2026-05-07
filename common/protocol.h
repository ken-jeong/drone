#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define MSG_HELLO        0x01
#define MSG_HELLO_ACK    0x02
#define MSG_POS_REPORT   0x10
#define MSG_MOVE_CMD     0x20
#define MSG_ARRIVED      0x21
#define MSG_PHASE_ACK    0x30
#define MSG_TERMINATE    0xFF

#define HEADER_SIZE      3

/* 한 메시지 단위로 송신 (TYPE 1B + LEN 2B + payload). */
int  send_msg(int fd, uint8_t type, const void *payload, uint16_t len);

/* 한 메시지 단위로 수신.
 * 반환: 0 정상, -1 오류(프레임 깨짐 등), -2 상대측 종료. */
int  recv_msg(int fd, uint8_t *type, void *buf, uint16_t bufsize, uint16_t *len);

/* 좌표 인코딩: m → mm 단위 int32 (네트워크 바이트 오더 직렬화는 pack_int32 사용). */
int32_t coord_to_wire(double m);
double  coord_from_wire(int32_t w);

/* 페이로드 정수 필드 직렬화 헬퍼. */
void    pack_int32(uint8_t *buf, int offset, int32_t value);
int32_t unpack_int32(const uint8_t *buf, int offset);

#endif
