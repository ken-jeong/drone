#include "protocol.h"

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

static int send_all(int fd, const void *buf, size_t len) {
    const uint8_t *p = (const uint8_t *)buf;
    size_t left = len;
    while (left > 0) {
        ssize_t n = send(fd, p, left, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) return -1;
        p += n;
        left -= (size_t)n;
    }
    return 0;
}

static int recv_all(int fd, void *buf, size_t len) {
    uint8_t *p = (uint8_t *)buf;
    size_t left = len;
    while (left > 0) {
        ssize_t n = recv(fd, p, left, 0);
        if (n == 0) return -2;
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        p += n;
        left -= (size_t)n;
    }
    return 0;
}

int send_msg(int fd, uint8_t type, const void *payload, uint16_t len) {
    uint8_t header[HEADER_SIZE];
    uint16_t nlen = htons(len);
    header[0] = type;
    memcpy(header + 1, &nlen, 2);
    if (send_all(fd, header, HEADER_SIZE) < 0) return -1;
    if (len > 0 && payload != NULL) {
        if (send_all(fd, payload, len) < 0) return -1;
    }
    return 0;
}

int recv_msg(int fd, uint8_t *type, void *buf, uint16_t bufsize, uint16_t *len) {
    uint8_t header[HEADER_SIZE];
    int r = recv_all(fd, header, HEADER_SIZE);
    if (r != 0) return r;

    uint16_t nlen;
    *type = header[0];
    memcpy(&nlen, header + 1, 2);
    uint16_t plen = ntohs(nlen);
    if (plen > bufsize) return -1;

    if (plen > 0) {
        r = recv_all(fd, buf, plen);
        if (r != 0) return r;
    }
    *len = plen;
    return 0;
}

int32_t coord_to_wire(double m) {
    return (int32_t)(m * 1000.0);
}

double coord_from_wire(int32_t w) {
    return (double)w / 1000.0;
}

void pack_int32(uint8_t *buf, int offset, int32_t value) {
    uint32_t n = htonl((uint32_t)value);
    memcpy(buf + offset, &n, 4);
}

int32_t unpack_int32(const uint8_t *buf, int offset) {
    uint32_t n;
    memcpy(&n, buf + offset, 4);
    return (int32_t)ntohl(n);
}
