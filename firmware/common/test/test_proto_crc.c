/* Cross-check harness for the byte-wise CRC table (task 1b).
 *
 * Proves, in C, that the new table-driven proto_crc16() is byte-identical to the
 * old bit-by-bit algorithm over 10 000 pseudo-random buffers of random length
 * (iteration 0 = empty, iteration 1 = the max frame size). It prints "<len> <crc>"
 * per iteration; test_proto_crc.py mirrors the identical LCG to rebuild each buffer
 * and asserts binascii.crc_hqx(buf, 0xFFFF) == the printed (table) CRC — so the
 * three implementations (C table, C bitwise, host crc_hqx) all agree.
 *
 * Built by host/tests/test_proto_crc.py:
 *   gcc -std=c11 -Wall -Wextra -Werror -I firmware/common/include ... */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "protocol.h"

#define ITERS   10000u
#define MAXLEN  512u          /* ≥ the 496 B max USB frame (16 hdr + 480 tele_robot) */

/* Reference: the original bit-by-bit CRC-16-CCITT this table replaces. */
static uint16_t ref_bitwise(const uint8_t *data, size_t len)
{
    uint16_t crc = PROTO_CRC_INIT;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8u);
        for (uint8_t b = 0; b < 8u; ++b) {
            crc = (crc & 0x8000u)
                ? (uint16_t)((crc << 1u) ^ PROTO_CRC_POLY)
                : (uint16_t)(crc << 1u);
        }
    }
    return crc;
}

/* Shared LCG — must stay bit-identical to the one in test_proto_crc.py. */
static uint32_t g_lcg = 0x12345678u;
static uint32_t lcg_next(void)
{
    g_lcg = (uint32_t)(g_lcg * 1664525u + 1013904223u);
    return g_lcg;
}

int main(void)
{
    static uint8_t buf[MAXLEN];
    for (uint32_t i = 0; i < ITERS; ++i) {
        uint32_t len;
        if (i == 0u)      len = 0u;
        else if (i == 1u) len = MAXLEN;
        else              len = lcg_next() % (MAXLEN + 1u);

        for (uint32_t k = 0; k < len; ++k) buf[k] = (uint8_t)(lcg_next() & 0xFFu);

        uint16_t table = proto_crc16(buf, len);
        uint16_t bits  = ref_bitwise(buf, len);
        if (table != bits) {
            fprintf(stderr, "MISMATCH table!=bitwise at i=%u len=%u: %04X vs %04X\n",
                    i, len, table, bits);
            return 1;
        }
        printf("%u %04X\n", len, table);
    }
    return 0;
}
