/* Harness for the RobStride ext-id codec (robostride_id.h), driven by
 * host/tests/test_robostride_id.py.
 *
 * Asserts pack→unpack round-trips and that the id fits 29 bits, over 20 000
 * pseudo-random (mode,data,id) triples, and prints "<mode> <data> <id> <ext>" so
 * Python can re-check the wire formula ext = mode<<24 | data<<8 | id and the
 * known-id vectors (tx frames + the type-2 feedback reply we parse).
 *
 * Built: gcc -std=c11 -Wall -Wextra -Werror -I firmware/common/include ... */
#include <stdint.h>
#include <stdio.h>
#include "robostride_id.h"

static uint32_t g_lcg = 0x00C0FFEEu;
static uint32_t lcg_next(void)
{
    g_lcg = (uint32_t)(g_lcg * 1664525u + 1013904223u);
    return g_lcg;
}

int main(void)
{
    for (int i = 0; i < 20000; ++i) {
        uint8_t  mode = (uint8_t)(lcg_next() & 0x1Fu);
        uint16_t data = (uint16_t)(lcg_next() & 0xFFFFu);
        uint8_t  id   = (uint8_t)(lcg_next() & 0xFFu);

        uint32_t ext = rs_extid_pack(mode, data, id);
        if (rs_extid_mode(ext) != mode || rs_extid_data(ext) != data ||
            rs_extid_id(ext) != id) {
            fprintf(stderr, "roundtrip fail i=%d\n", i);
            return 1;
        }
        if ((ext >> 29) != 0u) {               /* must fit the 29-bit ext id */
            fprintf(stderr, "bits>28 set i=%d ext=%08lX\n", i, (unsigned long)ext);
            return 2;
        }
        printf("%u %u %u %08lX\n", mode, data, id, (unsigned long)ext);
    }
    return 0;
}
