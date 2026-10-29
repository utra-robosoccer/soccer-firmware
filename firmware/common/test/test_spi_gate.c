/* Host tests for the SPI version gate (firmware/common/include/spi_gate.h).
 *
 * Pins:
 *   - trailer write → match (the normal case);
 *   - EVERY single-byte corruption of the trailer → mismatch (wrong version,
 *     and each byte of a wrong schema hash);
 *   - recovery: a matching trailer after a mismatched one is accepted again —
 *     the gate is stateless per frame, so a reflashed board needs no reset;
 *   - the robot-wide enable filter: ENABLE is rewritten to DISABLE only for a
 *     motor that is not already armed; every other request passes through.
 *
 * Build: gcc -std=c11 -Wall -Wextra -Werror -I firmware/common/include \
 *            firmware/common/test/test_spi_gate.c -o test_spi_gate
 */
#include "spi_gate.h"
#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

static void test_trailer_match_mismatch_recovery(void)
{
    uint8_t t[5];
    spi_trailer_write(t);
    CHECK(t[0] == (uint8_t)PROTO_VERSION, "trailer byte0 = PROTO_VERSION");
    CHECK(spi_trailer_match(t) == 1u, "own trailer must match");

    /* Every single-byte corruption must mismatch. */
    for (unsigned i = 0; i < sizeof(t); i++) {
        uint8_t bad[5];
        memcpy(bad, t, sizeof(t));
        bad[i] ^= 0x01u;
        CHECK(spi_trailer_match(bad) == 0u, "corrupted trailer accepted");
    }

    /* A v8-style frame (no trailer concept → whatever bytes land there). */
    uint8_t v8[5] = { 8u, 0u, 0u, 0u, 0u };
    CHECK(spi_trailer_match(v8) == 0u, "old-version trailer accepted");

    /* Recovery: a matching trailer right after mismatches is accepted again. */
    CHECK(spi_trailer_match(t) == 1u, "match after mismatch must recover");
}

static void test_enable_filter(void)
{
    /* ENABLE is refused (→ DISABLE) for every not-armed state... */
    const uint8_t not_armed[] = { DRV_BOOT, DRV_DISCOVERING, DRV_ABSENT,
                                  DRV_DISABLED, DRV_CLEARING, DRV_FAULT };
    for (unsigned i = 0; i < sizeof(not_armed); i++) {
        CHECK(spi_gate_filter_req(DRV_REQ_ENABLE, not_armed[i]) == DRV_REQ_DISABLE,
              "ENABLE not refused for a not-armed motor");
    }
    /* ...but passes for an already-armed motor (no mid-run force-disable). */
    const uint8_t armed[] = { DRV_ENABLING, DRV_ENABLED, DRV_STOPPING };
    for (unsigned i = 0; i < sizeof(armed); i++) {
        CHECK(spi_gate_filter_req(DRV_REQ_ENABLE, armed[i]) == DRV_REQ_ENABLE,
              "ENABLE must pass for an armed motor");
    }
    /* Every other request passes through in every state. */
    const uint8_t reqs[] = { DRV_REQ_DISABLE, DRV_REQ_STOP, DRV_REQ_RESET };
    for (unsigned r = 0; r < sizeof(reqs); r++) {
        for (uint8_t st = DRV_BOOT; st <= DRV_FAULT; st++) {
            CHECK(spi_gate_filter_req(reqs[r], st) == reqs[r],
                  "non-ENABLE request altered");
        }
    }
}

int main(void)
{
    test_trailer_match_mismatch_recovery();
    test_enable_filter();
    if (failures == 0) {
        printf("OK spi_gate\n");
        return 0;
    }
    printf("test_spi_gate: %d failure(s)\n", failures);
    return 1;
}
