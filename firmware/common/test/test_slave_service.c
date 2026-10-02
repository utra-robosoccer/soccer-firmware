/* Tests for slave_service_due() (slave_service.h) — forward-on-command servicing.
 * Driven by host/tests/test_slave_service.py. Thresholds in µs here (0.5 & 1.6 of a
 * 5 ms master cycle); the slave uses the same logic in CPU-cycle units. */
#include <stdio.h>
#include "slave_service.h"

static int fails = 0;
#define CK(expr, want, msg) do { uint8_t g=(expr); \
    if (g != (want)) { fprintf(stderr,"FAIL %s: got %u want %u\n",msg,g,(want)); fails++; } } while(0)

int main(void)
{
    const uint32_t MIN = 2500u, FB = 8000u;   /* 0.5 and 1.6 of a 5000 µs cycle */

    /* Service on EVERY valid exchange — ROBOT_CMD and NOP both arrive as exch_valid=1. */
    CK(slave_service_due(SVC_EXCHANGE, 1, 5000, 0, 0, MIN, FB), 1, "exchange valid (cmd or nop)");
    CK(slave_service_due(SVC_EXCHANGE, 1, 2500, 0, 0, MIN, FB), 1, "exchange at min boundary");

    /* Nothing on a CRC-failed exchange. */
    CK(slave_service_due(SVC_EXCHANGE, 0, 5000, 0, 0, MIN, FB), 0, "exchange crc-fail → none");

    /* No double service within one cycle (another exchange < min since last send). */
    CK(slave_service_due(SVC_EXCHANGE, 1, 1000, 0, 0, MIN, FB), 0, "exchange within min → none");

    /* Fallback ONLY after the exchange gap. */
    CK(slave_service_due(SVC_FALLBACK, 1, 8000, 0, 0, MIN, FB), 1, "fallback after gap");
    CK(slave_service_due(SVC_FALLBACK, 1, 6000, 0, 0, MIN, FB), 0, "fallback gap < fallback");
    CK(slave_service_due(SVC_FALLBACK, 1, 9000, 8000, 0, MIN, FB), 0, "fallback within min of last send");

    /* Fallback keeps firing every tick during a sustained gap (no self-throttle):
       last_exch stays old, last_send = previous fallback one cycle ago. */
    CK(slave_service_due(SVC_FALLBACK, 1, 10000, 5000, 0, MIN, FB), 1, "fallback sustained");

    if (fails) { fprintf(stderr, "%d checks failed\n", fails); return 1; }
    printf("OK slave_service\n");
    return 0;
}
