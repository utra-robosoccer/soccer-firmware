/* Tests for spi_resync_poll() (spi_resync.h) — the NSS-gating decision for the
 * slave SPI DMA resync. Driven by host/tests/test_spi_resync.py.
 *
 * Covers the requirement that re-arm happens ONLY while NSS is high (between
 * exchanges): NSS high → PROCEED (even past the timeout); NSS low within the bound
 * → WAIT; NSS low past the bound → TIMEOUT (skip, don't re-arm mid-exchange).
 *
 * Built: gcc -std=c11 -Wall -Wextra -Werror -I firmware/common/include ... */
#include <stdio.h>
#include "spi_resync.h"

static int fails = 0;
#define CHECK(expr, want, msg) do { \
    SpiResyncAction got = (expr); \
    if (got != (want)) { fprintf(stderr, "FAIL %s: got %d want %d\n", msg, got, (want)); fails++; } \
} while (0)

int main(void)
{
    const uint32_t TMO = 5;   /* e.g. 5 ms */

    /* NSS high → proceed immediately, regardless of waited. */
    CHECK(spi_resync_poll(1, 0,   TMO), SPI_RESYNC_PROCEED, "high@0");
    CHECK(spi_resync_poll(1, 3,   TMO), SPI_RESYNC_PROCEED, "high@3");
    /* NSS high must win even if the bound has elapsed (transfer just ended). */
    CHECK(spi_resync_poll(1, TMO, TMO), SPI_RESYNC_PROCEED, "high@timeout");
    CHECK(spi_resync_poll(1, 99,  TMO), SPI_RESYNC_PROCEED, "high>timeout");

    /* NSS low, still within the bound → keep waiting (don't re-arm mid-exchange). */
    CHECK(spi_resync_poll(0, 0,       TMO), SPI_RESYNC_WAIT, "low@0");
    CHECK(spi_resync_poll(0, 1,       TMO), SPI_RESYNC_WAIT, "low@1");
    CHECK(spi_resync_poll(0, TMO - 1, TMO), SPI_RESYNC_WAIT, "low@tmo-1");

    /* NSS low at/past the bound → timeout (skip; retry next cycle). */
    CHECK(spi_resync_poll(0, TMO,     TMO), SPI_RESYNC_TIMEOUT, "low@timeout");
    CHECK(spi_resync_poll(0, TMO + 1, TMO), SPI_RESYNC_TIMEOUT, "low>timeout");

    if (fails) { fprintf(stderr, "%d checks failed\n", fails); return 1; }
    printf("OK spi_resync gating\n");
    return 0;
}
