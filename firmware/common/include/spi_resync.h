#ifndef SPI_RESYNC_H
#define SPI_RESYNC_H
#include <stdint.h>

/* NSS-gating decision for the slave SPI DMA resync (see slave_spi.c).
 *
 * The slave's SPI-slave DMA counts bytes continuously; to re-align it after a bad
 * exchange we must abort + re-arm ONLY between exchanges, i.e. while NSS (PA4) is
 * HIGH (slave deselected). Re-arming while NSS is low starts the new DMA transfer
 * mid-exchange and desyncs again. So the caller polls this until it stops saying
 * WAIT: PROCEED (NSS high → safe to abort+re-arm), or TIMEOUT (NSS stuck low past
 * the bound → skip this attempt, retry next cycle; never block the control loop).
 *
 * Pure (no HAL) so host-tested: host/tests/test_spi_resync.py. `waited`/`timeout`
 * are in the caller's own units (ms via HAL_GetTick on the slave). NSS high always
 * wins over the timeout, so a transfer that ends during the wait proceeds at once. */
typedef enum {
    SPI_RESYNC_PROCEED = 0,  /* NSS high (between exchanges) → abort + re-arm now  */
    SPI_RESYNC_WAIT,         /* NSS low (mid-exchange), still within bound → poll  */
    SPI_RESYNC_TIMEOUT       /* NSS low past the bound → skip, retry next cycle    */
} SpiResyncAction;

static inline SpiResyncAction spi_resync_poll(uint8_t nss_high,
                                              uint32_t waited, uint32_t timeout)
{
    if (nss_high)          return SPI_RESYNC_PROCEED;   /* high wins over timeout */
    if (waited >= timeout) return SPI_RESYNC_TIMEOUT;
    return SPI_RESYNC_WAIT;
}

#endif /* SPI_RESYNC_H */
