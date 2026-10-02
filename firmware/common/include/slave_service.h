#ifndef SLAVE_SERVICE_H
#define SLAVE_SERVICE_H
#include <stdint.h>

/* Forward-on-command servicing decision (see slave main loop).
 *
 * The slave services its motors (one motor_runtime_update → send_mit per motor) once
 * per master cycle. Normally that is driven by the SPI exchange itself (every valid
 * CRC-OK exchange — ROBOT_CMD or NOP — is one master cycle): service immediately so a
 * new command reaches the motor without waiting for the slave's own tick. The slave's
 * periodic tick is only a FALLBACK for when exchanges stop entirely (SPI link lost),
 * so the motors keep holding until the watchdog trips.
 *
 * Decision (all times in one monotonic unit — the slave passes CPU cycles from DWT;
 * the host test uses µs):
 *   SVC_EXCHANGE  (a CRC-OK exchange arrived): service unless we serviced < min ago
 *                 (guards against a bunched pair double-servicing a cycle). A CRC-failed
 *                 exchange is NOT valid → no service (the caller resyncs instead).
 *   SVC_FALLBACK  (slave tick): service only if no exchange has been serviced for
 *                 >= fallback AND we did not just service < min ago.
 * `last_send` = last service of any kind; `last_exch` = last exchange-driven service.
 * Gating the fallback on `last_exch` (not `last_send`) keeps it at the full rate during
 * a real gap instead of self-throttling. Pure (no HAL) → host-tested. */
typedef enum { SVC_EXCHANGE = 0, SVC_FALLBACK } SvcTrigger;

static inline uint8_t slave_service_due(SvcTrigger t, uint8_t exch_valid,
        uint32_t now, uint32_t last_send, uint32_t last_exch,
        uint32_t min_ticks, uint32_t fallback_ticks)
{
    if ((uint32_t)(now - last_send) < min_ticks) return 0u;   /* no double within a cycle */
    if (t == SVC_EXCHANGE) return exch_valid ? 1u : 0u;       /* CRC fail → nothing */
    return (uint32_t)(now - last_exch) >= fallback_ticks;     /* fallback only in a gap */
}

#endif /* SLAVE_SERVICE_H */
