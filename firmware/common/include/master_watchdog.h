/* master_watchdog — pure slave-side master-loss ramp (HAL-free, host-testable).
 *
 * Consulted on the slave fallback (no fresh ROBOT_CMD this cycle), from the ms since the
 * last ROBOT_CMD exchange (NOP keepalives do NOT refresh it). The SPI link going quiet
 * means the master reset/died, so the joint ramps down gracefully rather than snapping off:
 *   [0, grace)            HOLD   — keep the last position, but with v_des and tau_ff zeroed
 *                                  so it doesn't coast on a stale velocity
 *   [grace, grace+damp)   DAMPED — Kp=0 + config damping Kd (backdrive-safe)
 *   [grace+damp, ∞)       IDLE   — disable
 */
#ifndef MASTER_WATCHDOG_H
#define MASTER_WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MASTER_HOLD_GRACE = 0u,  /* keep position, zero v_des + tau_ff */
    MASTER_DAMPED     = 1u,  /* backdrive-safe damping (config Kd) */
    MASTER_IDLE       = 2u,  /* disable */
} MasterLossPhase;

static inline MasterLossPhase master_loss_phase(
    uint32_t ms_since_cmd, uint32_t grace_ms, uint32_t damp_ms)
{
    if (ms_since_cmd < grace_ms)             return MASTER_HOLD_GRACE;
    if (ms_since_cmd < grace_ms + damp_ms)   return MASTER_DAMPED;
    return MASTER_IDLE;
}

#ifdef __cplusplus
}
#endif
#endif /* MASTER_WATCHDOG_H */
