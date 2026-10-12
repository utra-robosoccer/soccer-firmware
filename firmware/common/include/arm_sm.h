/* arm_sm — pure async-arm phase machine (HAL-free, host-testable).
 *
 * Replaces the blocking CAN enable handshake (~23 ms/motor of HAL_Delay + ACK spins,
 * ~116 ms loop stall arming 5 motors at once) with ONE action per master cycle, stepped
 * from motor_runtime_update. All motors may arm concurrently: a step emits at most one
 * CAN frame per motor per cycle — the same bus budget as IDLE-state polling — so total
 * arm time is a fixed number of cycles regardless of motor count.
 *
 * The inter-phase waits are fixed cycle counts sized to the proven blocking sequence's
 * worst case (ACK wait ≤10 ms + 10 ms settle; 5 ms after a fault clear). There is no
 * in-band ACK tracking here: confirmation that the motor actually ENABLED remains the
 * enable monitor's job once the motor starts driving (HOLD), exactly as before.
 *
 * The phase/wait registers live in MotorRuntime as plain uint8_t so this header stays
 * dependency-free. Callers: arm_sm_begin on the arm decision (idempotent arms must be
 * guarded by arm_sm_active), arm_sm_step once per cycle, arm_sm_cancel on an IDLE
 * request or master loss mid-arm.
 */
#ifndef ARM_SM_H
#define ARM_SM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Phases: the value names the NEXT action to emit. */
enum {
    ARM_PHASE_IDLE = 0,    /* no arm in progress                        */
    ARM_PHASE_CLEAR,       /* next: clear the latched motor fault        */
    ARM_PHASE_MODE,        /* next: write the MIT run mode               */
    ARM_PHASE_ENABLE,      /* next: enable the motor                     */
    ARM_PHASE_FINISH       /* next: capture hold + enter HOLD (complete) */
};

/* Actions returned by arm_sm_step, one per cycle at most. */
enum {
    ARM_ACT_NONE = 0,
    ARM_ACT_SEND_CLEAR,
    ARM_ACT_SEND_MODE,
    ARM_ACT_SEND_ENABLE,
    ARM_ACT_COMPLETE
};

static inline uint8_t arm_sm_active(uint8_t phase)
{
    return (uint8_t)(phase != ARM_PHASE_IDLE);
}

/* Start a handshake. A latched motor fault inserts the CLEAR phase first (the RS
   refuses enable otherwise). The first action is emitted on the next step. */
static inline void arm_sm_begin(uint8_t *phase, uint8_t *wait, uint8_t fault_latched)
{
    *phase = fault_latched ? (uint8_t)ARM_PHASE_CLEAR : (uint8_t)ARM_PHASE_MODE;
    *wait  = 0u;
}

static inline void arm_sm_cancel(uint8_t *phase, uint8_t *wait)
{
    *phase = ARM_PHASE_IDLE;
    *wait  = 0u;
}

/* One step per master cycle. clear_wait / settle_wait are the cycle counts to pause
   AFTER the clear / mode / enable sends. Returns the ARM_ACT_* to perform this cycle;
   ARM_ACT_COMPLETE means the handshake is done and the caller enters HOLD. */
static inline uint8_t arm_sm_step(uint8_t *phase, uint8_t *wait,
                                  uint8_t clear_wait, uint8_t settle_wait)
{
    if (*phase == ARM_PHASE_IDLE) return ARM_ACT_NONE;
    if (*wait > 0u) { (*wait)--; return ARM_ACT_NONE; }
    switch (*phase) {
    case ARM_PHASE_CLEAR:  *phase = ARM_PHASE_MODE;   *wait = clear_wait;  return ARM_ACT_SEND_CLEAR;
    case ARM_PHASE_MODE:   *phase = ARM_PHASE_ENABLE; *wait = settle_wait; return ARM_ACT_SEND_MODE;
    case ARM_PHASE_ENABLE: *phase = ARM_PHASE_FINISH; *wait = settle_wait; return ARM_ACT_SEND_ENABLE;
    case ARM_PHASE_FINISH: *phase = ARM_PHASE_IDLE;   *wait = 0u;          return ARM_ACT_COMPLETE;
    default:               *phase = ARM_PHASE_IDLE;   *wait = 0u;          return ARM_ACT_NONE;
    }
}

#ifdef __cplusplus
}
#endif
#endif /* ARM_SM_H */
