/* mode_sm — pure mode-request state machine (no HAL, host-testable).
 *
 * Decides the next MotorLifecycle from (current state, requested mode, flags,
 * wound?) and emits the side-effect actions the caller (motor_runtime) must then
 * perform with the CAN driver. Keeping the decision pure lets the transition
 * table, HOLD-capture, TO_ZERO entry, fault latching and both reset paths be
 * unit-tested with no hardware (firmware/common/test/test_mode_sm.c).
 *
 * Rules:
 *   - HOLD is the only arm-from-IDLE transition (and the only enable).
 *   - From an armed state (HOLD/MIT/DAMPED/TO_ZERO) any armed mode or IDLE is free.
 *   - From IDLE, an armed request other than HOLD is rejected.
 *   - A wound shaft refuses HOLD-arm → FAULT/CAUSE_WOUND.
 *   - FAULT is latched: armed requests are rejected until an IDLE request or the
 *     fault_reset flag clears it (then the SAME request is re-evaluated from IDLE).
 */
#ifndef MODE_SM_H
#define MODE_SM_H

#include <stdint.h>
#include "proto_common.h"   /* MotorLifecycle (LIFE_*), MotorModeReq (REQ_*) */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t next_state;      /* MotorLifecycle to adopt                          */
    uint8_t rejected;        /* request was illegal (set TELE_FLAG_REQUEST_REJECTED) */
    uint8_t set_cause_wound; /* latch CAUSE_WOUND (with next_state == LIFE_FAULT) */
    uint8_t clear_fault;     /* clear the latched fault word + cause              */
    uint8_t do_arm;          /* run the CAN enable handshake (IDLE → HOLD)        */
    uint8_t do_disable;      /* drop CAN output (→ IDLE)                          */
    uint8_t capture_hold;    /* capture current position into hold (armed → HOLD) */
    uint8_t enter_to_zero;   /* initialise the TO_ZERO creep                      */
} ModeDecision;

/* Decide the transition for one motor this cycle.
 *   state        : current MotorLifecycle
 *   req          : requested MotorModeReq
 *   valid        : CMD_FLAG_VALID — 0 means "no request", returns a no-op at state
 *   fault_reset  : CMD_FLAG_FAULT_RESET — clear a latched fault first
 *   wound        : 1 if the shaft is wound beyond the safe range (only consulted
 *                  on an IDLE→HOLD arm) */
ModeDecision mode_sm_step(uint8_t state, uint8_t req, uint8_t valid,
                          uint8_t fault_reset, uint8_t wound);

#ifdef __cplusplus
}
#endif
#endif /* MODE_SM_H */
