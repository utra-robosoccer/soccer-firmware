/* drive_sm — pure per-motor drive state machine (HAL-free, host-testable).
 *
 * Implements the request × state matrix, fault-reaction table and transient-state
 * progression over the protocol_wire.h MotorState/MotorReq/MotorFault enums.
 * arm_sm keeps sequencing ENABLING's CAN sends; completion is judged here
 * (rs_mode == Run seen / timeout), not by fixed waits.
 *
 * Three entry points, all pure (state in, decision out — the caller owns storage):
 *   drive_sm_on_request(state, req, fault_latched)  host request this cycle
 *   drive_sm_on_fault(state, fault, fault_latched)  a fault was DETECTED
 *   drive_sm_on_tick(state, inputs)                 per-cycle progression
 *
 * Latching discipline: fault latches at DETECTION, first fault wins — the caller
 * latches when a decision sets `latch_fault`, and passes fault_latched =
 * (fault != MOTOR_FAULT_NONE) back in. A fault-triggered STOPPING therefore ends
 * in MOTOR_FAULT even if a STOP lands mid-ramp. There is no per-motor clear:
 * MOTOR_FAULT and MOTOR_ABSENT leave only through SLAVE_REQ_RESET (the slave-level
 * blocking re-sweep), which re-initializes the runtime outside this machine.
 */
#ifndef DRIVE_SM_H
#define DRIVE_SM_H

#include <stdint.h>
#include <string.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Decision from a host motor request. At most one send_* / begin_* is set. */
typedef struct {
    uint8_t next_state;     /* MotorState                                        */
    uint8_t rejected;       /* set MOTOR_FLAG_REQ_REJECTED                       */
    uint8_t send_stop;      /* emit Type-4 (plain stop) now                      */
    uint8_t begin_enable;   /* start the ENABLING CAN sequence (arm_sm)          */
    uint8_t begin_stop;     /* start the controlled-stop damp                    */
    uint8_t abort_enable;   /* cancel an in-progress ENABLING sequence           */
    uint8_t reset_cmd_seq;  /* re-baseline cmd_seq tracking                      */
} DriveReqDecision;

/* Decision from a detected fault. */
typedef struct {
    uint8_t next_state;
    uint8_t latch_fault;    /* MOTOR_FAULT_* to latch; NONE if first-fault-wins
                               suppressed it (an earlier fault is already latched) */
    uint8_t send_stop;      /* immediate Type-4 (EMERGENCY reaction)             */
    uint8_t begin_stop;     /* enter the damp (CONTROLLED reaction)              */
    uint8_t abort_enable;   /* cancel an in-progress ENABLING sequence           */
} DriveFaultDecision;

/* Per-cycle inputs summarised by the runtime for the transient states. */
typedef struct {
    uint8_t run_seen;       /* ENABLING: fresh feedback showed rs_mode == Run     */
    uint8_t enable_timeout; /* ENABLING: ENABLE_TIMEOUT_MS elapsed since begin    */
    uint8_t stop_done;      /* STOPPING: damp dwell finished (ramp sent its stop) */
    uint8_t fault_latched;  /* fault != MOTOR_FAULT_NONE                          */
    uint8_t fb_timeout;     /* DISABLED: feedback older than the ABSENT threshold */
} DriveTickInputs;

typedef struct {
    uint8_t next_state;
    uint8_t latch_fault;    /* MOTOR_FAULT_* to latch (ENABLE_FAILED)            */
    uint8_t send_stop;      /* emit Type-4 now (failed enable)                   */
    uint8_t enter_enabled;  /* ENABLING→ENABLED edge: apply the latched mit now  */
} DriveTickDecision;

/* Faults that react with an EMERGENCY stop (Type-4 now, straight to MOTOR_FAULT;
 * the runtime's head-of-burst repair rule keeps re-sending the stop until the
 * motor reports Reset). Comms-broken faults have nothing to ramp against, and
 * OVERTORQUE must cut effort NOW, not damp through it. The only CONTROLLED
 * fault is MASTER_LOST. */
static inline uint8_t drive_sm_fault_immediate(uint8_t fault)
{
    return (uint8_t)(fault != MOTOR_FAULT_NONE && fault != MOTOR_FAULT_MASTER_LOST);
}

/* ── host request (level-triggered) ───────────────────────────────────────── */
static inline DriveReqDecision drive_sm_on_request(uint8_t state, uint8_t req,
                                                   uint8_t fault_latched)
{
    DriveReqDecision d;
    memset(&d, 0, sizeof(d));
    d.next_state = state;

    switch (req) {
    case MOTOR_REQ_STOP:
        if (state == MOTOR_ENABLED) {
            d.begin_stop    = 1u;
            d.next_state    = MOTOR_STOPPING;
            d.reset_cmd_seq = 1u;
        } else if (state == MOTOR_ENABLING) {  /* nothing to ramp yet          */
            d.abort_enable  = 1u;
            d.send_stop     = 1u;
            d.next_state    = fault_latched ? (uint8_t)MOTOR_FAULT
                                            : (uint8_t)MOTOR_DISABLED;
            d.reset_cmd_seq = 1u;
        }
        /* ABSENT / DISABLED / STOPPING / FAULT: no-op (STOP is the resting
           request; FAULT stays latched until SLAVE_REQ_RESET). */
        break;

    case MOTOR_REQ_ENABLE:
        if (state == MOTOR_DISABLED) {
            /* Readiness gating (fresh feedback, Reset mode, no fault bits,
               position in range) is the CALLER's job — the verified enable
               gate rejects before this machine ever sees the request. */
            d.begin_enable  = 1u;
            d.next_state    = MOTOR_ENABLING;
            d.reset_cmd_seq = 1u;
        } else if (state == MOTOR_ENABLING || state == MOTOR_ENABLED) {
            /* level-triggered repeat: no-op */
        } else {
            d.rejected = 1u;                 /* ABSENT / STOPPING / FAULT     */
        }
        break;

    default:
        d.rejected = 1u;
        break;
    }
    return d;
}

/* ── fault detected. The caller latches `latch_fault` if nonzero. ──────────── */
static inline DriveFaultDecision drive_sm_on_fault(uint8_t state, uint8_t fault,
                                                   uint8_t fault_latched)
{
    DriveFaultDecision d;
    memset(&d, 0, sizeof(d));
    d.next_state  = state;
    d.latch_fault = fault_latched ? (uint8_t)MOTOR_FAULT_NONE : fault; /* first wins */

    if (state != MOTOR_ENABLING && state != MOTOR_ENABLED &&
        state != MOTOR_STOPPING && state != MOTOR_DISABLED) {
        return d;      /* faults are only detectable with a live/driven motor  */
    }

    if (drive_sm_fault_immediate(fault)) {
        /* EMERGENCY: Type-4 now, straight to FAULT. Also abandons an
           in-progress damp or enable handshake. */
        d.send_stop    = 1u;
        d.abort_enable = (uint8_t)(state == MOTOR_ENABLING);
        d.next_state   = MOTOR_FAULT;
    } else {
        /* CONTROLLED (MASTER_LOST only): damp from ENABLED; anywhere else
           there is no meaningful motion to ramp down — Type-4 now. A repeat
           detection while already STOPPING changes nothing. */
        if (state == MOTOR_ENABLED) {
            d.begin_stop = 1u;
            d.next_state = MOTOR_STOPPING;
        } else if (state == MOTOR_STOPPING) {
            /* already ramping; the latch (if first) is the only effect */
        } else {
            d.send_stop    = 1u;
            d.abort_enable = (uint8_t)(state == MOTOR_ENABLING);
            d.next_state   = MOTOR_FAULT;
        }
    }
    return d;
}

/* ── per-cycle progression of the transient states ─────────────────────────── */
static inline DriveTickDecision drive_sm_on_tick(uint8_t state,
                                                 const DriveTickInputs *in)
{
    DriveTickDecision d;
    memset(&d, 0, sizeof(d));
    d.next_state  = state;
    d.latch_fault = MOTOR_FAULT_NONE;

    switch (state) {
    case MOTOR_ENABLING:
        if (in->run_seen) {
            d.next_state    = MOTOR_ENABLED;
            d.enter_enabled = 1u;
        } else if (in->enable_timeout) {
            d.latch_fault = in->fault_latched ? (uint8_t)MOTOR_FAULT_NONE
                                              : (uint8_t)MOTOR_FAULT_ENABLE_FAILED;
            d.send_stop   = 1u;
            d.next_state  = MOTOR_FAULT;
        }
        break;

    case MOTOR_STOPPING:
        if (in->stop_done) {
            d.next_state = in->fault_latched ? (uint8_t)MOTOR_FAULT
                                             : (uint8_t)MOTOR_DISABLED;
        }
        break;

    case MOTOR_DISABLED:
        /* Feedback gone stale while off → the motor fell off the bus. ABSENT
           is terminal until SLAVE_REQ_RESET re-runs the sweep. */
        if (in->fb_timeout) d.next_state = MOTOR_ABSENT;
        break;

    default:   /* ABSENT (terminal), ENABLED, FAULT: nothing time-driven here */
        break;
    }
    return d;
}

#ifdef __cplusplus
}
#endif
#endif /* DRIVE_SM_H */
