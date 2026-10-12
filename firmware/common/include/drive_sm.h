/* drive_sm — pure protocol-v9 drive state machine (HAL-free, host-testable).
 *
 * Implements the request × state matrix, fault-reaction table and transient-state
 * progression of docs/protocol_v9.md. Replaces mode_sm (the v8 mode-request machine)
 * when the slave runtime swaps over; arm_sm keeps sequencing ENABLING's CAN sends,
 * but completion is judged here (rs_mode == Run seen / timeout), not by fixed waits.
 *
 * Three entry points, all pure (state in, decision out — the caller owns storage):
 *   drive_sm_on_request(state, req, wound, fault_latched)  host request this cycle
 *   drive_sm_on_fault(state, fault_latched)                a fault was DETECTED
 *   drive_sm_on_tick(state, inputs)                        per-cycle progression
 *
 * Latching discipline (spec §2): drive_fault latches at DETECTION, first fault wins —
 * the caller latches when a decision sets `latch_fault`, and passes fault_latched =
 * (drive_fault != DRVF_NONE) back in. A fault-triggered STOPPING therefore ends in
 * FAULT even if a DISABLE lands mid-ramp. drive_fault is cleared on CLEARING entry
 * (clear_fault), so a re-latch reports the current fault, not the original one.
 */
#ifndef DRIVE_SM_H
#define DRIVE_SM_H

#include <stdint.h>
#include <string.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Decision from a host drive request. At most one send_* / begin_* is set. */
typedef struct {
    uint8_t next_state;     /* DriveState                                        */
    uint8_t rejected;       /* set TELE_FLAG_REQUEST_REJECTED                    */
    uint8_t send_disable;   /* emit Type-4 (plain stop) now                      */
    uint8_t send_clear;     /* emit Type-4 Byte0=1 (stop + clear RS fault) now   */
    uint8_t begin_enable;   /* start the ENABLING CAN sequence (arm_sm)          */
    uint8_t begin_stop;     /* start the stop ramp                               */
    uint8_t abort_enable;   /* cancel an in-progress ENABLING sequence           */
    uint8_t latch_fault;    /* DRVF_* to latch (DRVF_NONE = nothing)             */
    uint8_t clear_fault;    /* zero drive_fault (CLEARING entry)                 */
    uint8_t reset_cmd_seq;  /* re-baseline cmd_seq tracking                      */
} DriveReqDecision;

/* Decision from a detected fault. */
typedef struct {
    uint8_t next_state;
    uint8_t latch_fault;    /* DRVF_* to latch; DRVF_NONE if first-fault-wins
                               suppressed it (an earlier fault is already latched) */
    uint8_t send_disable;   /* immediate Type-4 (comms-broken reaction)          */
    uint8_t begin_stop;     /* enter the stop ramp (comms-intact reaction)       */
    uint8_t abort_enable;   /* cancel an in-progress ENABLING sequence           */
} DriveFaultDecision;

/* Per-cycle inputs summarised by the runtime for the transient states. */
typedef struct {
    uint8_t run_seen;       /* ENABLING: fresh feedback showed rs_mode == Run     */
    uint8_t enable_timeout; /* ENABLING: ENABLE_TIMEOUT_MS elapsed since begin    */
    uint8_t clear_reply;    /* CLEARING: fresh Type-2 seen since entry            */
    uint8_t clear_dirty;    /* CLEARING: that reply still carries fault bits/Cali */
    uint8_t clear_timeout;  /* CLEARING: CLEAR_TIMEOUT_MS elapsed since entry     */
    uint8_t stop_done;      /* STOPPING: ramp finished (ramp sent its disable)    */
    uint8_t fault_latched;  /* drive_fault != DRVF_NONE                           */
    uint8_t fb_timeout;     /* DISABLED: feedback older than the ABSENT threshold */
    uint8_t fb_fresh;       /* ABSENT: any feedback arrived                       */
} DriveTickInputs;

typedef struct {
    uint8_t next_state;
    uint8_t latch_fault;    /* DRVF_* to latch (ENABLE_FAILED / MOTOR_REPORTED)  */
    uint8_t send_disable;   /* emit Type-4 now (failed enable)                   */
    uint8_t enter_enabled;  /* ENABLING→ENABLED edge: apply any latched valid
                               tuple this cycle, else start the limp zero-poll   */
} DriveTickDecision;

static inline uint8_t drive_sm_comms_broken(uint8_t fault)
{
    return (uint8_t)(fault == DRVF_CAN_TIMEOUT || fault == DRVF_NOT_ENABLED ||
                     fault == DRVF_MOTOR_REPORTED || fault == DRVF_ENABLE_FAILED);
}

/* ── host request (level-triggered; see matrix in docs/protocol_v9.md §3) ──── */
static inline DriveReqDecision drive_sm_on_request(uint8_t state, uint8_t req,
                                                   uint8_t wound,
                                                   uint8_t fault_latched)
{
    DriveReqDecision d;
    memset(&d, 0, sizeof(d));
    d.next_state  = state;
    d.latch_fault = DRVF_NONE;

    /* CLEARING ignores every request until it resolves (no rejected flag). */
    if (state == DRV_CLEARING) return d;

    /* BOOT/DISCOVERING accept nothing. */
    if (state == DRV_BOOT || state == DRV_DISCOVERING) {
        d.rejected = 1u;
        return d;
    }

    switch (req) {
    case DRV_REQ_DISABLE:
        if (state == DRV_ENABLING) {
            d.abort_enable = 1u;
            d.send_disable = 1u;
            d.next_state   = DRV_DISABLED;
            d.reset_cmd_seq = 1u;
        } else if (state == DRV_ENABLED) {
            d.send_disable = 1u;
            d.next_state   = DRV_DISABLED;
            d.reset_cmd_seq = 1u;
        } else if (state == DRV_STOPPING) {
            d.send_disable = 1u;             /* cut the ramp short            */
            d.next_state   = fault_latched ? (uint8_t)DRV_FAULT
                                           : (uint8_t)DRV_DISABLED;
            d.reset_cmd_seq = 1u;
        }
        /* ABSENT / DISABLED / FAULT: no-op (FAULT stays latched). */
        break;

    case DRV_REQ_ENABLE:
        if (state == DRV_DISABLED) {
            if (wound) {                     /* refuse: never enable a wound shaft */
                d.next_state  = DRV_FAULT;
                d.latch_fault = DRVF_WOUND;
                d.rejected    = 1u;
            } else {
                d.begin_enable  = 1u;
                d.next_state    = DRV_ENABLING;
                d.reset_cmd_seq = 1u;
            }
        } else if (state == DRV_ENABLING || state == DRV_ENABLED ||
                   state == DRV_STOPPING) {
            /* repeat / racing a stop: no-op */
        } else {
            d.rejected = 1u;                 /* ABSENT / FAULT                */
        }
        break;

    case DRV_REQ_STOP:
        if (state == DRV_ENABLED) {
            d.begin_stop    = 1u;
            d.next_state    = DRV_STOPPING;
            d.reset_cmd_seq = 1u;
        } else if (state == DRV_ENABLING) {  /* nothing to ramp yet           */
            d.abort_enable  = 1u;
            d.send_disable  = 1u;
            d.next_state    = DRV_DISABLED;
            d.reset_cmd_seq = 1u;
        }
        /* ABSENT / DISABLED / STOPPING / FAULT: no-op. */
        break;

    case DRV_REQ_RESET:
        if (state == DRV_FAULT) {
            d.clear_fault = 1u;              /* re-latch reports the CURRENT fault */
            d.send_clear  = 1u;              /* Type-4 Byte0=1 on entry        */
            d.next_state  = DRV_CLEARING;
        } else if (state == DRV_DISABLED) {
            /* no-op: already clean */
        } else {
            d.rejected = 1u;                 /* ABSENT / ENABLING / ENABLED / STOPPING */
        }
        break;

    default:
        d.rejected = 1u;
        break;
    }
    return d;
}

/* ── fault detected (spec §4). The caller latches `latch_fault` if nonzero. ── */
static inline DriveFaultDecision drive_sm_on_fault(uint8_t state, uint8_t fault,
                                                   uint8_t fault_latched)
{
    DriveFaultDecision d;
    memset(&d, 0, sizeof(d));
    d.next_state  = state;
    d.latch_fault = fault_latched ? (uint8_t)DRVF_NONE : fault;  /* first wins */

    if (state != DRV_ENABLING && state != DRV_ENABLED && state != DRV_STOPPING &&
        state != DRV_DISABLED) {
        return d;      /* faults are only detectable with a live/driven motor  */
    }

    if (drive_sm_comms_broken(fault)) {
        /* Comms-broken: there is nothing to ramp against — Type-4 now. Also
           abandons an in-progress stop ramp or enable handshake. */
        d.send_disable = 1u;
        d.abort_enable = (uint8_t)(state == DRV_ENABLING);
        d.next_state   = DRV_FAULT;
    } else {
        /* Comms-intact (OVERTORQUE, MASTER_LOST): ramp from ENABLED; anywhere
           else there is no meaningful motion to ramp down — Type-4 now. A
           repeat detection while already STOPPING changes nothing. */
        if (state == DRV_ENABLED) {
            d.begin_stop = 1u;
            d.next_state = DRV_STOPPING;
        } else if (state == DRV_STOPPING) {
            /* already ramping; the latch (if first) is the only effect */
        } else {
            d.send_disable = 1u;
            d.abort_enable = (uint8_t)(state == DRV_ENABLING);
            d.next_state   = DRV_FAULT;
        }
    }
    return d;
}

/* ── per-cycle progression of the transient states (spec §2). ──────────────── */
static inline DriveTickDecision drive_sm_on_tick(uint8_t state,
                                                 const DriveTickInputs *in)
{
    DriveTickDecision d;
    memset(&d, 0, sizeof(d));
    d.next_state  = state;
    d.latch_fault = DRVF_NONE;

    switch (state) {
    case DRV_ENABLING:
        if (in->run_seen) {
            d.next_state    = DRV_ENABLED;
            d.enter_enabled = 1u;
        } else if (in->enable_timeout) {
            d.latch_fault  = in->fault_latched ? (uint8_t)DRVF_NONE
                                               : (uint8_t)DRVF_ENABLE_FAILED;
            d.send_disable = 1u;
            d.next_state   = DRV_FAULT;
        }
        break;

    case DRV_CLEARING:
        if (in->clear_reply) {
            if (in->clear_dirty) {
                d.latch_fault = DRVF_MOTOR_REPORTED;   /* fresh latch: fault was
                                                          cleared on entry      */
                d.next_state  = DRV_FAULT;
            } else {
                d.next_state = DRV_DISABLED;
            }
        } else if (in->clear_timeout) {
            d.next_state = DRV_ABSENT;
        }
        break;

    case DRV_STOPPING:
        if (in->stop_done) {
            d.next_state = in->fault_latched ? (uint8_t)DRV_FAULT
                                             : (uint8_t)DRV_DISABLED;
        }
        break;

    case DRV_DISABLED:
        if (in->fb_timeout) d.next_state = DRV_ABSENT;
        break;

    case DRV_ABSENT:
        if (in->fb_fresh) d.next_state = DRV_DISABLED;
        break;

    default:
        break;
    }
    return d;
}

#ifdef __cplusplus
}
#endif
#endif /* DRIVE_SM_H */
