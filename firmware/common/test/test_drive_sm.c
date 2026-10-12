/* Pure (HAL-free) tests for the protocol-v9 drive state machine (drive_sm.h).
 * Covers every cell of the request × state matrix (docs/protocol_v9.md §3), every
 * fault-reaction row (§4) including the latch-at-detection rule, and the
 * transient-state tick table. Exit 0 = OK. */
#include "drive_sm.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

/* A decision with no actions and no state change. */
static uint8_t req_noop(DriveReqDecision d, uint8_t state)
{
    return (uint8_t)(d.next_state == state && !d.rejected && !d.send_disable &&
                     !d.send_clear && !d.begin_enable && !d.begin_stop &&
                     !d.abort_enable && d.latch_fault == DRVF_NONE &&
                     !d.clear_fault && !d.reset_cmd_seq);
}

static uint8_t req_rej(DriveReqDecision d, uint8_t state)
{
    return (uint8_t)(d.next_state == state && d.rejected && !d.send_disable &&
                     !d.send_clear && !d.begin_enable && !d.begin_stop &&
                     d.latch_fault == DRVF_NONE && !d.clear_fault);
}

static DriveReqDecision R(uint8_t st, uint8_t rq)        /* plain request */
{
    return drive_sm_on_request(st, rq, 0u, 0u);
}

int main(void)
{
    DriveReqDecision d;
    DriveFaultDecision f;
    DriveTickDecision t;
    DriveTickInputs in;

    /* ── request × state matrix ──────────────────────────────────────────── */

    /* BOOT / DISCOVERING: everything rejected. */
    for (uint8_t st = DRV_BOOT; st <= DRV_DISCOVERING; st++)
        for (uint8_t rq = DRV_REQ_DISABLE; rq <= DRV_REQ_RESET; rq++)
            CHECK(req_rej(R(st, rq), st), "BOOT/DISCOVERING reject everything");

    /* CLEARING: everything ignored silently (no rejected flag). */
    for (uint8_t rq = DRV_REQ_DISABLE; rq <= DRV_REQ_RESET; rq++)
        CHECK(req_noop(R(DRV_CLEARING, rq), DRV_CLEARING), "CLEARING ignores all requests");

    /* ABSENT: DISABLE/STOP no-op, ENABLE/RESET rejected. */
    CHECK(req_noop(R(DRV_ABSENT, DRV_REQ_DISABLE), DRV_ABSENT), "ABSENT+DISABLE no-op");
    CHECK(req_noop(R(DRV_ABSENT, DRV_REQ_STOP),    DRV_ABSENT), "ABSENT+STOP no-op");
    CHECK(req_rej (R(DRV_ABSENT, DRV_REQ_ENABLE),  DRV_ABSENT), "ABSENT+ENABLE rejected");
    CHECK(req_rej (R(DRV_ABSENT, DRV_REQ_RESET),   DRV_ABSENT), "ABSENT+RESET rejected");

    /* DISABLED: only ENABLE acts. */
    CHECK(req_noop(R(DRV_DISABLED, DRV_REQ_DISABLE), DRV_DISABLED), "DISABLED+DISABLE no-op");
    CHECK(req_noop(R(DRV_DISABLED, DRV_REQ_STOP),    DRV_DISABLED), "DISABLED+STOP no-op");
    CHECK(req_noop(R(DRV_DISABLED, DRV_REQ_RESET),   DRV_DISABLED), "DISABLED+RESET no-op");
    d = R(DRV_DISABLED, DRV_REQ_ENABLE);
    CHECK(d.next_state == DRV_ENABLING && d.begin_enable && d.reset_cmd_seq &&
          !d.rejected && !d.send_disable, "DISABLED+ENABLE → ENABLING");

    /* DISABLED + ENABLE with a wound shaft: FAULT(WOUND), never enables. */
    d = drive_sm_on_request(DRV_DISABLED, DRV_REQ_ENABLE, 1u, 0u);
    CHECK(d.next_state == DRV_FAULT && d.latch_fault == DRVF_WOUND &&
          d.rejected && !d.begin_enable, "wound ENABLE → FAULT(WOUND), no enable");

    /* ENABLING: DISABLE and STOP both abort+disable; ENABLE no-op; RESET rejected. */
    d = R(DRV_ENABLING, DRV_REQ_DISABLE);
    CHECK(d.next_state == DRV_DISABLED && d.abort_enable && d.send_disable &&
          d.reset_cmd_seq, "ENABLING+DISABLE aborts handshake");
    d = R(DRV_ENABLING, DRV_REQ_STOP);
    CHECK(d.next_state == DRV_DISABLED && d.abort_enable && d.send_disable,
          "ENABLING+STOP = DISABLE (nothing to ramp)");
    CHECK(req_noop(R(DRV_ENABLING, DRV_REQ_ENABLE), DRV_ENABLING), "ENABLING+ENABLE no-op");
    CHECK(req_rej (R(DRV_ENABLING, DRV_REQ_RESET),  DRV_ENABLING), "ENABLING+RESET rejected");

    /* ENABLED: DISABLE now; STOP ramps; ENABLE no-op; RESET rejected. */
    d = R(DRV_ENABLED, DRV_REQ_DISABLE);
    CHECK(d.next_state == DRV_DISABLED && d.send_disable && !d.begin_stop &&
          d.reset_cmd_seq, "ENABLED+DISABLE → Type-4 now");
    d = R(DRV_ENABLED, DRV_REQ_STOP);
    CHECK(d.next_state == DRV_STOPPING && d.begin_stop && !d.send_disable &&
          d.reset_cmd_seq, "ENABLED+STOP → STOPPING");
    CHECK(req_noop(R(DRV_ENABLED, DRV_REQ_ENABLE), DRV_ENABLED), "ENABLED+ENABLE no-op");
    CHECK(req_rej (R(DRV_ENABLED, DRV_REQ_RESET),  DRV_ENABLED), "ENABLED+RESET rejected");

    /* STOPPING: DISABLE cuts the ramp short — end state honours the fault latch. */
    d = drive_sm_on_request(DRV_STOPPING, DRV_REQ_DISABLE, 0u, 0u);
    CHECK(d.next_state == DRV_DISABLED && d.send_disable,
          "STOPPING+DISABLE, no fault → DISABLED");
    d = drive_sm_on_request(DRV_STOPPING, DRV_REQ_DISABLE, 0u, 1u);
    CHECK(d.next_state == DRV_FAULT && d.send_disable,
          "STOPPING+DISABLE, fault latched → FAULT (no laundering)");
    CHECK(req_noop(R(DRV_STOPPING, DRV_REQ_ENABLE), DRV_STOPPING), "STOPPING+ENABLE no-op");
    CHECK(req_noop(R(DRV_STOPPING, DRV_REQ_STOP),   DRV_STOPPING), "STOPPING+STOP no-op");
    CHECK(req_rej (R(DRV_STOPPING, DRV_REQ_RESET),  DRV_STOPPING), "STOPPING+RESET rejected");

    /* FAULT: latched; only RESET leaves, into CLEARING with the wire clear. */
    CHECK(req_noop(R(DRV_FAULT, DRV_REQ_DISABLE), DRV_FAULT), "FAULT+DISABLE stays latched");
    CHECK(req_noop(R(DRV_FAULT, DRV_REQ_STOP),    DRV_FAULT), "FAULT+STOP no-op");
    CHECK(req_rej (R(DRV_FAULT, DRV_REQ_ENABLE),  DRV_FAULT), "FAULT+ENABLE rejected");
    d = R(DRV_FAULT, DRV_REQ_RESET);
    CHECK(d.next_state == DRV_CLEARING && d.send_clear && d.clear_fault &&
          !d.rejected, "FAULT+RESET → CLEARING, clears fault code on entry");

    /* ── fault reactions ─────────────────────────────────────────────────── */

    /* Comms-intact from ENABLED: ramp. */
    f = drive_sm_on_fault(DRV_ENABLED, DRVF_OVERTORQUE, 0u);
    CHECK(f.next_state == DRV_STOPPING && f.begin_stop && !f.send_disable &&
          f.latch_fault == DRVF_OVERTORQUE, "OVERTORQUE in ENABLED → STOPPING");
    f = drive_sm_on_fault(DRV_ENABLED, DRVF_MASTER_LOST, 0u);
    CHECK(f.next_state == DRV_STOPPING && f.begin_stop &&
          f.latch_fault == DRVF_MASTER_LOST, "MASTER_LOST in ENABLED → STOPPING");

    /* Comms-intact but nothing to ramp (ENABLING): immediate. */
    f = drive_sm_on_fault(DRV_ENABLING, DRVF_MASTER_LOST, 0u);
    CHECK(f.next_state == DRV_FAULT && f.send_disable && f.abort_enable,
          "MASTER_LOST in ENABLING → immediate disable");

    /* Comms-broken: immediate from every armed state. */
    f = drive_sm_on_fault(DRV_ENABLED, DRVF_CAN_TIMEOUT, 0u);
    CHECK(f.next_state == DRV_FAULT && f.send_disable && !f.begin_stop,
          "CAN_TIMEOUT in ENABLED → immediate disable");
    f = drive_sm_on_fault(DRV_ENABLED, DRVF_NOT_ENABLED, 0u);
    CHECK(f.next_state == DRV_FAULT && f.send_disable, "NOT_ENABLED → immediate");
    f = drive_sm_on_fault(DRV_ENABLING, DRVF_CAN_TIMEOUT, 0u);
    CHECK(f.next_state == DRV_FAULT && f.send_disable && f.abort_enable,
          "CAN_TIMEOUT in ENABLING aborts the handshake");
    f = drive_sm_on_fault(DRV_DISABLED, DRVF_MOTOR_REPORTED, 0u);
    CHECK(f.next_state == DRV_FAULT && f.send_disable &&
          f.latch_fault == DRVF_MOTOR_REPORTED, "vendor bits in DISABLED → FAULT");

    /* Comms-broken during STOPPING abandons the ramp. */
    f = drive_sm_on_fault(DRV_STOPPING, DRVF_CAN_TIMEOUT, 1u);
    CHECK(f.next_state == DRV_FAULT && f.send_disable,
          "CAN_TIMEOUT mid-ramp abandons STOPPING");
    CHECK(f.latch_fault == DRVF_NONE, "first fault wins: mid-ramp latch suppressed");

    /* Comms-intact repeat during STOPPING: keep ramping. */
    f = drive_sm_on_fault(DRV_STOPPING, DRVF_OVERTORQUE, 1u);
    CHECK(f.next_state == DRV_STOPPING && !f.send_disable && !f.begin_stop,
          "comms-intact repeat during STOPPING changes nothing");

    /* First-fault-wins latch suppression. */
    f = drive_sm_on_fault(DRV_ENABLED, DRVF_CAN_TIMEOUT, 1u);
    CHECK(f.latch_fault == DRVF_NONE && f.next_state == DRV_FAULT,
          "second fault still disables but does not re-latch");

    /* Faults are ignored in non-live states. */
    f = drive_sm_on_fault(DRV_FAULT, DRVF_CAN_TIMEOUT, 1u);
    CHECK(f.next_state == DRV_FAULT && !f.send_disable, "fault in FAULT: no action");
    f = drive_sm_on_fault(DRV_ABSENT, DRVF_CAN_TIMEOUT, 0u);
    CHECK(f.next_state == DRV_ABSENT && !f.send_disable, "fault in ABSENT: no action");

    /* ── ticks ───────────────────────────────────────────────────────────── */
    memset(&in, 0, sizeof(in));

    /* ENABLING: run seen → ENABLED (edge flagged); timeout → ENABLE_FAILED. */
    in.run_seen = 1u;
    t = drive_sm_on_tick(DRV_ENABLING, &in);
    CHECK(t.next_state == DRV_ENABLED && t.enter_enabled, "Run seen → ENABLED edge");
    memset(&in, 0, sizeof(in));
    in.enable_timeout = 1u;
    t = drive_sm_on_tick(DRV_ENABLING, &in);
    CHECK(t.next_state == DRV_FAULT && t.send_disable &&
          t.latch_fault == DRVF_ENABLE_FAILED, "enable timeout → ENABLE_FAILED");
    in.fault_latched = 1u;
    t = drive_sm_on_tick(DRV_ENABLING, &in);
    CHECK(t.latch_fault == DRVF_NONE && t.next_state == DRV_FAULT,
          "enable timeout with earlier latch keeps the first fault");
    memset(&in, 0, sizeof(in));
    t = drive_sm_on_tick(DRV_ENABLING, &in);
    CHECK(t.next_state == DRV_ENABLING, "ENABLING holds while waiting");

    /* CLEARING: clean / dirty / silent. */
    in.clear_reply = 1u;
    t = drive_sm_on_tick(DRV_CLEARING, &in);
    CHECK(t.next_state == DRV_DISABLED && t.latch_fault == DRVF_NONE,
          "clean reply → DISABLED");
    in.clear_dirty = 1u;
    t = drive_sm_on_tick(DRV_CLEARING, &in);
    CHECK(t.next_state == DRV_FAULT && t.latch_fault == DRVF_MOTOR_REPORTED,
          "dirty reply → FAULT(MOTOR_REPORTED) re-latched");
    memset(&in, 0, sizeof(in));
    in.clear_timeout = 1u;
    t = drive_sm_on_tick(DRV_CLEARING, &in);
    CHECK(t.next_state == DRV_ABSENT, "no reply → ABSENT");

    /* STOPPING completion honours the latch. */
    memset(&in, 0, sizeof(in));
    in.stop_done = 1u;
    t = drive_sm_on_tick(DRV_STOPPING, &in);
    CHECK(t.next_state == DRV_DISABLED, "requested stop completes → DISABLED");
    in.fault_latched = 1u;
    t = drive_sm_on_tick(DRV_STOPPING, &in);
    CHECK(t.next_state == DRV_FAULT, "fault-triggered stop completes → FAULT");

    /* Presence: DISABLED ⇄ ABSENT. */
    memset(&in, 0, sizeof(in));
    in.fb_timeout = 1u;
    t = drive_sm_on_tick(DRV_DISABLED, &in);
    CHECK(t.next_state == DRV_ABSENT, "feedback timeout → ABSENT");
    memset(&in, 0, sizeof(in));
    in.fb_fresh = 1u;
    t = drive_sm_on_tick(DRV_ABSENT, &in);
    CHECK(t.next_state == DRV_DISABLED, "reply while ABSENT → DISABLED");
    memset(&in, 0, sizeof(in));
    t = drive_sm_on_tick(DRV_ENABLED, &in);
    CHECK(t.next_state == DRV_ENABLED, "ENABLED has no tick transitions");

    if (failures) { fprintf(stderr, "%d failure(s)\n", failures); return 1; }
    printf("OK drive_sm\n");
    return 0;
}
