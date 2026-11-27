/* Pure (HAL-free) tests for the drive state machine (drive_sm.h) over the
 * protocol_wire.h MotorState/MotorReq/MotorFault enums. Covers every cell of
 * the request × state matrix, every fault-reaction row (EMERGENCY vs the one
 * CONTROLLED fault, MASTER_LOST) including the first-fault-wins latch rule,
 * and the transient-state tick table (ENABLING / STOPPING / DISABLED→ABSENT,
 * with ABSENT terminal). Exit 0 = OK. */
#include "drive_sm.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

/* A decision with no actions and no state change. */
static uint8_t req_noop(DriveReqDecision d, uint8_t state)
{
    return (uint8_t)(d.next_state == state && !d.rejected && !d.send_stop &&
                     !d.begin_enable && !d.begin_stop && !d.abort_enable &&
                     !d.reset_cmd_seq);
}

static uint8_t req_rej(DriveReqDecision d, uint8_t state)
{
    return (uint8_t)(d.next_state == state && d.rejected && !d.send_stop &&
                     !d.begin_enable && !d.begin_stop && !d.abort_enable);
}

static DriveReqDecision R(uint8_t st, uint8_t rq)        /* plain request */
{
    return drive_sm_on_request(st, rq, 0u);
}

int main(void)
{
    DriveReqDecision d;
    DriveFaultDecision f;
    DriveTickDecision t;
    DriveTickInputs in;

    /* ── request × state matrix ──────────────────────────────────────────── */

    /* ABSENT: terminal — STOP no-op, ENABLE rejected. */
    CHECK(req_noop(R(MOTOR_ABSENT, MOTOR_REQ_STOP),   MOTOR_ABSENT), "ABSENT+STOP no-op");
    CHECK(req_rej (R(MOTOR_ABSENT, MOTOR_REQ_ENABLE), MOTOR_ABSENT), "ABSENT+ENABLE rejected");

    /* DISABLED: STOP is the resting request; ENABLE begins the handshake
       (readiness gating is the caller's verified-enable gate). */
    CHECK(req_noop(R(MOTOR_DISABLED, MOTOR_REQ_STOP), MOTOR_DISABLED), "DISABLED+STOP no-op");
    d = R(MOTOR_DISABLED, MOTOR_REQ_ENABLE);
    CHECK(d.next_state == MOTOR_ENABLING && d.begin_enable && d.reset_cmd_seq &&
          !d.rejected && !d.send_stop, "DISABLED+ENABLE → ENABLING");

    /* ENABLING: STOP aborts the handshake (nothing to ramp yet); ENABLE no-op. */
    d = R(MOTOR_ENABLING, MOTOR_REQ_STOP);
    CHECK(d.next_state == MOTOR_DISABLED && d.abort_enable && d.send_stop &&
          d.reset_cmd_seq, "ENABLING+STOP aborts → DISABLED");
    d = drive_sm_on_request(MOTOR_ENABLING, MOTOR_REQ_STOP, 1u);
    CHECK(d.next_state == MOTOR_FAULT && d.abort_enable && d.send_stop,
          "ENABLING+STOP with latch → FAULT (no laundering)");
    CHECK(req_noop(R(MOTOR_ENABLING, MOTOR_REQ_ENABLE), MOTOR_ENABLING),
          "ENABLING+ENABLE no-op");

    /* ENABLED: STOP ramps; ENABLE no-op. */
    d = R(MOTOR_ENABLED, MOTOR_REQ_STOP);
    CHECK(d.next_state == MOTOR_STOPPING && d.begin_stop && !d.send_stop &&
          d.reset_cmd_seq, "ENABLED+STOP → STOPPING");
    CHECK(req_noop(R(MOTOR_ENABLED, MOTOR_REQ_ENABLE), MOTOR_ENABLED),
          "ENABLED+ENABLE no-op");

    /* STOPPING: both requests inert while the ramp runs. */
    CHECK(req_noop(R(MOTOR_STOPPING, MOTOR_REQ_STOP),   MOTOR_STOPPING), "STOPPING+STOP no-op");
    CHECK(req_rej (R(MOTOR_STOPPING, MOTOR_REQ_ENABLE), MOTOR_STOPPING), "STOPPING+ENABLE rejected");

    /* FAULT: latched — STOP no-op, ENABLE rejected. Only SLAVE_REQ_RESET
       (outside this machine) leaves FAULT. */
    CHECK(req_noop(R(MOTOR_FAULT, MOTOR_REQ_STOP),   MOTOR_FAULT), "FAULT+STOP stays latched");
    CHECK(req_rej (R(MOTOR_FAULT, MOTOR_REQ_ENABLE), MOTOR_FAULT), "FAULT+ENABLE rejected");

    /* Unknown request value: rejected everywhere. */
    CHECK(req_rej(R(MOTOR_ENABLED, 0x7Fu), MOTOR_ENABLED), "unknown req rejected");

    /* ── fault classification ────────────────────────────────────────────── */
    CHECK(!drive_sm_fault_immediate(MOTOR_FAULT_NONE), "NONE not immediate");
    CHECK(!drive_sm_fault_immediate(MOTOR_FAULT_MASTER_LOST), "MASTER_LOST controlled");
    CHECK(drive_sm_fault_immediate(MOTOR_FAULT_FEEDBACK_TIMEOUT) &&
          drive_sm_fault_immediate(MOTOR_FAULT_ENABLE_FAILED) &&
          drive_sm_fault_immediate(MOTOR_FAULT_LOST_ENABLE) &&
          drive_sm_fault_immediate(MOTOR_FAULT_OVERTORQUE) &&
          drive_sm_fault_immediate(MOTOR_FAULT_RS_REPORTED),
          "all other faults are EMERGENCY");

    /* ── fault reactions ─────────────────────────────────────────────────── */

    /* CONTROLLED (MASTER_LOST) from ENABLED: damp through STOPPING. */
    f = drive_sm_on_fault(MOTOR_ENABLED, MOTOR_FAULT_MASTER_LOST, 0u);
    CHECK(f.next_state == MOTOR_STOPPING && f.begin_stop && !f.send_stop &&
          f.latch_fault == MOTOR_FAULT_MASTER_LOST, "MASTER_LOST in ENABLED → STOPPING");

    /* CONTROLLED fault but nothing to ramp (ENABLING): immediate. */
    f = drive_sm_on_fault(MOTOR_ENABLING, MOTOR_FAULT_MASTER_LOST, 0u);
    CHECK(f.next_state == MOTOR_FAULT && f.send_stop && f.abort_enable,
          "MASTER_LOST in ENABLING → immediate stop");

    /* OVERTORQUE is an immediate stop: cut effort NOW, no damping through it. */
    f = drive_sm_on_fault(MOTOR_ENABLED, MOTOR_FAULT_OVERTORQUE, 0u);
    CHECK(f.next_state == MOTOR_FAULT && f.send_stop && !f.begin_stop &&
          f.latch_fault == MOTOR_FAULT_OVERTORQUE, "OVERTORQUE in ENABLED → immediate FAULT");

    /* Comms-broken: immediate from every armed state. */
    f = drive_sm_on_fault(MOTOR_ENABLED, MOTOR_FAULT_FEEDBACK_TIMEOUT, 0u);
    CHECK(f.next_state == MOTOR_FAULT && f.send_stop && !f.begin_stop,
          "FEEDBACK_TIMEOUT in ENABLED → immediate stop");
    f = drive_sm_on_fault(MOTOR_ENABLED, MOTOR_FAULT_LOST_ENABLE, 0u);
    CHECK(f.next_state == MOTOR_FAULT && f.send_stop, "LOST_ENABLE → immediate");
    f = drive_sm_on_fault(MOTOR_ENABLING, MOTOR_FAULT_FEEDBACK_TIMEOUT, 0u);
    CHECK(f.next_state == MOTOR_FAULT && f.send_stop && f.abort_enable,
          "FEEDBACK_TIMEOUT in ENABLING aborts the handshake");
    f = drive_sm_on_fault(MOTOR_DISABLED, MOTOR_FAULT_RS_REPORTED, 0u);
    CHECK(f.next_state == MOTOR_FAULT && f.send_stop &&
          f.latch_fault == MOTOR_FAULT_RS_REPORTED, "vendor bits in DISABLED → FAULT");

    /* Comms-broken during STOPPING abandons the ramp. */
    f = drive_sm_on_fault(MOTOR_STOPPING, MOTOR_FAULT_FEEDBACK_TIMEOUT, 1u);
    CHECK(f.next_state == MOTOR_FAULT && f.send_stop,
          "FEEDBACK_TIMEOUT mid-ramp abandons STOPPING");
    CHECK(f.latch_fault == MOTOR_FAULT_NONE, "first fault wins: mid-ramp latch suppressed");

    /* CONTROLLED-fault repeat during STOPPING: keep ramping. */
    f = drive_sm_on_fault(MOTOR_STOPPING, MOTOR_FAULT_MASTER_LOST, 1u);
    CHECK(f.next_state == MOTOR_STOPPING && !f.send_stop && !f.begin_stop,
          "MASTER_LOST repeat during STOPPING changes nothing");

    /* OVERTORQUE mid-ramp (e.g. the damp itself overloads): abandon the ramp. */
    f = drive_sm_on_fault(MOTOR_STOPPING, MOTOR_FAULT_OVERTORQUE, 1u);
    CHECK(f.next_state == MOTOR_FAULT && f.send_stop,
          "OVERTORQUE mid-ramp abandons STOPPING");

    /* First-fault-wins latch suppression. */
    f = drive_sm_on_fault(MOTOR_ENABLED, MOTOR_FAULT_FEEDBACK_TIMEOUT, 1u);
    CHECK(f.latch_fault == MOTOR_FAULT_NONE && f.next_state == MOTOR_FAULT,
          "second fault still stops but does not re-latch");

    /* Faults are ignored in non-live states. */
    f = drive_sm_on_fault(MOTOR_FAULT, MOTOR_FAULT_FEEDBACK_TIMEOUT, 1u);
    CHECK(f.next_state == MOTOR_FAULT && !f.send_stop, "fault in FAULT: no action");
    f = drive_sm_on_fault(MOTOR_ABSENT, MOTOR_FAULT_FEEDBACK_TIMEOUT, 0u);
    CHECK(f.next_state == MOTOR_ABSENT && !f.send_stop, "fault in ABSENT: no action");

    /* ── ticks ───────────────────────────────────────────────────────────── */
    memset(&in, 0, sizeof(in));

    /* ENABLING: Run seen → ENABLED (edge flagged); timeout → ENABLE_FAILED. */
    in.run_seen = 1u;
    t = drive_sm_on_tick(MOTOR_ENABLING, &in);
    CHECK(t.next_state == MOTOR_ENABLED && t.enter_enabled, "Run seen → ENABLED edge");
    memset(&in, 0, sizeof(in));
    in.enable_timeout = 1u;
    t = drive_sm_on_tick(MOTOR_ENABLING, &in);
    CHECK(t.next_state == MOTOR_FAULT && t.send_stop &&
          t.latch_fault == MOTOR_FAULT_ENABLE_FAILED, "enable timeout → ENABLE_FAILED");
    in.fault_latched = 1u;
    t = drive_sm_on_tick(MOTOR_ENABLING, &in);
    CHECK(t.latch_fault == MOTOR_FAULT_NONE && t.next_state == MOTOR_FAULT,
          "enable timeout with earlier latch keeps the first fault");
    memset(&in, 0, sizeof(in));
    t = drive_sm_on_tick(MOTOR_ENABLING, &in);
    CHECK(t.next_state == MOTOR_ENABLING, "ENABLING holds while waiting");

    /* STOPPING completion honours the latch. */
    memset(&in, 0, sizeof(in));
    in.stop_done = 1u;
    t = drive_sm_on_tick(MOTOR_STOPPING, &in);
    CHECK(t.next_state == MOTOR_DISABLED, "requested stop completes → DISABLED");
    in.fault_latched = 1u;
    t = drive_sm_on_tick(MOTOR_STOPPING, &in);
    CHECK(t.next_state == MOTOR_FAULT, "fault-triggered stop completes → FAULT");

    /* Presence: DISABLED → ABSENT on silence; ABSENT is TERMINAL (no tick,
       request, or fault brings it back — only SLAVE_REQ_RESET re-sweeps). */
    memset(&in, 0, sizeof(in));
    in.fb_timeout = 1u;
    t = drive_sm_on_tick(MOTOR_DISABLED, &in);
    CHECK(t.next_state == MOTOR_ABSENT, "feedback timeout → ABSENT");
    memset(&in, 0, sizeof(in));
    t = drive_sm_on_tick(MOTOR_ABSENT, &in);
    CHECK(t.next_state == MOTOR_ABSENT, "ABSENT is terminal on tick");
    t = drive_sm_on_tick(MOTOR_ENABLED, &in);
    CHECK(t.next_state == MOTOR_ENABLED, "ENABLED has no tick transitions");

    if (failures) { fprintf(stderr, "%d failure(s)\n", failures); return 1; }
    printf("OK drive_sm\n");
    return 0;
}
