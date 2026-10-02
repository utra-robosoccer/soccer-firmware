/* Pure (HAL-free) tests for the mode-request state machine (mode_sm.c).
 *
 * Covers allowed/rejected transitions, HOLD-arm vs re-hold capture, the wound
 * refusal, TO_ZERO entry, fault latching, and both fault-clear paths. Exit 0 = OK.
 */
#include "mode_sm.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

#define V 1u   /* valid */

static ModeDecision step(uint8_t state, uint8_t req)
{
    return mode_sm_step(state, req, V, 0u, 0u);
}

static void test_arm_from_idle(void)
{
    ModeDecision d = step(LIFE_IDLE, REQ_HOLD);
    CHECK(d.do_arm && d.next_state == LIFE_HOLD && !d.rejected, "IDLE+HOLD should arm");

    /* Wound shaft → refuse arm, latch CAUSE_WOUND. */
    d = mode_sm_step(LIFE_IDLE, REQ_HOLD, V, 0u, /*wound=*/1u);
    CHECK(!d.do_arm && d.next_state == LIFE_FAULT && d.set_cause_wound && d.rejected,
          "IDLE+HOLD wound should fault");

    /* Armed requests other than HOLD are rejected from IDLE. */
    for (uint8_t req = REQ_MIT; req <= REQ_TO_ZERO; req++) {
        d = step(LIFE_IDLE, req);
        CHECK(d.rejected && d.next_state == LIFE_IDLE, "IDLE+armed(non-HOLD) rejected");
    }
    d = step(LIFE_IDLE, REQ_IDLE);
    CHECK(!d.rejected && !d.do_disable && d.next_state == LIFE_IDLE, "IDLE+IDLE no-op");
}

static void test_armed_transitions(void)
{
    const uint8_t armed[] = { LIFE_HOLD, LIFE_MIT, LIFE_DAMPED, LIFE_TO_ZERO };
    for (unsigned i = 0; i < sizeof(armed); i++) {
        uint8_t s = armed[i];
        CHECK(step(s, REQ_MIT).next_state == LIFE_MIT, "armed→MIT");
        CHECK(step(s, REQ_DAMPED).next_state == LIFE_DAMPED, "armed→DAMPED");

        ModeDecision z = step(s, REQ_TO_ZERO);
        CHECK(z.next_state == LIFE_TO_ZERO, "armed→TO_ZERO");
        /* enter_to_zero only when ENTERING (not already TO_ZERO). */
        CHECK(z.enter_to_zero == (s != LIFE_TO_ZERO ? 1u : 0u), "enter_to_zero on entry only");

        ModeDecision h = step(s, REQ_HOLD);
        CHECK(h.next_state == LIFE_HOLD && !h.do_arm, "armed→HOLD no re-enable");
        CHECK(h.capture_hold == 1u, "armed→HOLD captures position");

        ModeDecision idle = step(s, REQ_IDLE);
        CHECK(idle.next_state == LIFE_IDLE && idle.do_disable, "armed→IDLE disables");
    }
    /* MIT→MIT does not re-capture (no HOLD capture flag). */
    CHECK(step(LIFE_MIT, REQ_MIT).capture_hold == 0u, "MIT→MIT no capture");
}

static void test_fault_latch_and_reset(void)
{
    /* FAULT latches: armed requests rejected, state unchanged. */
    CHECK(step(LIFE_FAULT, REQ_HOLD).rejected &&
          step(LIFE_FAULT, REQ_HOLD).next_state == LIFE_FAULT, "FAULT+HOLD rejected/latched");
    CHECK(step(LIFE_FAULT, REQ_MIT).rejected, "FAULT+MIT rejected");

    /* Reset path 1: an IDLE request clears the latch → IDLE. */
    ModeDecision d = step(LIFE_FAULT, REQ_IDLE);
    CHECK(d.next_state == LIFE_IDLE && d.do_disable, "FAULT+IDLE clears to IDLE");

    /* Reset path 2: the fault_reset flag clears, then the request is evaluated
       from IDLE in the SAME cycle. */
    d = mode_sm_step(LIFE_FAULT, REQ_HOLD, V, /*fault_reset=*/1u, /*wound=*/0u);
    CHECK(d.clear_fault && d.do_arm && d.next_state == LIFE_HOLD,
          "fault_reset + HOLD clears then arms");
    d = mode_sm_step(LIFE_FAULT, REQ_IDLE, V, 1u, 0u);
    CHECK(d.clear_fault && d.next_state == LIFE_IDLE, "fault_reset + IDLE clears to IDLE");
    /* fault_reset + HOLD but wound → clears fault, then refuses arm (re-faults). */
    d = mode_sm_step(LIFE_FAULT, REQ_HOLD, V, 1u, 1u);
    CHECK(d.clear_fault && d.set_cause_wound && d.next_state == LIFE_FAULT,
          "fault_reset + HOLD wound re-faults");
}

static void test_invalid_slot(void)
{
    /* valid=0 → no-op: keep state, emit no actions. */
    ModeDecision d = mode_sm_step(LIFE_MIT, REQ_IDLE, 0u, 0u, 0u);
    CHECK(d.next_state == LIFE_MIT && !d.do_disable && !d.rejected && !d.do_arm,
          "invalid slot is a no-op");
}

int main(void)
{
    test_arm_from_idle();
    test_armed_transitions();
    test_fault_latch_and_reset();
    test_invalid_slot();
    if (failures == 0) { printf("mode_sm tests: OK\n"); return 0; }
    fprintf(stderr, "mode_sm tests: %d FAILURE(S)\n", failures);
    return 1;
}
