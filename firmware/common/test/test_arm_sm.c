/* Pure (HAL-free) tests for the async-arm phase machine (arm_sm.h). Exit 0 = OK. */
#include "arm_sm.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

#define CLEAR_W  1u   /* ~5 ms  @ 200 Hz */
#define SETTLE_W 4u   /* ~20 ms @ 200 Hz */

static uint8_t step(uint8_t *ph, uint8_t *w) { return arm_sm_step(ph, w, CLEAR_W, SETTLE_W); }

/* Step until the next non-NONE action, counting cycles; returns the action. */
static uint8_t run_to_action(uint8_t *ph, uint8_t *w, unsigned *cycles)
{
    for (unsigned i = 0; i < 64u; i++) {
        uint8_t a = step(ph, w);
        (*cycles)++;
        if (a != ARM_ACT_NONE) return a;
    }
    return 0xFFu;   /* never fired — the CHECK on the action will fail loudly */
}

int main(void)
{
    uint8_t ph = ARM_PHASE_IDLE, w = 0u;
    unsigned c;

    /* Idle machine does nothing, forever. */
    CHECK(!arm_sm_active(ph), "idle: inactive");
    CHECK(step(&ph, &w) == ARM_ACT_NONE && ph == ARM_PHASE_IDLE, "idle: step is a no-op");

    /* No latched fault: MODE → ENABLE → COMPLETE with the settle waits between. */
    arm_sm_begin(&ph, &w, 0u);
    CHECK(arm_sm_active(ph) && ph == ARM_PHASE_MODE && w == 0u, "begin(no fault) → MODE, no wait");
    CHECK(step(&ph, &w) == ARM_ACT_SEND_MODE, "first step sends MODE immediately");
    c = 0u; CHECK(run_to_action(&ph, &w, &c) == ARM_ACT_SEND_ENABLE, "then ENABLE");
    CHECK(c == SETTLE_W + 1u, "ENABLE after exactly settle_wait cycles");
    c = 0u; CHECK(run_to_action(&ph, &w, &c) == ARM_ACT_COMPLETE, "then COMPLETE");
    CHECK(c == SETTLE_W + 1u, "COMPLETE after exactly settle_wait cycles");
    CHECK(!arm_sm_active(ph), "complete: machine is idle again");
    CHECK(step(&ph, &w) == ARM_ACT_NONE, "complete: further steps are no-ops");

    /* Latched fault: CLEAR phase first, with its own (shorter) wait. */
    arm_sm_begin(&ph, &w, 1u);
    CHECK(ph == ARM_PHASE_CLEAR, "begin(fault) → CLEAR first");
    CHECK(step(&ph, &w) == ARM_ACT_SEND_CLEAR, "first step sends CLEAR");
    c = 0u; CHECK(run_to_action(&ph, &w, &c) == ARM_ACT_SEND_MODE, "then MODE");
    CHECK(c == CLEAR_W + 1u, "MODE after exactly clear_wait cycles");

    /* Cancel mid-handshake: machine goes idle, nothing further fires. */
    arm_sm_cancel(&ph, &w);
    CHECK(!arm_sm_active(ph) && w == 0u, "cancel: idle, wait cleared");
    CHECK(step(&ph, &w) == ARM_ACT_NONE, "cancel: step is a no-op");

    /* Re-begin after cancel works (fresh handshake). */
    arm_sm_begin(&ph, &w, 0u);
    CHECK(step(&ph, &w) == ARM_ACT_SEND_MODE, "re-begin after cancel sends MODE");

    /* Zero waits degenerate to one action per cycle, in order. */
    ph = ARM_PHASE_IDLE; w = 0u;
    arm_sm_begin(&ph, &w, 1u);
    CHECK(arm_sm_step(&ph, &w, 0u, 0u) == ARM_ACT_SEND_CLEAR,  "zero-wait: CLEAR");
    CHECK(arm_sm_step(&ph, &w, 0u, 0u) == ARM_ACT_SEND_MODE,   "zero-wait: MODE");
    CHECK(arm_sm_step(&ph, &w, 0u, 0u) == ARM_ACT_SEND_ENABLE, "zero-wait: ENABLE");
    CHECK(arm_sm_step(&ph, &w, 0u, 0u) == ARM_ACT_COMPLETE,    "zero-wait: COMPLETE");
    CHECK(arm_sm_step(&ph, &w, 0u, 0u) == ARM_ACT_NONE,        "zero-wait: then idle");

    if (failures == 0) { printf("OK arm_sm\n"); return 0; }
    fprintf(stderr, "arm_sm: %d FAILURE(S)\n", failures);
    return 1;
}
