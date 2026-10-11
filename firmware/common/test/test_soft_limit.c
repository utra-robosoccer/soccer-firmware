/* Pure (HAL-free) tests for the one-sided soft-limit clamp (soft_limit.h). Exit 0 = OK. */
#include "soft_limit.h"
#include <stdio.h>
#include <math.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)
#define EQ(a, b) (fabsf((a) - (b)) < 1e-6f)

#define LO (-0.79f)
#define HI ( 0.79f)

int main(void)
{
    float p;
    /* Inside the range, actual inside: ordinary clamp. */
    p = 0.5f;  CHECK(!soft_clamp_pos(&p, 0.4f, LO, HI) && EQ(p, 0.5f), "in-range unclamped");
    p = 0.9f;  CHECK( soft_clamp_pos(&p, 0.4f, LO, HI) && EQ(p, HI),   "over hi (shaft in) → hi");
    p = -0.9f; CHECK( soft_clamp_pos(&p, 0.0f, LO, HI) && EQ(p, LO),   "under lo (shaft in) → lo");

    /* Shaft parked OUTSIDE (0.89 > hi 0.79): hold where it is, no clamp, no error. */
    p = 0.89f; CHECK(!soft_clamp_pos(&p, 0.89f, LO, HI) && EQ(p, 0.89f),
                     "arm at 0.89 holds at 0.89 (no clamp → no overtorque)");
    /* ...may move BACK toward the range. */
    p = 0.5f;  CHECK(!soft_clamp_pos(&p, 0.89f, LO, HI) && EQ(p, 0.5f), "return toward range allowed");
    p = HI;    CHECK(!soft_clamp_pos(&p, 0.89f, LO, HI) && EQ(p, HI),   "to the limit allowed");
    /* ...but NOT further out than it already is. */
    p = 0.95f; CHECK( soft_clamp_pos(&p, 0.89f, LO, HI) && EQ(p, 0.89f), "further out clamped to actual");

    /* Symmetric on the low side: further out than the shaft → clamped back to the shaft. */
    p = -0.95f; CHECK( soft_clamp_pos(&p, -0.90f, LO, HI) && EQ(p, -0.90f),
                      "further out (low) clamped to actual");
    p = -0.90f; CHECK(!soft_clamp_pos(&p, -0.90f, LO, HI) && EQ(p, -0.90f),
                      "hold at shaft (low, outside) unclamped");

    /* ── soft_clamp_mit: full MIT-tuple clamp (pos + outward ff cancel + |tau_ff| bound) ── */
    {
        float v, t;
        uint8_t c;
        #define TAU_LIM 0.8f

        /* In range, feedforward within bound: everything passes through untouched. */
        p = 0.5f; v = 1.0f; t = 0.3f;
        c = soft_clamp_mit(&p, &v, &t, 0.4f, LO, HI, TAU_LIM);
        CHECK(c == 0u && EQ(p, 0.5f) && EQ(v, 1.0f) && EQ(t, 0.3f), "mit: in-range passthrough");

        /* |tau_ff| bound alone (no position clamp): clamped to ±TAU_LIM, FF bit set. */
        p = 0.0f; v = 0.0f; t = 2.0f;
        c = soft_clamp_mit(&p, &v, &t, 0.0f, LO, HI, TAU_LIM);
        CHECK(c == SOFT_CLAMPED_FF && EQ(t, TAU_LIM), "mit: tau_ff bound high");
        p = 0.0f; v = 0.0f; t = -2.0f;
        c = soft_clamp_mit(&p, &v, &t, 0.0f, LO, HI, TAU_LIM);
        CHECK(c == SOFT_CLAMPED_FF && EQ(t, -TAU_LIM), "mit: tau_ff bound low");

        /* Position clamped from above: outward (positive) vel AND tau_ff cancelled. */
        p = 0.9f; v = 1.0f; t = 0.3f;
        c = soft_clamp_mit(&p, &v, &t, 0.4f, LO, HI, TAU_LIM);
        CHECK(c == (SOFT_CLAMPED_POS | SOFT_CLAMPED_FF) && EQ(p, HI) && EQ(v, 0.0f) && EQ(t, 0.0f),
              "mit: over hi cancels outward ff");

        /* Position clamped from above, but INWARD (negative) feedforward is kept. */
        p = 0.9f; v = -1.0f; t = -0.3f;
        c = soft_clamp_mit(&p, &v, &t, 0.4f, LO, HI, TAU_LIM);
        CHECK(c == SOFT_CLAMPED_POS && EQ(p, HI) && EQ(v, -1.0f) && EQ(t, -0.3f),
              "mit: over hi keeps inward ff");

        /* Position clamped from below: outward (negative) vel and tau_ff cancelled. */
        p = -0.9f; v = -1.0f; t = -0.3f;
        c = soft_clamp_mit(&p, &v, &t, 0.0f, LO, HI, TAU_LIM);
        CHECK(c == (SOFT_CLAMPED_POS | SOFT_CLAMPED_FF) && EQ(p, LO) && EQ(v, 0.0f) && EQ(t, 0.0f),
              "mit: under lo cancels outward ff");

        /* Shaft parked outside: holding at the shaft is unclamped; inward tau_ff kept. */
        p = 0.89f; v = 0.0f; t = -0.2f;
        c = soft_clamp_mit(&p, &v, &t, 0.89f, LO, HI, TAU_LIM);
        CHECK(c == 0u && EQ(p, 0.89f) && EQ(t, -0.2f), "mit: parked outside, inward ff kept");

        /* Shaft parked outside, commanded further out with outward tau_ff: both clamped. */
        p = 0.95f; v = 0.0f; t = 0.2f;
        c = soft_clamp_mit(&p, &v, &t, 0.89f, LO, HI, TAU_LIM);
        CHECK(c == (SOFT_CLAMPED_POS | SOFT_CLAMPED_FF) && EQ(p, 0.89f) && EQ(t, 0.0f),
              "mit: parked outside, outward ff cancelled");

        /* Outward cancel and magnitude bound compose: inward tau over the bound while
           position is clamped → kept inward but bounded. */
        p = 0.9f; v = 0.0f; t = -2.0f;
        c = soft_clamp_mit(&p, &v, &t, 0.4f, LO, HI, TAU_LIM);
        CHECK(c == (SOFT_CLAMPED_POS | SOFT_CLAMPED_FF) && EQ(p, HI) && EQ(t, -TAU_LIM),
              "mit: inward tau bounded while pos clamped");
        #undef TAU_LIM
    }

    if (failures == 0) { printf("OK soft_limit\n"); return 0; }
    fprintf(stderr, "soft_limit: %d FAILURE(S)\n", failures);
    return 1;
}
