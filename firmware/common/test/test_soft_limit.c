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

    if (failures == 0) { printf("OK soft_limit\n"); return 0; }
    fprintf(stderr, "soft_limit: %d FAILURE(S)\n", failures);
    return 1;
}
