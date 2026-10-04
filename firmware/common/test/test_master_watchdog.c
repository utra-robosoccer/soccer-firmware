/* Pure (HAL-free) tests for the slave master-loss ramp (master_watchdog.h). Exit 0 = OK. */
#include "master_watchdog.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

#define GRACE 50u
#define DAMP  300u

int main(void)
{
    /* Hold-grace window. */
    CHECK(master_loss_phase(0u,   GRACE, DAMP) == MASTER_HOLD_GRACE, "t=0 → HOLD_GRACE");
    CHECK(master_loss_phase(49u,  GRACE, DAMP) == MASTER_HOLD_GRACE, "just under grace → HOLD_GRACE");
    /* Grace boundary → DAMPED. */
    CHECK(master_loss_phase(50u,  GRACE, DAMP) == MASTER_DAMPED, "at grace → DAMPED");
    CHECK(master_loss_phase(349u, GRACE, DAMP) == MASTER_DAMPED, "just under grace+damp → DAMPED");
    /* Damp boundary → IDLE. */
    CHECK(master_loss_phase(350u, GRACE, DAMP) == MASTER_IDLE, "at grace+damp → IDLE");
    CHECK(master_loss_phase(5000u, GRACE, DAMP) == MASTER_IDLE, "well past → IDLE");

    if (failures == 0) { printf("OK master_watchdog\n"); return 0; }
    fprintf(stderr, "master_watchdog: %d FAILURE(S)\n", failures);
    return 1;
}
