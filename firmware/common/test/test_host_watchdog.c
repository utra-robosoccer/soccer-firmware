/* Pure (HAL-free) tests for the master host-death watchdog (host_watchdog.h).
 * Exit 0 = OK. */
#include "host_watchdog.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

#define LOST 12u    /* host-death trigger (cycles) */
#define DAMP 60u    /* DAMPED dwell before IDLE (cycles) */

static void test_no_trip_when_unarmed(void)
{
    HostWatchdog w; host_watchdog_reset(&w);
    for (uint32_t c = 0; c < 100u; c++) {
        CHECK(host_watchdog_step(&w, c, /*armed=*/0u, 0u, LOST, DAMP) == HOST_LINK_OK,
              "unarmed never trips");
    }
}

static void test_no_trip_under_decimation_gap(void)
{
    /* A 50 Hz host on a 200 Hz master → fresh command every 4 cycles. cycles_since_fresh
       cycles 0,1,2,3,0,… never reaches LOST, so an armed link stays OK. */
    HostWatchdog w; host_watchdog_reset(&w);
    for (uint32_t i = 0; i < 200u; i++) {
        uint32_t since = i % 4u;
        CHECK(host_watchdog_step(&w, since, /*armed=*/1u, 0u, LOST, DAMP) == HOST_LINK_OK,
              "normal decimation gap must not trip");
    }
}

static void test_trip_sequence_and_latch(void)
{
    HostWatchdog w; host_watchdog_reset(&w);
    /* Stay OK until the gap reaches LOST. */
    for (uint32_t since = 0; since < LOST; since++)
        CHECK(host_watchdog_step(&w, since, 1u, 0u, LOST, DAMP) == HOST_LINK_OK, "pre-trip OK");
    /* At LOST → DAMPED. */
    CHECK(host_watchdog_step(&w, LOST, 1u, 0u, LOST, DAMP) == HOST_LINK_DAMPED, "trips to DAMPED");
    /* DAMPED dwell (DAMP cycles counted after the trip) then IDLE. */
    HostLinkAction a = HOST_LINK_DAMPED;
    for (uint32_t k = 1; k <= DAMP; k++)
        a = host_watchdog_step(&w, LOST + k, 1u, 0u, LOST, DAMP);
    CHECK(a == HOST_LINK_IDLE, "DAMPED dwell then IDLE");
    /* The step just before IDLE was still DAMPED (dwell is DAMP cycles, not DAMP-1). */
    /* Latched: fresh commands (since=0) without fault_reset stay IDLE. */
    for (uint32_t k = 0; k < 20u; k++)
        CHECK(host_watchdog_step(&w, 0u, 1u, /*fault_reset=*/0u, LOST, DAMP) == HOST_LINK_IDLE,
              "latched until fault_reset");
}

static void test_recovery_via_fault_reset(void)
{
    HostWatchdog w; host_watchdog_reset(&w);
    /* Drive to IDLE. */
    host_watchdog_step(&w, LOST, 1u, 0u, LOST, DAMP);
    for (uint32_t k = 1; k <= DAMP; k++) host_watchdog_step(&w, LOST + k, 1u, 0u, LOST, DAMP);
    CHECK(w.state == HOST_LINK_IDLE, "reached IDLE");
    /* A fresh command WITH fault_reset recovers to OK. */
    CHECK(host_watchdog_step(&w, 0u, 1u, /*fault_reset=*/1u, LOST, DAMP) == HOST_LINK_OK,
          "fault_reset recovers");
    /* And it keeps running normally after. */
    CHECK(host_watchdog_step(&w, 1u, 1u, 0u, LOST, DAMP) == HOST_LINK_OK, "runs after recovery");
}

static void test_fault_reset_during_damped(void)
{
    HostWatchdog w; host_watchdog_reset(&w);
    host_watchdog_step(&w, LOST, 1u, 0u, LOST, DAMP);         /* → DAMPED */
    CHECK(w.state == HOST_LINK_DAMPED, "in DAMPED");
    CHECK(host_watchdog_step(&w, 0u, 1u, 1u, LOST, DAMP) == HOST_LINK_OK, "fault_reset from DAMPED");
}

int main(void)
{
    test_no_trip_when_unarmed();
    test_no_trip_under_decimation_gap();
    test_trip_sequence_and_latch();
    test_recovery_via_fault_reset();
    test_fault_reset_during_damped();
    if (failures == 0) { printf("OK host_watchdog\n"); return 0; }
    fprintf(stderr, "host_watchdog: %d FAILURE(S)\n", failures);
    return 1;
}
