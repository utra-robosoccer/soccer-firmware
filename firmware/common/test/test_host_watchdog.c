/* Pure (HAL-free) tests for the master host-death watchdog (host_watchdog.h).
 * Exit 0 = OK. */
#include "host_watchdog.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

#define LOST 12u    /* host-death trigger (cycles) */

static void test_no_trip_when_unarmed(void)
{
    HostWatchdog w; host_watchdog_reset(&w);
    for (uint32_t c = 0; c < 100u; c++) {
        CHECK(host_watchdog_step(&w, c, /*armed=*/0u, 0u, LOST) == HOST_LINK_OK,
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
        CHECK(host_watchdog_step(&w, since, /*armed=*/1u, 0u, LOST) == HOST_LINK_OK,
              "normal decimation gap must not trip");
    }
}

static void test_trip_and_latch(void)
{
    HostWatchdog w; host_watchdog_reset(&w);
    /* Stay OK until the gap reaches LOST. */
    for (uint32_t since = 0; since < LOST; since++)
        CHECK(host_watchdog_step(&w, since, 1u, 0u, LOST) == HOST_LINK_OK, "pre-trip OK");
    /* At LOST → STOPPED (the slave owns the ramp from here). */
    CHECK(host_watchdog_step(&w, LOST, 1u, 0u, LOST) == HOST_LINK_STOPPED,
          "trips to STOPPED");
    /* Latched: fresh commands (since=0) without a RESET stay STOPPED — the master
       keeps forcing STOP, blocking re-enable, until the host acknowledges. */
    for (uint32_t k = 0; k < 20u; k++)
        CHECK(host_watchdog_step(&w, 0u, 1u, /*reset_seen=*/0u, LOST) == HOST_LINK_STOPPED,
              "latched until a RESET request");
    /* Still latched even once nothing is armed any more (slave reached DISABLED). */
    CHECK(host_watchdog_step(&w, 0u, /*armed=*/0u, 0u, LOST) == HOST_LINK_STOPPED,
          "stays latched when disarmed");
}

static void test_recovery_via_reset_request(void)
{
    HostWatchdog w; host_watchdog_reset(&w);
    host_watchdog_step(&w, LOST, 1u, 0u, LOST);               /* → STOPPED */
    CHECK(w.state == HOST_LINK_STOPPED, "reached STOPPED");
    /* A fresh command carrying any RESET request recovers to OK. */
    CHECK(host_watchdog_step(&w, 0u, 1u, /*reset_seen=*/1u, LOST) == HOST_LINK_OK,
          "RESET recovers");
    /* And it keeps running normally after. */
    CHECK(host_watchdog_step(&w, 1u, 1u, 0u, LOST) == HOST_LINK_OK, "runs after recovery");
    /* A RESET while OK is a no-op for the watchdog. */
    CHECK(host_watchdog_step(&w, 2u, 1u, 1u, LOST) == HOST_LINK_OK, "RESET while OK is benign");
}

int main(void)
{
    test_no_trip_when_unarmed();
    test_no_trip_under_decimation_gap();
    test_trip_and_latch();
    test_recovery_via_reset_request();
    if (failures == 0) { printf("OK host_watchdog\n"); return 0; }
    fprintf(stderr, "host_watchdog: %d FAILURE(S)\n", failures);
    return 1;
}
