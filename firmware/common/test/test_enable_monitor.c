/* Host unit tests for enable_monitor_step (enable_monitor.c).
 *
 * The monitor decides, per control tick, whether an armed motor that keeps
 * reporting not-running should be faulted (NOT_ENABLED). Pure function, so we
 * drive it with explicit tick sequences and assert the verdict + counter.
 *
 * Build (see host/tests/test_enable_monitor.py, which runs it in CI):
 *   gcc -std=c11 -I firmware/slave/slave_general/Core/Inc \
 *       firmware/common/test/test_enable_monitor.c \
 *       firmware/slave/slave_general/Core/Src/enable_monitor.c -o test_enable_monitor
 * Exit 0 = all pass.
 */
#include "enable_monitor.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

/* Convenience: NORMAL=1 means running. */
#define NORMAL   1u
#define NOT_NORM 0u
#define FRESH    1u
#define STALE    0u
#define ON       1u
#define OFF      0u

int main(void)
{
    /* 1. Steady NORMAL fresh frames never fault; count stays 0. */
    {
        uint8_t c = 0;
        for (int i = 0; i < 50; i++)
            CHECK(enable_monitor_step(ON, FRESH, NORMAL, 3u, &c) == ENABLE_MON_OK,
                  "steady normal must stay OK");
        CHECK(c == 0u, "steady normal count stays 0");
    }

    /* 2a. K=3: faults exactly on the 3rd consecutive not-NORMAL fresh frame. */
    {
        uint8_t c = 0;
        CHECK(enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &c) == ENABLE_MON_OK,   "1st not-norm OK");
        CHECK(c == 1u, "count 1");
        CHECK(enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &c) == ENABLE_MON_OK,   "2nd not-norm OK");
        CHECK(c == 2u, "count 2");
        CHECK(enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &c) == ENABLE_MON_FAULT_NOT_ENABLED,
              "3rd not-norm faults");
    }

    /* 2b. K=1: faults on the very first not-NORMAL fresh frame. */
    {
        uint8_t c = 0;
        CHECK(enable_monitor_step(ON, FRESH, NOT_NORM, 1u, &c) == ENABLE_MON_FAULT_NOT_ENABLED,
              "K=1 faults immediately");
    }

    /* 3. A NORMAL fresh frame resets the run; interspersed never reaches K. */
    {
        uint8_t c = 0;
        enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &c);   /* 1 */
        enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &c);   /* 2 */
        CHECK(enable_monitor_step(ON, FRESH, NORMAL, 3u, &c) == ENABLE_MON_OK, "normal resets");
        CHECK(c == 0u, "count reset by normal");
        CHECK(enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &c) == ENABLE_MON_OK, "post-reset 1 OK");
        CHECK(c == 1u, "count back to 1");
    }

    /* 4. Stale ticks (no fresh feedback) hold the count and never fault; a fresh
          not-NORMAL frame then resumes counting to the fault. */
    {
        uint8_t c = 0;
        enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &c);   /* count 1 */
        enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &c);   /* count 2 */
        for (int i = 0; i < 100; i++)
            CHECK(enable_monitor_step(ON, STALE, NOT_NORM, 3u, &c) == ENABLE_MON_OK,
                  "stale never faults");
        CHECK(c == 2u, "stale holds the count");
        CHECK(enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &c) == ENABLE_MON_FAULT_NOT_ENABLED,
              "fresh after stale reaches the fault");
    }

    /* 5. active=0 (idle / entry-not-done) resets the count regardless of inputs. */
    {
        uint8_t c = 2;
        CHECK(enable_monitor_step(OFF, FRESH, NOT_NORM, 3u, &c) == ENABLE_MON_OK, "inactive OK");
        CHECK(c == 0u, "inactive resets count");
    }

    /* 6. Suspend→resume window: while suspended (active=0) the count stays 0; on
          resume, a failed re-enable (fresh not-NORMAL x K) faults, while a good
          re-enable (fresh NORMAL) stays OK. */
    {
        uint8_t c = 0;
        /* mid-run, then a suspend window */
        enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &c);        /* count 1 */
        for (int i = 0; i < 5; i++)
            enable_monitor_step(OFF, FRESH, NOT_NORM, 3u, &c);   /* suspended */
        CHECK(c == 0u, "suspend clears count");
        /* resume, good re-enable */
        uint8_t g = 0;
        for (int i = 0; i < 10; i++)
            CHECK(enable_monitor_step(ON, FRESH, NORMAL, 3u, &g) == ENABLE_MON_OK,
                  "resume good re-enable OK");
        /* resume, failed re-enable */
        uint8_t b = 0;
        enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &b);
        enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &b);
        CHECK(enable_monitor_step(ON, FRESH, NOT_NORM, 3u, &b) == ENABLE_MON_FAULT_NOT_ENABLED,
              "resume failed re-enable faults");
    }

    /* 7. Counter saturates and does not wrap past 255. */
    {
        uint8_t c = 0;
        for (int i = 0; i < 1000; i++)
            enable_monitor_step(ON, FRESH, NOT_NORM, 200u, &c);
        CHECK(c == 255u, "counter saturates at 255");
    }

    /* 8. NULL count pointer is safe. */
    CHECK(enable_monitor_step(ON, FRESH, NOT_NORM, 3u, 0) == ENABLE_MON_OK, "NULL count safe");

    if (failures) { fprintf(stderr, "%d failure(s)\n", failures); return 1; }
    printf("enable_monitor: all tests passed\n");
    return 0;
}
