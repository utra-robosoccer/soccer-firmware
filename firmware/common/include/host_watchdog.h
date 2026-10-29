/* host_watchdog — pure master-side host-death dead-man (HAL-free, host-testable).
 *
 * The master counts master cycles since the last FRESH host command (the mailbox swap).
 * While any motor is armed, if that count reaches HOST_LOST_CYCLES the master stops
 * forwarding the stale command and forces STOP into every slot — latched. The slave
 * owns the ramp (STOPPING → DISABLED, drive_fault NONE); robot_state = HOST_LOST is
 * the record. Recovery is ONLY via a fresh host command carrying a RESET request.
 * A command stream that is not armed (everything off) never trips it.
 */
#ifndef HOST_WATCHDOG_H
#define HOST_WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HOST_LINK_OK      = 0u,  /* forward the host command normally            */
    HOST_LINK_STOPPED = 1u,  /* host lost: master forces STOP (latched)      */
} HostLinkAction;

typedef struct {
    HostLinkAction state;
} HostWatchdog;

static inline void host_watchdog_reset(HostWatchdog *w)
{
    w->state = HOST_LINK_OK;
}

/* One call per master cycle (after the mailbox swap).
 *   cycles_since_fresh : master cycles since the last fresh host command (0 = fresh now)
 *   armed              : 1 if any motor is currently armed (ENABLING/ENABLED/STOPPING)
 *   reset_seen         : 1 if a FRESH host command carrying a RESET request arrived
 *   lost_cycles        : HOST_LOST_CYCLES (host-death trigger)
 * Returns the action the master must apply to the slaves this cycle. */
static inline HostLinkAction host_watchdog_step(
    HostWatchdog *w, uint32_t cycles_since_fresh, uint8_t armed, uint8_t reset_seen,
    uint32_t lost_cycles)
{
    if (reset_seen) {                        /* explicit recovery */
        w->state = HOST_LINK_OK;
    }
    if (w->state == HOST_LINK_OK &&
        armed && cycles_since_fresh >= lost_cycles) {
        w->state = HOST_LINK_STOPPED;        /* latched until a RESET arrives */
    }
    return w->state;
}

#ifdef __cplusplus
}
#endif
#endif /* HOST_WATCHDOG_H */
