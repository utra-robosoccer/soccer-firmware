/* host_watchdog — pure master-side host-death dead-man (HAL-free, host-testable).
 *
 * The master counts master cycles since the last FRESH host command (the mailbox swap).
 * While any motor is armed, if that count reaches HOST_LOST_CYCLES the master stops
 * forwarding the stale command and drives the slaves DAMPED for HOST_LOST_DAMP_CYCLES,
 * then IDLE — latched. Recovery is ONLY via a fresh host command carrying fault_reset.
 * A command that is not armed (everything IDLE) never trips it.
 */
#ifndef HOST_WATCHDOG_H
#define HOST_WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HOST_LINK_OK     = 0u,  /* forward the host command normally          */
    HOST_LINK_DAMPED = 1u,  /* host lost: master drives DAMPED            */
    HOST_LINK_IDLE   = 2u,  /* host lost: master drives IDLE (latched)    */
} HostLinkAction;

typedef struct {
    HostLinkAction state;
    uint32_t       damp_ticks;   /* cycles spent in DAMPED */
} HostWatchdog;

static inline void host_watchdog_reset(HostWatchdog *w)
{
    w->state = HOST_LINK_OK;
    w->damp_ticks = 0u;
}

/* One call per master cycle (after the mailbox swap).
 *   cycles_since_fresh : master cycles since the last fresh host command (0 = fresh now)
 *   armed              : 1 if any motor is currently armed
 *   fault_reset        : 1 if a FRESH host command carrying CMD_FLAG_FAULT_RESET arrived
 *   lost_cycles        : HOST_LOST_CYCLES (host-death trigger)
 *   damp_cycles        : HOST_LOST_DAMP_CYCLES (DAMPED dwell before IDLE)
 * Returns the action the master must apply to the slaves this cycle. */
static inline HostLinkAction host_watchdog_step(
    HostWatchdog *w, uint32_t cycles_since_fresh, uint8_t armed, uint8_t fault_reset,
    uint32_t lost_cycles, uint32_t damp_cycles)
{
    if (fault_reset) {                       /* explicit recovery */
        w->state = HOST_LINK_OK;
        w->damp_ticks = 0u;
    }
    switch (w->state) {
        case HOST_LINK_OK:
            if (armed && cycles_since_fresh >= lost_cycles) {
                w->state = HOST_LINK_DAMPED;
                w->damp_ticks = 0u;
            }
            break;
        case HOST_LINK_DAMPED:
            if (++w->damp_ticks >= damp_cycles) {
                w->state = HOST_LINK_IDLE;
            }
            break;
        case HOST_LINK_IDLE:
        default:
            break;                           /* latched until fault_reset */
    }
    return w->state;
}

#ifdef __cplusplus
}
#endif
#endif /* HOST_WATCHDOG_H */
