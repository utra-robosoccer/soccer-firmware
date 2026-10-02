/* mode_sm.c — pure mode-request state machine. See mode_sm.h. */
#include "mode_sm.h"
#include <string.h>

static uint8_t is_armed(uint8_t state)
{
    return state == LIFE_HOLD || state == LIFE_MIT ||
           state == LIFE_DAMPED || state == LIFE_TO_ZERO;
}

ModeDecision mode_sm_step(uint8_t state, uint8_t req, uint8_t valid,
                          uint8_t fault_reset, uint8_t wound)
{
    ModeDecision d;
    memset(&d, 0, sizeof(d));
    d.next_state = state;

    if (!valid) return d;            /* no request this cycle → hold state */

    /* fault_reset clears a latched fault, then the request is evaluated as if the
       motor were already IDLE. */
    if (fault_reset && state == LIFE_FAULT) {
        d.clear_fault = 1u;
        state = LIFE_IDLE;
        d.next_state = LIFE_IDLE;
    }

    switch (req) {
        case REQ_IDLE:
            if (state != LIFE_IDLE) d.do_disable = 1u;
            d.next_state = LIFE_IDLE;
            break;

        case REQ_HOLD:
            if (state == LIFE_IDLE) {
                if (wound) {                 /* refuse arm, latch CAUSE_WOUND */
                    d.next_state      = LIFE_FAULT;
                    d.set_cause_wound = 1u;
                    d.rejected        = 1u;
                } else {
                    d.do_arm     = 1u;       /* enable + capture (handshake) */
                    d.next_state = LIFE_HOLD;
                }
            } else if (is_armed(state)) {
                d.capture_hold = 1u;         /* re-hold, no re-enable */
                d.next_state   = LIFE_HOLD;
            } else {
                d.rejected = 1u;             /* FAULT / BOOT / DISCOVERING */
            }
            break;

        case REQ_MIT:
            if (is_armed(state)) d.next_state = LIFE_MIT;
            else                 d.rejected   = 1u;
            break;

        case REQ_DAMPED:
            if (is_armed(state)) d.next_state = LIFE_DAMPED;
            else                 d.rejected   = 1u;
            break;

        case REQ_TO_ZERO:
            if (is_armed(state)) {
                if (state != LIFE_TO_ZERO) d.enter_to_zero = 1u;
                d.next_state = LIFE_TO_ZERO;
            } else {
                d.rejected = 1u;
            }
            break;

        default:
            d.rejected = 1u;
            break;
    }
    return d;
}
