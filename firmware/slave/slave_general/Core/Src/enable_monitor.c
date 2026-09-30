#include "enable_monitor.h"

EnableMonVerdict enable_monitor_step(uint8_t active, uint8_t fresh_fb,
                                     uint8_t reported_norm, uint8_t k_threshold,
                                     uint8_t *count)
{
    if (count == 0) {
        return ENABLE_MON_OK;
    }
    if (!active) {                    /* idle / suspended / entry not done */
        *count = 0u;
        return ENABLE_MON_OK;
    }
    if (!fresh_fb) {                  /* stale tick: hold; silence is CAN_TIMEOUT's job */
        return ENABLE_MON_OK;
    }
    if (reported_norm) {              /* healthy fresh frame → reset */
        *count = 0u;
        return ENABLE_MON_OK;
    }
    if (*count < 255u) {
        (*count)++;
    }
    return (*count >= k_threshold) ? ENABLE_MON_FAULT_NOT_ENABLED : ENABLE_MON_OK;
}
