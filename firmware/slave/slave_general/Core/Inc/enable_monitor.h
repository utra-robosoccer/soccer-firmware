/* enable_monitor — continuous "is the motor actually enabled?" safety check.
 *
 * Pure decision logic (no HAL, no globals) so it is host-testable. While the slave
 * holds a motor armed, the motor must report running mode in its Type-2 feedback;
 * if it reports otherwise for K consecutive FRESH feedback frames, the caller must
 * fault the motor (CAUSE_NOT_ENABLED) rather than keep believing it is armed.
 *
 * Silence (no fresh feedback at all) is NOT this module's concern — the caller's
 * CAN-feedback timeout owns that; here a stale tick simply holds the counter.
 */
#ifndef ENABLE_MONITOR_H
#define ENABLE_MONITOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENABLE_MON_OK = 0,
    ENABLE_MON_FAULT_NOT_ENABLED = 1
} EnableMonVerdict;

/* Advance the monitor one control tick for one motor. Updates *count in place.
 *
 *   active        : 1 if the check should run this tick — motor is in an armed
 *                   lifecycle, its entry handshake is done, and it is not inside a
 *                   firmware-intentional disable window. 0 forces *count = 0.
 *   fresh_fb      : 1 if a new Type-2 feedback frame arrived since the last call.
 *   reported_norm : 1 if the motor's reported mode == running/NORMAL, else 0.
 *   k_threshold   : consecutive not-NORMAL fresh frames required to fault (>= 1).
 *   count         : caller-owned per-motor counter (in/out).
 *
 * Returns ENABLE_MON_FAULT_NOT_ENABLED once *count reaches k_threshold, else OK.
 */
EnableMonVerdict enable_monitor_step(uint8_t active, uint8_t fresh_fb,
                                     uint8_t reported_norm, uint8_t k_threshold,
                                     uint8_t *count);

#ifdef __cplusplus
}
#endif

#endif /* ENABLE_MONITOR_H */
