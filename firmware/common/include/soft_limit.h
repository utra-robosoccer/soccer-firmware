/* soft_limit — pure one-sided soft-limit clamp (HAL-free, host-testable).
 *
 * A joint can end up OUTSIDE its [lo,hi] soft range (manual move while idle, a power-cycle
 * single-turn wrap, re-zeroing). Clamping a command to the limit then would fight the shaft
 * (command lo/hi while the shaft is further out → a position error large enough to trip the
 * overtorque guard). Instead the allowed range is EXTENDED to include the actual position:
 * a command may hold where the shaft is or move it back toward [lo,hi], but never further
 * OUT than it already is. Inside the range this is the ordinary [lo,hi] clamp.
 */
#ifndef SOFT_LIMIT_H
#define SOFT_LIMIT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Clamp *pos one-sided given the shaft's actual position. Returns 1 if it was clamped. */
static inline uint8_t soft_clamp_pos(float *pos, float actual, float lo, float hi)
{
    float ehi = (actual > hi) ? actual : hi;   /* allow holding/returning from beyond hi */
    float elo = (actual < lo) ? actual : lo;   /* ...or beyond lo                          */
    if (*pos > ehi) { *pos = ehi; return 1u; }
    if (*pos < elo) { *pos = elo; return 1u; }
    return 0u;
}

#ifdef __cplusplus
}
#endif
#endif /* SOFT_LIMIT_H */
