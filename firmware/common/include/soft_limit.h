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

/* Clamp bits returned by soft_clamp_mit (pure; the caller maps them to TELE_FLAG_*). */
#define SOFT_CLAMPED_POS 0x01u
#define SOFT_CLAMPED_FF  0x02u

/* Clamp *x into [lo,hi]. Returns 1 if it was clamped. */
static inline uint8_t soft_clamp_range(float *x, float lo, float hi)
{
    if (*x > hi) { *x = hi; return 1u; }
    if (*x < lo) { *x = lo; return 1u; }
    return 0u;
}

/* Full MIT-command clamp: the one-sided position clamp above, plus cancellation of any
 * feedforward (vel, tau_ff) that would drive further OUT past a clamped position, plus
 * magnitude bounds |vel| ≤ vel_lim (cfg->max_vel) and |tau_ff| ≤ tau_lim (cfg->max_tau,
 * the overtorque trip threshold — so the commanded feedforward alone can never exceed
 * what the torque guard allows). Returns SOFT_CLAMPED_* bits. */
static inline uint8_t soft_clamp_mit(float *pos, float *vel, float *tau_ff,
                                     float actual, float lo, float hi,
                                     float vel_lim, float tau_lim)
{
    float before = *pos;
    uint8_t clamp = 0u;
    if (soft_clamp_pos(pos, actual, lo, hi)) {
        clamp |= SOFT_CLAMPED_POS;
        if (before > *pos) {   /* clamped from above: outward = positive */
            if (*vel    > 0.0f) { *vel    = 0.0f; clamp |= SOFT_CLAMPED_FF; }
            if (*tau_ff > 0.0f) { *tau_ff = 0.0f; clamp |= SOFT_CLAMPED_FF; }
        } else {               /* clamped from below: outward = negative */
            if (*vel    < 0.0f) { *vel    = 0.0f; clamp |= SOFT_CLAMPED_FF; }
            if (*tau_ff < 0.0f) { *tau_ff = 0.0f; clamp |= SOFT_CLAMPED_FF; }
        }
    }
    if (soft_clamp_range(vel,    -vel_lim, vel_lim)) clamp |= SOFT_CLAMPED_FF;
    if (soft_clamp_range(tau_ff, -tau_lim, tau_lim)) clamp |= SOFT_CLAMPED_FF;
    return clamp;
}

#ifdef __cplusplus
}
#endif
#endif /* SOFT_LIMIT_H */
