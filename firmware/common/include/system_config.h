/* AUTO-GENERATED — DO NOT EDIT.
 * Sources: bench-1-chain/slave0.yaml
 * Regenerate: python3 scripts/gen_motor_config.py --system configs/bench-1-chain/slave0.yaml
 */
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_SLAVES            1u
#define MAX_MOTORS_PER_SLAVE  3u
#define TOTAL_MOTORS          3u

/* SPI transport encoding bounds (global, widest model across all slaves). */
#define MOTOR_P_MIN  -12.57f
#define MOTOR_P_MAX  12.57f
#define MOTOR_V_MIN  -44.0f
#define MOTOR_V_MAX  44.0f
#define MOTOR_T_MIN  -17.0f
#define MOTOR_T_MAX  17.0f

/* Per-motor CAN-feedback staleness threshold (ms). Kept distinct from the
 * 200 ms master-link watchdog so "CAN died" and "upstream died" stay separable
 * as fault causes. */
#define MOTOR_CAN_FB_TIMEOUT_MS  100u

/* Number of active motors on each slave (chain order). */
static const uint8_t slave_motor_counts[NUM_SLAVES] = { 3u };

/* Per-slave RobStride CAN node ids (chain order). Unused slots are 0. */
static const uint8_t slave_motor_ids[NUM_SLAVES][MAX_MOTORS_PER_SLAVE] = {
    { 1u, 2u, 3u }
};

#ifdef __cplusplus
}
#endif
#endif /* SYSTEM_CONFIG_H */
