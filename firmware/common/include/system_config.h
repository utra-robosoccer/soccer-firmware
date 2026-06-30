/* AUTO-GENERATED — DO NOT EDIT.
 * Sources: slave0.yaml, slave1.yaml
 * Regenerate: python3 scripts/gen_motor_config.py --system configs/slave0.yaml configs/slave1.yaml
 */
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_SLAVES            2u
#define MAX_MOTORS_PER_SLAVE  5u
#define TOTAL_MOTORS          7u

/* SPI transport encoding bounds (global, widest model across all slaves). */
#define MOTOR_P_MIN  -12.57f
#define MOTOR_P_MAX  12.57f
#define MOTOR_V_MIN  -44.0f
#define MOTOR_V_MAX  44.0f
#define MOTOR_T_MIN  -17.0f
#define MOTOR_T_MAX  17.0f

/* Number of active motors on each slave (chain order). */
static const uint8_t slave_motor_counts[NUM_SLAVES] = { 5u, 2u };

/* Per-slave RobStride CAN node ids (chain order). Unused slots are 0. */
static const uint8_t slave_motor_ids[NUM_SLAVES][MAX_MOTORS_PER_SLAVE] = {
    { 1u, 2u, 3u, 4u, 5u },
    { 6u, 7u, 0u, 0u, 0u }
};

#ifdef __cplusplus
}
#endif
#endif /* SYSTEM_CONFIG_H */
