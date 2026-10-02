/* AUTO-GENERATED — DO NOT EDIT.
 * Sources: 1s_1m/slave0.yaml
 * Regenerate: python3 scripts/gen_motor_config.py --system configs/1s_1m/slave0.yaml
 */
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_SLAVES            1u
#define MAX_MOTORS_PER_SLAVE  1u
#define TOTAL_MOTORS          1u

/* SPI transport encoding bounds (global, widest model across all slaves). */
#define MOTOR_P_MIN  -12.57f
#define MOTOR_P_MAX  12.57f
#define MOTOR_V_MIN  -44.0f
#define MOTOR_V_MAX  44.0f
#define MOTOR_T_MIN  -17.0f
#define MOTOR_T_MAX  17.0f

/* Timing, rates & derived tick counts (system-wide; mirror of motor_config.h). */
/* Configured rates (Hz) — reported in MasterStatus and the .bin header. */
#define MASTER_POLL_HZ  200u
#define TELEMETRY_HZ    200u
#define SLAVE_TICK_HZ   200u
#define HOST_CMD_HZ     50u

/* One master poll cycle (µs). The slave derives its forward/fallback service
   thresholds from this, so they scale with the poll rate (incl. 400 Hz). */
#define MASTER_CYCLE_US  5000u

/* Master SPI1 clock divider off APB2 (72 MHz): MX_SPI1_Init maps it to the
   SPI_BAUDRATEPRESCALER_* enum. Valid: 2,4,8,16,32,64,128,256. */
#define MASTER_SPI_PRESCALER_DIV  16u

/* Derived loop periods (ms) — do not hand-edit; change the rate instead. */
#define MASTER_POLL_PERIOD_MS  5u
#define MASTER_TELE_PERIOD_MS  5u
#define MOTOR_LOOP_PERIOD_MS   5u
#define MOTOR_LOOP_DT_S        ((float)MOTOR_LOOP_PERIOD_MS * 0.001f)

/* Timeouts/debounces (ms) and tick/frame counts derived from the slave tick. */
#define MOTOR_WATCHDOG_MS        200u
#define MOTOR_CAN_FB_TIMEOUT_MS  100u
#define MOTOR_ZERO_STALL_MS      1500u
#define MOTOR_ZERO_SETTLE_TICKS  10u   /* 50 ms / 5 ms tick */
#define MOTOR_ENABLE_MON_K       3u   /* 15 ms / 5 ms tick */

/* Dead-man watchdogs (task 6). Master host-death is counted in master cycles; the slave
   master-loss ramp is in ms (HAL_GetTick). */
#define HOST_CMD_TIMEOUT_MS      60u   /* 3 host periods, 25 ms floor */
#define HOST_LOST_CYCLES         12u   /* host-death trigger (master cycles) */
#define HOST_LOST_DAMP_CYCLES    60u   /* master DAMPED dwell before IDLE (cycles) */
#define MASTER_LOST_GRACE_MS     50u   /* slave hold (v/tau=0) after exchanges stop */
#define MASTER_LOST_DAMP_MS      300u   /* slave DAMPED dwell before IDLE */

/* Slave SPI TX-arm deadline (µs after an exchange): wait for motor replies, then arm the
   next exchange's DMA so the fresh reply rides it (removes the one-exchange telemetry lag). */
#define TX_ARM_DEADLINE_US       3000u

/* Number of active motors on each slave (chain order). */
static const uint8_t slave_motor_counts[NUM_SLAVES] = { 1u };

/* Per-slave RobStride CAN node ids (chain order). Unused slots are 0. */
static const uint8_t slave_motor_ids[NUM_SLAVES][MAX_MOTORS_PER_SLAVE] = {
    { 1u }
};

#ifdef __cplusplus
}
#endif
#endif /* SYSTEM_CONFIG_H */
