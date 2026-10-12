/* AUTO-GENERATED — DO NOT EDIT.
 * Source: configs/2s_10m/firmware_config.yaml (section slave0)
 * Regenerate: SOCCER_SETUP=2s_10m python3 scripts/gen_motor_config.py --slave slave0
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define N_MOTORS 5u

/* SPI transport encoding bounds shared by slave (pack) and master (unpack).
 * Set to the widest model so every motor's value fits losslessly. */
#define MOTOR_P_MIN  -12.57f
#define MOTOR_P_MAX  12.57f
#define MOTOR_V_MIN  -44.0f
#define MOTOR_V_MAX  44.0f
#define MOTOR_T_MIN  -17.0f
#define MOTOR_T_MAX  17.0f

/* TO_ZERO motion parameters */
#define MOTOR_WOUND_OFFSET_MAX 9.0f
#define MOTOR_FAULT_CHECK_DISABLED 0u

/* Timing, rates & derived tick counts (system-wide) */
/* Configured rates (Hz) — reported in MasterStatus and the .bin header. */
#define MASTER_POLL_HZ  200u
#define TELEMETRY_HZ    200u
#define SLAVE_TICK_HZ   200u
#define HOST_CMD_HZ     200u

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
#define MOTOR_CAN_FB_TIMEOUT_MS  100u
#define MOTOR_ENABLE_MON_K       3u   /* 15 ms / 5 ms tick */
#define MOTOR_ENABLE_TIMEOUT_MS  100u   /* ENABLING → DRVF_ENABLE_FAILED */
#define MOTOR_CLEAR_TIMEOUT_MS   100u   /* CLEARING → ABSENT */
#define MOTOR_ABSENT_PING_MS     100u   /* poll period while ABSENT */

/* Dead-man watchdogs (task 6). Master host-death is counted in master cycles; the slave
   master-loss ramp is in ms (HAL_GetTick). */
#define HOST_CMD_TIMEOUT_MS      25u   /* 3 host periods, 25 ms floor */
#define HOST_LOST_CYCLES         5u   /* host-death trigger (master cycles) */
#define MASTER_LOST_GRACE_MS     50u   /* slave hold (v/tau=0) after exchanges stop */
#define MASTER_LOST_DAMP_MS      300u   /* stop-ramp damp dwell (STOPPING) */

/* Slave SPI TX-arm deadline (µs after an exchange): wait for motor replies, then arm the
   next exchange's DMA so the fresh reply rides it (removes the one-exchange telemetry lag). */
#define TX_ARM_DEADLINE_US       3000u

/* Motor models present on the bus */
typedef enum {
    MOTOR_MODEL_RS00 = 0u,
    MOTOR_MODEL_RS02 = 1u
} MotorModel;

/* Per-model CAN "operation control mode" (Type 1) ranges — velocity, torque AND
 * Kp/Kd (RS00/02 use Kp 0–500, Kd 0–5; RS03/04/06 use Kp 0–5000, Kd 0–100).
 * From slaveN.yaml `models:`. Only position is shared (MOTOR_P_MIN/MAX). */
typedef struct {
    float v_min;
    float v_max;
    float t_min;
    float t_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
} MotorCanRange;

static const MotorCanRange motor_can_ranges[] = {
    [MOTOR_MODEL_RS00] = { -33.0f, 33.0f, -14.0f, 14.0f, 0.0f, 500.0f, 0.0f, 5.0f },
    [MOTOR_MODEL_RS02] = { -44.0f, 44.0f, -17.0f, 17.0f, 0.0f, 500.0f, 0.0f, 5.0f }
};

typedef struct {
    uint8_t     can_id;
    MotorModel  model;
    const char *joint_name;
    float       soft_min;    /* rad */
    float       soft_max;    /* rad */
    float       max_vel;     /* rad/s */
    float       max_tau;     /* Nm — torque trip: |tau| above this idles motor */
    float       default_kp;
    float       default_kd;
} MotorConfig;

static const MotorConfig motor_configs[N_MOTORS] = {
    {
        .can_id     = 1u,
        .model      = MOTOR_MODEL_RS02,
        .joint_name = "bench_j1",
        .soft_min   = -0.79f,
        .soft_max   = 0.79f,
        .max_vel    = 10.0f,
        .max_tau    = 0.8f,
        .default_kp = 15.0f,
        .default_kd = 1.0f,
    },
    {
        .can_id     = 2u,
        .model      = MOTOR_MODEL_RS02,
        .joint_name = "bench_j2",
        .soft_min   = -1.05f,
        .soft_max   = 1.05f,
        .max_vel    = 10.0f,
        .max_tau    = 0.8f,
        .default_kp = 15.0f,
        .default_kd = 1.0f,
    },
    {
        .can_id     = 3u,
        .model      = MOTOR_MODEL_RS00,
        .joint_name = "bench_j3",
        .soft_min   = -1.31f,
        .soft_max   = 1.31f,
        .max_vel    = 10.0f,
        .max_tau    = 0.8f,
        .default_kp = 15.0f,
        .default_kd = 1.0f,
    },
    {
        .can_id     = 4u,
        .model      = MOTOR_MODEL_RS00,
        .joint_name = "bench_j4",
        .soft_min   = -1.31f,
        .soft_max   = 1.31f,
        .max_vel    = 10.0f,
        .max_tau    = 0.8f,
        .default_kp = 15.0f,
        .default_kd = 1.0f,
    },
    {
        .can_id     = 5u,
        .model      = MOTOR_MODEL_RS00,
        .joint_name = "bench_j5",
        .soft_min   = -1.31f,
        .soft_max   = 1.31f,
        .max_vel    = 10.0f,
        .max_tau    = 0.8f,
        .default_kp = 15.0f,
        .default_kd = 1.0f,
    }
};

/* Look up a motor's per-model CAN range by its bus id. Returns NULL if the id
 * is not part of this slave's chain. */
static inline const MotorCanRange *motor_can_range_by_id(uint8_t can_id)
{
    for (uint8_t i = 0u; i < N_MOTORS; i++) {
        if (motor_configs[i].can_id == can_id) {
            return &motor_can_ranges[motor_configs[i].model];
        }
    }
    return (const MotorCanRange *)0;
}

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_CONFIG_H */
