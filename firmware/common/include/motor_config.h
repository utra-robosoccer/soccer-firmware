/* AUTO-GENERATED — DO NOT EDIT.
 * Source: configs/slave0.yaml
 * Regenerate: python3 scripts/gen_motor_config.py configs/slave0.yaml
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

/* Goto-zero motion parameters */
#define MOTOR_ZERO_TOL 0.05f
#define MOTOR_ZERO_RATE 0.3f
#define MOTOR_ZERO_KP  4.0f
#define MOTOR_ZERO_KD  1.0f

/* Motor models present on the bus */
typedef enum {
    MOTOR_MODEL_RS00 = 0u,
    MOTOR_MODEL_RS02 = 1u
} MotorModel;

/* Per-model CAN "operation control mode" (Type 1) velocity/torque ranges.
 * Position, Kp and Kd are identical across models and stay global. */
typedef struct {
    float v_min;
    float v_max;
    float t_min;
    float t_max;
} MotorCanRange;

static const MotorCanRange motor_can_ranges[] = {
    [MOTOR_MODEL_RS00] = { -33.0f, 33.0f, -14.0f, 14.0f },
    [MOTOR_MODEL_RS02] = { -44.0f, 44.0f, -17.0f, 17.0f }
};

typedef struct {
    uint8_t     can_id;
    MotorModel  model;
    const char *joint_name;
    float       soft_min;    /* rad */
    float       soft_max;    /* rad */
    float       max_vel;     /* rad/s */
    float       max_tau;     /* Nm */
    float       default_kp;
    float       default_kd;
} MotorConfig;

static const MotorConfig motor_configs[N_MOTORS] = {
    {
        .can_id     = 1u,
        .model      = MOTOR_MODEL_RS02,
        .joint_name = "motor0",
        .soft_min   = -6.28f,
        .soft_max   = 6.28f,
        .max_vel    = 10.0f,
        .max_tau    = 10.0f,
        .default_kp = 15.0f,
        .default_kd = 1.0f,
    },
    {
        .can_id     = 2u,
        .model      = MOTOR_MODEL_RS02,
        .joint_name = "motor1",
        .soft_min   = -6.28f,
        .soft_max   = 6.28f,
        .max_vel    = 10.0f,
        .max_tau    = 10.0f,
        .default_kp = 15.0f,
        .default_kd = 1.0f,
    },
    {
        .can_id     = 3u,
        .model      = MOTOR_MODEL_RS00,
        .joint_name = "motor2",
        .soft_min   = -6.28f,
        .soft_max   = 6.28f,
        .max_vel    = 10.0f,
        .max_tau    = 10.0f,
        .default_kp = 15.0f,
        .default_kd = 1.0f,
    },
    {
        .can_id     = 4u,
        .model      = MOTOR_MODEL_RS00,
        .joint_name = "motor3",
        .soft_min   = -6.28f,
        .soft_max   = 6.28f,
        .max_vel    = 10.0f,
        .max_tau    = 10.0f,
        .default_kp = 15.0f,
        .default_kd = 1.0f,
    },
    {
        .can_id     = 5u,
        .model      = MOTOR_MODEL_RS00,
        .joint_name = "motor4",
        .soft_min   = -6.28f,
        .soft_max   = 6.28f,
        .max_vel    = 10.0f,
        .max_tau    = 10.0f,
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
