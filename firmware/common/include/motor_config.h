#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define N_MOTORS 2u

/* Encoding bounds shared by slave (pack) and master (unpack) */
#define MOTOR_P_MIN  -12.57f
#define MOTOR_P_MAX   12.57f
#define MOTOR_V_MIN  -44.0f
#define MOTOR_V_MAX   44.0f
#define MOTOR_T_MIN  -17.0f
#define MOTOR_T_MAX   17.0f

/* Goto-zero motion parameters */
#define MOTOR_ZERO_TOL   0.05f  /* rad (~3°)  — arrival threshold          */
#define MOTOR_ZERO_RATE  0.3f   /* rad/s      — constant approach speed     */
#define MOTOR_ZERO_KP    4.0f   /* position gain while zeroing              */
#define MOTOR_ZERO_KD    1.0f


typedef struct {
    uint8_t     can_id;
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
        .can_id     = 2u,
        .joint_name = "motor0",
        .soft_min   = -6.28f,
        .soft_max   =  6.28f,
        .max_vel    = 10.0f,
        .max_tau    = 10.0f,
        .default_kp = 15.0f,
        .default_kd =  1.0f,
    },
    {
        .can_id     = 1u,
        .joint_name = "motor1",
        .soft_min   = -6.28f,
        .soft_max   =  6.28f,
        .max_vel    = 10.0f,
        .max_tau    = 10.0f,
        .default_kp = 15.0f,
        .default_kd =  1.0f,
    },
};

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_CONFIG_H */
