#ifndef MOTOR_RUNTIME_H
#define MOTOR_RUNTIME_H

#include "stm32f4xx_hal.h"
#include "proto_common.h"
#include <stdint.h>

#define MOTOR_WATCHDOG_MS 200u

typedef struct {
    const MotorConfig *cfg;
    MotorLifecycle     state;
    float              pos;
    float              vel;
    float              tau;
    float              temp;
    float              hold_pos;
    float              hold_vel;    /* velocity feedforward for ARMED_HOLD */
    uint16_t           fault_flags;
    uint16_t           last_cmd_seq;
    uint32_t           watchdog_ms;
    uint8_t            alive;
} MotorRuntime;

extern MotorRuntime motors_rt[N_MOTORS];

/* Lifecycle */
void motor_runtime_init(void);

/* Called every 10 ms from the CAN poll timer */
void motor_runtime_update(uint32_t now_ms);

/* ARM_HOLD: transition IDLE → ARMED_HOLD. Returns HAL_OK or HAL_ERROR. */
HAL_StatusTypeDef motor_runtime_arm(uint8_t idx, uint16_t cmd_seq);

/* GOTO_ZERO: enable, creep toward pos=0 with low gains, then lock ARMED_HOLD. */
HAL_StatusTypeDef motor_runtime_goto_zero(uint8_t idx, uint16_t cmd_seq);

/* DISABLE: transition any state → IDLE */
HAL_StatusTypeDef motor_runtime_disable(uint8_t idx, uint16_t cmd_seq);

/* Refresh watchdog (called when master sends SPI_CMD_HOLD) */
void motor_runtime_refresh_watchdog(uint8_t idx);

/* Pack telemetry for SPI TX buffer */
void motor_runtime_pack_tele(SpiMotorTele *out, uint8_t idx);

/* Aggregate alive bitmask (bit i = motor i alive) */
uint8_t motor_runtime_motors_alive(void);

#endif /* MOTOR_RUNTIME_H */
