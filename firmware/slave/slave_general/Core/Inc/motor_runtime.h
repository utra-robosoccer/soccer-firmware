#ifndef MOTOR_RUNTIME_H
#define MOTOR_RUNTIME_H

#include "stm32f4xx_hal.h"
#include "proto_common.h"
#include <stdint.h>

#define MOTOR_WATCHDOG_MS 200u

/* Per-motor CAN-feedback staleness threshold. If an armed motor's Type-2
   feedback goes silent this long, trip it to IDLE with CAUSE_CAN_TIMEOUT. Kept
   below MOTOR_WATCHDOG_MS so "CAN died" (this) and "master link died" (watchdog)
   stay distinguishable as fault causes. */
#define MOTOR_CAN_FB_TIMEOUT_MS 100u

/* Control-loop period — single source of truth for slave loop timing.
   motor_runtime_update() runs at this cadence; goto-zero derives its waypoint
   dt from MOTOR_LOOP_DT_S so the creep rate stays MOTOR_ZERO_RATE (rad/s)
   regardless of loop frequency. Keep zeroing slow enough that a human can cut
   power mid-creep. */
#define MOTOR_LOOP_PERIOD_MS 5U
#define MOTOR_LOOP_DT_S      ((float)MOTOR_LOOP_PERIOD_MS * 0.001f)

typedef struct {
    const MotorConfig *cfg;
    MotorLifecycle     state;
    float              pos;          /* wrapped to [-pi, pi] (home frame) */
    float              vel;
    float              tau;
    float              temp;
    float              hold_pos;      /* home-frame setpoint */
    float              hold_vel;      /* velocity feedforward for ARMED_HOLD */
    float              pos_offset;    /* raw - wrapped: the multiple of 2*pi the
                                         motor adopted at power-up. Added back to
                                         every command so the motor takes the
                                         short path to a home-frame target. */
    uint8_t            motor_fault;   /* 6 compact Type-2 fault bits (u8)         */
    uint32_t           fault_word;    /* telemetry mirror of the snapshot's       */
    uint32_t           last_fb_ms;    /* snapshot's last-Type-2 stamp, for fb_age */
    uint8_t            cause;         /* MotorFaultCause — LATCHED on trip,
                                         cleared only on arm/disable              */
    uint8_t            cmd_flags;     /* SPI_CMDFLAG_* — recomputed every tick     */
    uint8_t            last_apply_clamp; /* clamp bits from the most recent apply_mit */
    uint8_t            got_fresh_cmd; /* a fresh MIT arrived since the last tick   */
    uint32_t           watchdog_ms;
    uint8_t            alive;
} MotorRuntime;

extern MotorRuntime motors_rt[N_MOTORS];

/* Lifecycle */
void motor_runtime_init(void);

/* Called every MOTOR_LOOP_PERIOD_MS from the CAN poll loop */
void motor_runtime_update(uint32_t now_ms);

/* ARM_HOLD: transition IDLE → ARMED_HOLD. Returns HAL_OK or HAL_ERROR.
   Clears any latched fault (cause + fault_word), issuing a CAN fault-clear first
   when the motor's own fault was latched. ARM is thus a fault-clearing action —
   the host must treat it as deliberate/edge-triggered, never a periodic retry. */
HAL_StatusTypeDef motor_runtime_arm(uint8_t idx);

/* GOTO_ZERO: enable, creep toward pos=0 with low gains, then lock ARMED_HOLD. */
HAL_StatusTypeDef motor_runtime_goto_zero(uint8_t idx);

/* DISABLE: transition any state → IDLE (commanded; clears latched fault) */
HAL_StatusTypeDef motor_runtime_disable(uint8_t idx);

/* Refresh watchdog (called when master sends SPI_CMD_HOLD) */
void motor_runtime_refresh_watchdog(uint8_t idx);

/* Apply a streamed MIT setpoint while armed, clamping the commanded position
   to the motor's soft angle limits (soft_min/soft_max). No-op if not armed. */
void motor_runtime_apply_mit(uint8_t idx, float pos, float vel);

/* Pack one motor's telemetry atom for the SPI TX frame */
void motor_runtime_pack_tele(MotorState *out, uint8_t idx);

/* Aggregate alive bitmask (bit i = motor i alive) */
uint8_t motor_runtime_motors_alive(void);

#endif /* MOTOR_RUNTIME_H */
