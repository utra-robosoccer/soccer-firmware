/*
 * motor_types.h — shared motor domain model.
 *
 * The plain-C data types that describe a motor's identity, mode, faults and
 * live telemetry. Extracted out of robostride.h so that modules which only need
 * the data model (motor_chain / can_rx, motor_runtime / motor_control) do not
 * have to include the CAN protocol driver. robostride.h includes this file and
 * keeps only the CAN-frame internals (exCanIdInfo, decode masks, scaling
 * constants) and the codec prototypes.
 *
 * No HAL / hardware dependency — safe to include from host-side unit tests.
 */
#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>

/* RobStride run mode (Communication Type 18 "run_mode" parameter). */
typedef enum {
    MIT_MODE        = 0,
    POS_PP_MODE     = 1,
    VELOCITY_MODE   = 2,
    CURRENT_MODE    = 3,
    CSP_MODE        = 4
} rs_runmode_t;

/* RobStride high-level operating status (Type-2 feedback mode field). */
typedef enum {
    RS_MODE_RESET = 0,
    RS_MODE_CALI,
    RS_MODE_NORMAL
} motor_status_t;

/* Decoded per-motor fault bits (from the Type-2 feedback fault field). */
typedef struct {
    uint8_t uncalibrated;
    uint8_t stall_overload;
    uint8_t encoder_fault;
    uint8_t overheat;
    uint8_t driver_fault;
    uint8_t undervoltage;
} motor_error_t;

/* Live per-motor state, populated from CAN feedback. */
typedef struct {
    uint8_t id;
    uint16_t master_id;
    uint64_t mcu_id; // Unique MCU Identifier (from Type 0)

    motor_status_t status;
    motor_error_t motor_errors;
    rs_runmode_t motor_mode;

    float temperature; // motor temperature

    // MIT parameters
    float pos;  // rad
    float rpm;  // rad/s
    float kp;
    float kd;
    float torq; // Nm

    // Telemetry health. Read only via motor_get_snapshot() (a critical-section
    // struct copy), so no volatile is needed here — the copy's memory barrier
    // orders the ISR's writes. last_fb_ms is ISR-written; fault_word is written
    // by the ISR (0x3022 latch) and by main via motor_set_fault_word().
    uint32_t last_fb_ms;   // HAL_GetTick() at last Type-2 feedback
    uint32_t fault_word;   // latched 0x3022 read (0=clear, 0xFFFFFFFF=read-fail)

    // Motor set point
    float set_pos;
    float set_rpm;
    float set_kp;
    float set_kd;
    float set_torq;
} motor_t;

#endif /* MOTOR_TYPES_H */
