#ifndef ROBOSTRIDE
#define ROBOSTRIDE
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"
#include <string.h> // Required for memcpy

// --- Constants (DO NOT CHANGE) ---
#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -20.0f
#define V_MAX    20.0f
#define KP_MIN    0.0f
#define KP_MAX 5000.0f
#define KD_MIN    0.0f
#define KD_MAX  100.0f
#define T_MIN   -60.0f
#define T_MAX    60.0f

#define CAN_MASTER_ID 0xFD

// --- Structs ---
typedef struct {
    uint32_t id   : 8;  // Bits 0-7: Target Motor ID (Tx) or Master ID (Rx Type 2)
    uint32_t data : 16; // Bits 8-23: Data Payload / Master ID / Status
    uint32_t mode : 5;  // Bits 24-28: Communication Type
    uint32_t res  : 3;  // Bits 29-31: Unused (CAN Ext ID is 29 bits)
} exCanIdInfo;

typedef enum{
    MIT_MODE        = 0,
    POS_PP_MODE     = 1,
    VELOCITY_MODE   = 2,
    CURRENT_MODE    = 3,
    CSP_MODE        = 4
} rs_runmode_t;

typedef enum{
    RS_MODE_RESET = 0,
    RS_MODE_CALI,
    RS_MODE_NORMAL
} motor_status_t;

// --- Feedback Decoding Masks (Based on exCanIdInfo.data field) ---
// The .data field corresponds to CAN bits 8-23 (16 bits total).
// According to Datasheet Type 2 Rx:
// CAN Bits 8-15:  Motor ID -> .data bits 0-7
// CAN Bits 16-21: Faults   -> .data bits 8-13
// CAN Bits 22-23: Mode     -> .data bits 14-15

#define RS_FB_DATA_ID_MASK      0x00FF
#define RS_FB_DATA_FAULT_MASK   0x3F00
#define RS_FB_DATA_MODE_MASK    0xC000

// Internal Fault Bit Offsets (Relative to the shifted fault byte)
#define FAULT_BIT_UNCALIBRATED    5 // Bit 21 in CAN (5 in fault byte)
#define FAULT_BIT_STALL_OVERLOAD  4 // Bit 20 in CAN
#define FAULT_BIT_ENCODER_FAULT   3 // Bit 19 in CAN
#define FAULT_BIT_OVERHEAT        2 // Bit 18 in CAN
#define FAULT_BIT_DRIVER_FAULT    1 // Bit 17 in CAN
#define FAULT_BIT_UNDERVOLTAGE    0 // Bit 16 in CAN

typedef struct {
    uint8_t uncalibrated;
    uint8_t stall_overload;
    uint8_t encoder_fault;
    uint8_t overheat;
    uint8_t driver_fault;
    uint8_t undervoltage;
} motor_error_t;

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
} motor_t;



extern CAN_RxHeaderTypeDef rs_can_rx_header;

// --- Function Prototypes ---

// Comm Type 0
HAL_StatusTypeDef can_get_motor_id(uint8_t id, uint16_t master_id);
HAL_StatusTypeDef can_unpack_get_id(motor_t* motor, uint8_t* recv_buf);

// Comm Type 1
HAL_StatusTypeDef can_mit_control_set(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd);

// Comm Type 2 (Rx Handling)
HAL_StatusTypeDef can_unpack_motor_feedback(motor_t* motor, uint8_t* recv_buf);

// Comm Type 3
HAL_StatusTypeDef can_enable_motor(uint8_t id, uint16_t master_id);

// Comm Type 4
HAL_StatusTypeDef can_disable_motor(uint8_t id, uint16_t master_id);

// Comm Type 6
HAL_StatusTypeDef can_set_mech_zero(uint8_t id, uint16_t master_id);

// Comm Type 7
HAL_StatusTypeDef can_set_motor_can_id(uint8_t id, uint16_t master_id, uint8_t new_motor_id);

// Comm Type 17
HAL_StatusTypeDef can_read_single_param(uint8_t id, uint16_t master_id, uint16_t index);
HAL_StatusTypeDef can_unpack_single_param(uint8_t* recv_buf, float* result_val);

// Comm Type 18
HAL_StatusTypeDef can_change_motor_mode(uint8_t id, uint16_t master_id, rs_runmode_t rs_runmode);

#endif
