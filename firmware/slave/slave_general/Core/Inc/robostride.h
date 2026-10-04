#ifndef ROBOSTRIDE
#define ROBOSTRIDE
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"
#include <string.h> // Required for memcpy
#include "motor_types.h"  // motor_t, motor_status_t, motor_error_t, rs_runmode_t
/* Strict-aliasing-safe RobStride ext-id codec (path relative to Core/Inc/, as in
   proto_common.h). Replaces the old (exCanIdInfo*)&ExtId pointer cast. */
#include "../../../../common/include/robostride_id.h"

// --- Constants (DO NOT CHANGE) ---
#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -44.0f
#define V_MAX    44.0f
#define KP_MIN    0.0f
#define KP_MAX  500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f
#define T_MIN   -17.0f
#define T_MAX    17.0f

#define CAN_MASTER_ID 0xFD
#define ROBOSTRIDE_READ_ONLY_MOTOR_ID 2u

// The 29-bit ext-id layout (id[0:7] / data[8:23] / mode[24:28]) is now encoded and
// decoded by rs_extid_pack/mode/data/id (robostride_id.h) with explicit shifts —
// no bitfield-struct pointer cast (that was undefined at -O2 under strict aliasing).

// rs_runmode_t and motor_status_t are defined in motor_types.h

// --- Feedback Decoding Masks (applied to the 16-bit data field, rs_extid_data()) ---
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

// motor_error_t and motor_t are defined in motor_types.h

extern uint32_t TxMailbox;
extern CAN_RxHeaderTypeDef rs_can_rx_header;
extern volatile uint32_t can_tx_ok_count;
extern volatile uint32_t can_tx_error_count;
extern volatile uint32_t can_last_tx_error;


// --- Function Prototypes ---

// CAN Bus Start
HAL_StatusTypeDef can_bus_init(void);
HAL_StatusTypeDef can_bus_init_read_state_filter(uint8_t motor_id, uint16_t master_id);

HAL_StatusTypeDef motor_enable_one_motor(uint8_t m_id);

// Comm Type 0
HAL_StatusTypeDef can_get_motor_id(uint8_t id, uint16_t master_id);
HAL_StatusTypeDef can_unpack_get_id(motor_t* motor, uint8_t* recv_buf);

// Comm Type 1
HAL_StatusTypeDef can_mit_control_set(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd);
HAL_StatusTypeDef can_read_motor_state(uint8_t id);

// Comm Type 2 (Rx Handling)
HAL_StatusTypeDef can_unpack_motor_feedback(motor_t* motor, uint8_t* recv_buf);


// Comm Type 3
HAL_StatusTypeDef can_enable_motor(uint8_t id, uint16_t master_id);

// Comm Type 4
HAL_StatusTypeDef can_disable_motor(uint8_t id, uint16_t master_id);
HAL_StatusTypeDef can_clear_fault(uint8_t id, uint16_t master_id);  // Type-4, Byte0=1

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
