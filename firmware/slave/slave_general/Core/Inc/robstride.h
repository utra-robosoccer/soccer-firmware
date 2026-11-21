/* robstride — thin send wrappers over the pure RobStride codec: each call
 * encodes one frame (common/motor_rs_can.h) and queues it on can_link.
 * Decoders are pure and called straight from motor_chain's RX dispatch. */
#ifndef ROBSTRIDE
#define ROBSTRIDE
#include <stdint.h>
#include "can_link.h"     // transport: can_link_tx / can_link_on_frame
/* Pure RobStride CAN codec (ext-id layout, RsFrame, rs_*_encode / rs_decode_*),
   path relative to Core/Inc/, as in proto_common.h. */
#include "../../../../common/include/motor_rs_can.h"

#define CAN_MASTER_ID 0xFD

// Comm Type 0
HAL_StatusTypeDef rs_get_motor_id(uint8_t id, uint16_t master_id);

// Comm Type 1
HAL_StatusTypeDef rs_mit_control_set(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd);
HAL_StatusTypeDef rs_read_motor_state(uint8_t id);

// Comm Type 3
HAL_StatusTypeDef rs_enable_motor(uint8_t id, uint16_t master_id);

// Comm Type 4
HAL_StatusTypeDef rs_disable_motor(uint8_t id, uint16_t master_id);
HAL_StatusTypeDef rs_clear_fault(uint8_t id, uint16_t master_id);  // Type-4, Byte0=1

// Comm Type 6
HAL_StatusTypeDef rs_set_mech_zero(uint8_t id, uint16_t master_id);

// Comm Type 17
HAL_StatusTypeDef rs_read_single_param(uint8_t id, uint16_t master_id, uint16_t index);

// Comm Type 18
HAL_StatusTypeDef rs_change_motor_mode(uint8_t id, uint16_t master_id, rs_runmode_t rs_runmode);

#endif
