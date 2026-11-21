/* robstride — thin send wrappers over the pure RobStride codec: each call
 * encodes one frame (common/motor_rs_can.h) and queues it on can_link.
 * Sends are fire-and-forget: HAL status stays inside can_link (failures are
 * accounted in can_tx_error_count -> telemetry can_tx_errors).
 * Decoders are pure and called straight from can_rx's RX dispatch. */
#ifndef ROBSTRIDE
#define ROBSTRIDE
#include <stdint.h>
#include "can_link.h"     // transport: can_link_tx / can_link_on_frame
/* Pure RobStride CAN codec (ext-id layout, RsFrame, rs_*_encode / rs_decode_*),
   path relative to Core/Inc/, as in proto_common.h. */
#include "../../../../common/include/motor_rs_can.h"

#define CAN_MASTER_ID 0xFD

// Comm Type 0
void rs_get_motor_id(uint8_t id, uint16_t master_id);

// Comm Type 1
void rs_mit_control_set(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd);
void rs_read_motor_state(uint8_t id);

// Comm Type 3
void rs_enable_motor(uint8_t id, uint16_t master_id);

// Comm Type 4
void rs_disable_motor(uint8_t id, uint16_t master_id);
void rs_clear_fault(uint8_t id, uint16_t master_id);  // Type-4, Byte0=1

// Comm Type 6
void rs_set_mech_zero(uint8_t id, uint16_t master_id);

// Comm Type 17
void rs_read_single_param(uint8_t id, uint16_t master_id, uint16_t index);

// Comm Type 18
void rs_change_motor_mode(uint8_t id, uint16_t master_id, rs_runmode_t rs_runmode);

#endif
