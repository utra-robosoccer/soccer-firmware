#include "motor_chain.h"
#include "robstride.h"   /* vendor-protocol frames: rs_unpack_* (wire-level RX is can_link.c) */

#define PI 3.1415926f

/* Private to this TU — owned by the CAN RX ISR. Main-loop code reads it only
   through motor_get_snapshot() and writes only via motor_set_fault_word(). */
static motor_t motors[MAX_MOTOR_COUNT]; //motor chain array, indexed in motor_user_configs[] order

// Protocol-level RX diagnostics (updated from can_link_on_frame, ISR context).
volatile uint8_t can_rx_flag = 0;
volatile uint32_t can_feedback_count = 0;
volatile uint8_t can_feedback_motor_id = 0;
volatile uint32_t can_type0_count = 0;
volatile uint32_t can_type2_count = 0;
volatile uint32_t can_type17_count = 0;
volatile uint32_t can_unknown_type_count = 0;
volatile uint32_t can_no_lut_count = 0;
volatile uint32_t can_unpack_error_count = 0;
volatile uint8_t can_last_type = 0;
volatile uint8_t can_last_sender_id = 0;

// ---------------------------------------------------------
// Helper: Get Motor Pointer by ID using LUT
// ---------------------------------------------------------
static motor_t* get_motor_by_id(uint8_t id)
{
    for (int i = 0; i < MAX_MOTOR_COUNT; i++) {
        if (motor_user_configs[i].id.can_id == id) {
            return &motors[i];
        }
    }
    return NULL; // Return NULL if ID is not in the LUT
}

// ---------------------------------------------------------
// Cross-context access for main-loop code (see motor_chain.h)
// ---------------------------------------------------------

// Coherent snapshot: the ONLY read path into motors[] from outside the ISR.
// The masked region is exactly the struct copy — no logic inside — so a
// multi-field read can never be split across two CAN feedback cycles.
void motor_get_snapshot(uint8_t idx, motor_t *out)
{
    if (idx >= MAX_MOTOR_COUNT || out == NULL) {
        return;
    }
    __disable_irq();
    *out = motors[idx];
    __enable_irq();
}

// Write path for the fault-word sentinel/clear. A single aligned 32-bit store
// is atomic w.r.t. the CAN ISR (which also writes this field), so no masking.
void motor_set_fault_word(uint8_t idx, uint32_t val)
{
    if (idx < MAX_MOTOR_COUNT) {
        motors[idx].fault_word = val;
    }
}

// Bind CAN ids before the bus starts so the RX ISR's lookup can match feedback.
void motor_chain_bind_ids(void)
{
    for (uint8_t i = 0; i < MAX_MOTOR_COUNT; i++) {
        motors[i].id        = motor_user_configs[i].id.can_id;
        motors[i].master_id = CAN_MASTER_ID;
    }
}

/* Dispatch one received RobStride frame (called by can_link's FIFO0 callback,
   ISR context): comm type → LUT lookup → unpack into motors[]. */
void can_link_on_frame(uint32_t ext_id, uint8_t data[8])
{
    uint8_t comm_type = rs_extid_mode(ext_id); // Bits 24-28
    uint8_t sender_id = (uint8_t)(rs_extid_data(ext_id) & 0xFF);
    can_last_type = comm_type;
    can_last_sender_id = sender_id;

    // Use the LUT helper instead of checking sender_id <= MAX_MOTOR_COUNT
    motor_t *target_motor = get_motor_by_id(sender_id);

    if (target_motor == NULL) {
        can_no_lut_count++;
    }

    if (target_motor != NULL)
    {
        switch (comm_type)
        {
            case 2: // Motor Feedback
                can_type2_count++;
                if (rs_decode_feedback(ext_id, data,
                                       motor_rs_config_by_id(sender_id), target_motor)) {
                    target_motor->last_fb_ms = HAL_GetTick();  // per-motor fb_age source
                    target_motor->fb_count++;                  // per-motor fresh-feedback tick
                    can_feedback_motor_id = sender_id;
                    can_feedback_count++;
                    can_rx_flag = 1;
                } else {
                    can_unpack_error_count++;
                }
                break;

            case 17: // Single Parameter Read reply
            {
                can_type17_count++;
                // Reply carries the register index (bytes 0-1) and value (bytes 4-7).
                uint16_t idx = (uint16_t)(data[0] | (data[1] << 8));
                uint32_t raw = (uint32_t)data[4] |
                               ((uint32_t)data[5] << 8) |
                               ((uint32_t)data[6] << 16) |
                               ((uint32_t)data[7] << 24);
                if (idx == 0x3022u) {
                    target_motor->fault_word = raw;  // latch fault register
                }
                can_rx_flag = 1;
                break;
            }

            case 0: // Get ID Response
                can_type0_count++;
                if (rs_decode_get_id(ext_id, data, target_motor)) {
                    can_rx_flag = 1;
                } else {
                    can_unpack_error_count++;
                }
                break;

            default:
                // Handle unknown types or other responses (e.g. Type 1 response is Type 2)
                can_unknown_type_count++;
                break;
        }

    }
}
