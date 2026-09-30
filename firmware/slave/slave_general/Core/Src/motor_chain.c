#include "motor_chain.h"
#include "robostride.h"   /* CAN codec: can_unpack_*, exCanIdInfo, rs_can_rx_header */

#define PI 3.1415926f

/* Private to this TU — owned by the CAN RX ISR. Main-loop code reads it only
   through motor_get_snapshot() and writes only via motor_set_fault_word(). */
static motor_t motors[MAX_MOTOR_COUNT]; //motor chain array, indexed in motor_configs[] order

// For FIFO Intr Callback function
volatile uint8_t can_rx_flag = 0;
volatile uint32_t can_raw_rx_count = 0;
volatile uint32_t can_feedback_count = 0;
volatile uint8_t can_feedback_motor_id = 0;
volatile uint32_t can_type0_count = 0;
volatile uint32_t can_type2_count = 0;
volatile uint32_t can_type17_count = 0;
volatile uint32_t can_unknown_type_count = 0;
volatile uint32_t can_no_lut_count = 0;
volatile uint32_t can_unpack_error_count = 0;
volatile uint32_t can_last_ext_id = 0;
volatile uint32_t can_last_rx_word0 = 0;
volatile uint32_t can_last_rx_word1 = 0;
volatile uint8_t can_last_type = 0;
volatile uint8_t can_last_sender_id = 0;
volatile uint8_t can_last_dlc = 0;
volatile uint8_t can_last_ide = 0;
uint8_t rx_data[8];

// ---------------------------------------------------------
// Helper: Get Motor Pointer by ID using LUT
// ---------------------------------------------------------
static motor_t* get_motor_by_id(uint8_t id)
{
    for (int i = 0; i < MAX_MOTOR_COUNT; i++) {
        if (motor_configs[i].can_id == id) {
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
        motors[i].id        = motor_configs[i].can_id;
        motors[i].master_id = CAN_MASTER_ID;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rs_can_rx_header, rx_data);
    can_raw_rx_count++;
    can_last_ext_id = rs_can_rx_header.ExtId;
    can_last_dlc = rs_can_rx_header.DLC;
    can_last_ide = rs_can_rx_header.IDE;
    can_last_rx_word0 = ((uint32_t)rx_data[0]) |
                        ((uint32_t)rx_data[1] << 8) |
                        ((uint32_t)rx_data[2] << 16) |
                        ((uint32_t)rx_data[3] << 24);
    can_last_rx_word1 = ((uint32_t)rx_data[4]) |
                        ((uint32_t)rx_data[5] << 8) |
                        ((uint32_t)rx_data[6] << 16) |
                        ((uint32_t)rx_data[7] << 24);

    exCanIdInfo *rx_id_info = (exCanIdInfo *)&rs_can_rx_header.ExtId;

    uint8_t comm_type = rx_id_info->mode; // Bits 24-28
    uint8_t sender_id = 0;

    sender_id = (uint8_t)(rx_id_info->data & 0xFF);
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
                if (can_unpack_motor_feedback(target_motor, rx_data) == HAL_OK) {
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
                uint16_t idx = (uint16_t)(rx_data[0] | (rx_data[1] << 8));
                uint32_t raw = (uint32_t)rx_data[4] |
                               ((uint32_t)rx_data[5] << 8) |
                               ((uint32_t)rx_data[6] << 16) |
                               ((uint32_t)rx_data[7] << 24);
                if (idx == 0x3022u) {
                    target_motor->fault_word = raw;  // latch fault register
                }
                can_rx_flag = 1;
                break;
            }

            case 0: // Get ID Response
                can_type0_count++;
                if (can_unpack_get_id(target_motor, rx_data) == HAL_OK) {
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
