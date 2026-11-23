#include "can_rx.h"
#include "robstride.h"   /* vendor-protocol frames: rs_decode_* (wire-level RX is can_link.c) */
#include <string.h>

#define PI 3.1415926f

/* Private to this TU — owned by the CAN RX ISR. Main-loop code reads it only
   through motor_get_snapshot() and writes only via motor_set_fault_word(). */
static motor_t motors[MAX_MOTOR_COUNT]; // last-feedback cache, indexed in motor_user_configs[] order

/* CAN-id-sweep bitmap: bit per id that answered a Type-0 probe. Owned by the
   RX ISR (marked on every Type-0 reply, roster or not); main-loop code reads
   it only through can_rx_sweep_snapshot(). */
static uint8_t sweep_seen[32];

/* zero_sta (reg 0x7029) boot-check capture: raw little-endian value + seen bit
   per motor slot. Written by the RX ISR on Type-17 replies; main-loop code
   reads it only through can_rx_zero_sta(). */
static volatile uint32_t zero_sta_raw[MAX_MOTOR_COUNT];
static volatile uint8_t  zero_sta_seen;

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
// Cross-context access for main-loop code (see can_rx.h)
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

// Coherent copy of the id-sweep bitmap, same masking rule as motor_get_snapshot.
void can_rx_sweep_snapshot(uint8_t out[32])
{
    __disable_irq();
    memcpy(out, sweep_seen, sizeof(sweep_seen));
    __enable_irq();
}

// zero_sta boot capture: returns 1 once motor idx's 0x7029 reply arrived and
// writes its raw value. Single aligned loads, and the ISR stores the value
// before the seen bit — no masking needed.
uint8_t can_rx_zero_sta(uint8_t idx, uint32_t *raw_out)
{
    if (idx >= MAX_MOTOR_COUNT || !(zero_sta_seen & (uint8_t)(1u << idx))) {
        return 0u;
    }
    *raw_out = zero_sta_raw[idx];
    return 1u;
}

// Bind CAN ids before the bus starts so the RX ISR's lookup can match feedback.
void can_rx_bind_ids(void)
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

    /* Type-0 replies feed the id-sweep bitmap BEFORE the roster lookup: an id
       outside the configured roster answering is exactly what the boot sweep
       must see (it blocks arming). */
    if (comm_type == 0u) {
        can_type0_count++;
        rs_id_sweep_mark(sweep_seen, sender_id);
        if (target_motor != NULL) {
            if (rs_decode_get_id(ext_id, data, target_motor)) {
                can_rx_flag = 1;
            } else {
                can_unpack_error_count++;
            }
        }
        return;
    }

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
                } else if (idx == 0x7029u) {
                    /* zero_sta reply for the boot check (raw first, then the
                       seen bit, so a set bit implies a valid value). */
                    uint8_t slot = (uint8_t)(target_motor - motors);
                    zero_sta_raw[slot] = raw;
                    zero_sta_seen |= (uint8_t)(1u << slot);
                }
                can_rx_flag = 1;
                break;
            }

            default:
                // Handle unknown types or other responses (e.g. Type 1 response is Type 2)
                can_unknown_type_count++;
                break;
        }

    }
}
