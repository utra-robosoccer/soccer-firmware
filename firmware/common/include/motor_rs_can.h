/* motor_rs_can — the RobStride private CAN protocol, pure and host-testable:
 * the 29-bit ext-id layout, the RsFrame value type, every frame type's encoder
 * and every reply's decoder. No HAL — transport is the slave's can_link.
 * Tested by host/tests/test_motor_rs_can.py (golden vectors shared in spirit
 * with the Python codec in tools/robstride_usb_can/rs02_can.py).
 *
 * Ext-id layout:
 *   bits  0- 7 : motor id          (Tx target id / Rx reporting id)
 *   bits  8-23 : 16-bit data field  (master id, packed torque, or Rx status word)
 *   bits 24-28 : communication type ("mode", 5 bits)
 *   bits 29-31 : reserved (0)
 *
 * Built and parsed with explicit shifts + masks. This replaces the previous
 * `*(exCanIdInfo*)&header.ExtId` pointer cast, which (a) violated strict aliasing
 * — undefined at -O2, where the compiler may drop the field stores and corrupt
 * the arbitration id — and (b) relied on the implementation-defined ordering of
 * C bitfields. */
#ifndef MOTOR_RS_CAN_H
#define MOTOR_RS_CAN_H

#include <stdint.h>
#include <string.h>

#include "motor_rs_config.h"   /* MotorRsConfig (tier-1 vendor ranges), MOTOR_RS_P_MIN/MAX */
#include "motor_types.h"       /* motor_t, motor_status_t, rs_runmode_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ── Ext-id pack/parse ─────────────────────────────────────────────────────── */
static inline uint32_t rs_extid_pack(uint8_t mode, uint16_t data, uint8_t id)
{
    return ((uint32_t)(mode & 0x1Fu) << 24) |
           ((uint32_t)data           << 8)  |
            (uint32_t)id;
}

static inline uint8_t  rs_extid_mode(uint32_t ext) { return (uint8_t)((ext >> 24) & 0x1Fu); }
static inline uint16_t rs_extid_data(uint32_t ext) { return (uint16_t)((ext >> 8)  & 0xFFFFu); }
static inline uint8_t  rs_extid_id(uint32_t ext)   { return (uint8_t)( ext         & 0xFFu); }

/* ── Frame value: what encoders return and the transport sends ─────────────── */
typedef struct {
    uint32_t ext_id;
    uint8_t  data[8];
} RsFrame;

/* ── Feedback (Type 2) status-word layout, in the ext-id data field ────────── */
// CAN Bits 8-15:  Motor ID -> .data bits 0-7
// CAN Bits 16-21: Faults   -> .data bits 8-13
// CAN Bits 22-23: Mode     -> .data bits 14-15
#define RS_FB_DATA_ID_MASK      0x00FF
#define RS_FB_DATA_FAULT_MASK   0x3F00
#define RS_FB_DATA_MODE_MASK    0xC000

// Fault bit offsets (relative to the shifted fault byte)
#define FAULT_BIT_UNDERVOLTAGE    0 // Bit 16 in CAN
#define FAULT_BIT_DRIVER_FAULT    1 // Bit 17 in CAN
#define FAULT_BIT_OVERHEAT        2 // Bit 18 in CAN
#define FAULT_BIT_ENCODER_FAULT   3 // Bit 19 in CAN
#define FAULT_BIT_STALL_OVERLOAD  4 // Bit 20 in CAN
#define FAULT_BIT_UNCALIBRATED    5 // Bit 21 in CAN

/* ── Scaling helpers (vendor Type-1/Type-2 linear u16 mapping) ─────────────── */
static inline uint16_t rs_float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * (float)((1U << bits) - 1U) / span);
}

static inline float rs_uint_to_float(uint32_t x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1U << bits) - 1U)) + offset;
}

/* Ranges used when the caller has no MotorRsConfig for an id (an unconfigured
 * motor). Mirrors the historical robstride.h constants (RS02 values). */
static inline const MotorRsConfig *rs_default_ranges(void)
{
    static const MotorRsConfig d = {
        .v_min = -44.0f, .v_max = 44.0f,
        .t_min = -17.0f, .t_max = 17.0f,
        .kp_min = 0.0f,  .kp_max = 500.0f,
        .kd_min = 0.0f,  .kd_max = 5.0f,
    };
    return &d;
}

/* ── Encoders — one per frame type we send ─────────────────────────────────── */

/* Type 0: Get Device ID (reply carries the 64-bit MCU UID). */
static inline RsFrame rs_get_id_encode(uint8_t id, uint16_t master_id)
{
    RsFrame f = { rs_extid_pack(0, master_id, id), {0} };
    return f;
}

/* Type 1: MIT control. Velocity, torque AND Kp/Kd scale over the per-MODEL
 * vendor ranges (tier 1); only position is shared across models. The packed
 * torque rides in the ext-id data field. rg == NULL → rs_default_ranges(). */
static inline RsFrame rs_mit_encode(const MotorRsConfig *rg, uint8_t id,
                                    float torque, float pos, float vel,
                                    float kp, float kd)
{
    if (rg == NULL) rg = rs_default_ranges();
    RsFrame f;
    f.ext_id = rs_extid_pack(1, rs_float_to_uint(torque, rg->t_min, rg->t_max, 16), id);
    uint16_t p = rs_float_to_uint(pos, MOTOR_RS_P_MIN, MOTOR_RS_P_MAX, 16);
    uint16_t v = rs_float_to_uint(vel, rg->v_min, rg->v_max, 16);
    uint16_t gp = rs_float_to_uint(kp, rg->kp_min, rg->kp_max, 16);
    uint16_t gd = rs_float_to_uint(kd, rg->kd_min, rg->kd_max, 16);
    f.data[0] = (uint8_t)(p >> 8);  f.data[1] = (uint8_t)(p & 0xFF);
    f.data[2] = (uint8_t)(v >> 8);  f.data[3] = (uint8_t)(v & 0xFF);
    f.data[4] = (uint8_t)(gp >> 8); f.data[5] = (uint8_t)(gp & 0xFF);
    f.data[6] = (uint8_t)(gd >> 8); f.data[7] = (uint8_t)(gd & 0xFF);
    return f;
}

/* Type 3: Enable. */
static inline RsFrame rs_enable_encode(uint8_t id, uint16_t master_id)
{
    RsFrame f = { rs_extid_pack(3, master_id, id), {0} };
    return f;
}

/* Type 4: Disable (stop). */
static inline RsFrame rs_disable_encode(uint8_t id, uint16_t master_id)
{
    RsFrame f = { rs_extid_pack(4, master_id, id), {0} };
    return f;
}

/* Type 4 variant: clear latched fault — stop frame with Byte0 = 1. The motor
 * latches its own faults; a Type-3 enable is refused until this clears them. */
static inline RsFrame rs_clear_fault_encode(uint8_t id, uint16_t master_id)
{
    RsFrame f = { rs_extid_pack(4, master_id, id), {0} };
    f.data[0] = 0x1;
    return f;
}

/* Type 6: Set mechanical zero. */
static inline RsFrame rs_mech_zero_encode(uint8_t id, uint16_t master_id)
{
    RsFrame f = { rs_extid_pack(0x6, master_id, id), {0} };
    f.data[0] = 0x1;
    return f;
}

/* Type 17: Read single parameter (register index little-endian in bytes 0-1). */
static inline RsFrame rs_param_read_encode(uint8_t id, uint16_t master_id, uint16_t index)
{
    RsFrame f = { rs_extid_pack(0x11, master_id, id), {0} };
    f.data[0] = (uint8_t)(index & 0xFF);
    f.data[1] = (uint8_t)((index >> 8) & 0xFF);
    return f;
}

/* Type 18: Write the run-mode register (0x7005). */
static inline RsFrame rs_runmode_encode(uint8_t id, uint16_t master_id, rs_runmode_t runmode)
{
    const uint16_t reg_idx = 0x7005;
    RsFrame f = { rs_extid_pack(0x12, master_id, id), {0} };
    f.data[0] = (uint8_t)(reg_idx & 0xFF);
    f.data[1] = (uint8_t)(reg_idx >> 8);
    f.data[4] = (uint8_t)(runmode & 0xFF);
    return f;
}

/* ── Decoders — one per reply we parse. Return 1 on success, 0 on reject. ──── */

/* Type 0 reply: 64-bit MCU UID (little-endian payload); the reporting id rides
 * in the ext-id data field and corrects motor->id if they disagree. */
static inline uint8_t rs_decode_get_id(uint32_t ext_id, const uint8_t data[8], motor_t *motor)
{
    if (rs_extid_mode(ext_id) != 0) return 0;

    uint8_t reported_id = (uint8_t)(rs_extid_data(ext_id) & 0xFF);
    if (reported_id != motor->id) {
        motor->id = reported_id;
    }

    uint64_t uid = 0;
    for (int i = 0; i < 8; i++) {
        uid |= ((uint64_t)data[i] << (8 * i));
    }
    motor->mcu_id = uid;
    return 1;
}

/* Type 2 feedback: status word in the ext-id data field (id / fault bits /
 * mode), payload = pos, vel, torque, temperature — high byte first (datasheet
 * §643). Rejects a frame whose reporting id is not motor->id. rg == NULL →
 * rs_default_ranges(). */
static inline uint8_t rs_decode_feedback(uint32_t ext_id, const uint8_t data[8],
                                         const MotorRsConfig *rg, motor_t *motor)
{
    if (rs_extid_mode(ext_id) != 2) return 0;

    uint16_t data_field = rs_extid_data(ext_id);
    uint8_t feedback_id = data_field & RS_FB_DATA_ID_MASK;
    if (feedback_id != motor->id) {
        return 0;
    }

    motor->status = (motor_status_t)((data_field & RS_FB_DATA_MODE_MASK) >> 14);

    uint8_t fault_byte = (uint8_t)((data_field & RS_FB_DATA_FAULT_MASK) >> 8);
    motor->motor_errors.undervoltage   = (fault_byte >> FAULT_BIT_UNDERVOLTAGE)   & 0x01;
    motor->motor_errors.driver_fault   = (fault_byte >> FAULT_BIT_DRIVER_FAULT)   & 0x01;
    motor->motor_errors.overheat       = (fault_byte >> FAULT_BIT_OVERHEAT)       & 0x01;
    motor->motor_errors.encoder_fault  = (fault_byte >> FAULT_BIT_ENCODER_FAULT)  & 0x01;
    motor->motor_errors.stall_overload = (fault_byte >> FAULT_BIT_STALL_OVERLOAD) & 0x01;
    motor->motor_errors.uncalibrated   = (fault_byte >> FAULT_BIT_UNCALIBRATED)   & 0x01;

    if (rg == NULL) rg = rs_default_ranges();
    motor->pos  = rs_uint_to_float((uint32_t)(data[0] << 8 | data[1]),
                                   MOTOR_RS_P_MIN, MOTOR_RS_P_MAX, 16);
    motor->rpm  = rs_uint_to_float((uint32_t)(data[2] << 8 | data[3]),
                                   rg->v_min, rg->v_max, 16);
    motor->torq = rs_uint_to_float((uint32_t)(data[4] << 8 | data[5]),
                                   rg->t_min, rg->t_max, 16);
    motor->temperature = (float)(data[6] << 8 | data[7]) / 10.0f;
    return 1;
}

/* Type 17 reply: value in bytes 4-7 (little-endian), viewed as a float. */
static inline uint8_t rs_decode_single_param(uint32_t ext_id, const uint8_t data[8], float *result_val)
{
    if (rs_extid_mode(ext_id) != 0x11) return 0;

    uint32_t raw_val = 0;
    raw_val |= data[4];
    raw_val |= (uint32_t)data[5] << 8;
    raw_val |= (uint32_t)data[6] << 16;
    raw_val |= (uint32_t)data[7] << 24;

    memcpy(result_val, &raw_val, sizeof(float));
    return 1;
}

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_RS_CAN_H */
