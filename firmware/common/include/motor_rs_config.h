/* RS (vendor) limits — tier 1 of the motor-config tiers.
 *
 * Fixed RobStride per-model CAN "operation control" (Type 1) encode ranges,
 * transcribed from each model's user manual reference code (Product_Information
 * repo, manuals dated 2026-07-13; see docs/robstride-motor-reference.md).
 * These are properties of the motors themselves and NEVER change at runtime or
 * per robot — hence hand-written here, not generated from a robot config.
 * gen_motor_config.py keeps a mirror (RS_CONFIG) and refuses to generate if a
 * robot config disagrees with this table.
 *
 * The three tiers:
 *   1. MotorRsConfig  (this file)   — vendor encode ranges, per MODEL, fixed.
 *   2. MotorUserConfig user limits      — per-MOTOR operating envelope chosen by us
 *      (soft_min/max, max_vel, max_tau, default gains); generated, must fit
 *      inside tier 1.
 *   3. MotorFallback                — per-MOTOR MIT constants the slave applies
 *      autonomously (host/master death damp ramp); generated, must fit inside
 *      tier 1.
 */
#ifndef MOTOR_RS_CONFIG_H
#define MOTOR_RS_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Position encode range is identical across models: ±4π. */
#define MOTOR_RS_P_MIN  -12.57f
#define MOTOR_RS_P_MAX   12.57f

/* All RobStride models this firmware knows. Fixed order — do not reorder
 * (values are baked into generated motor_user_configs[] entries). Extend by
 * appending, with ranges sourced from that model's manual. */
typedef enum {
    MOTOR_MODEL_RS00 = 0u,
    MOTOR_MODEL_RS02 = 1u,
    MOTOR_MODEL_RS03 = 2u,
    MOTOR_MODEL_RS05 = 3u,
    MOTOR_MODEL_RS06 = 4u,
    MOTOR_MODEL_COUNT = 5u
} MotorModel;

/* Expected firmware-version bytes: the 6 bytes a motor returns after the
 * 0x00,0xC4 echo in its reply to the Type-4 version-read variant. Read once
 * from a known-good motor on the bench and recorded here; the slave verifies
 * every motor at boot and a mismatch latches the chain blocked. The exact
 * match is deliberate: every OTA forces an edit here, so fleet firmware can't
 * drift unnoticed — and since each model has its own build, it doubles as a
 * model check. ALL-ZEROS means "not yet read": a motor of that model blocks
 * boot until its bytes are recorded (probe: Type-4 with Byte0=0, Byte1=0xC4). */
#define RS_FW_VERSION_LEN 6u

typedef struct {
    float v_min;    /* rad/s */
    float v_max;
    float t_min;    /* Nm    */
    float t_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    uint8_t fw_version[RS_FW_VERSION_LEN];
} MotorRsConfig;

static const MotorRsConfig motor_rs_configs[MOTOR_MODEL_COUNT] = {
    [MOTOR_MODEL_RS00] = { -33.0f, 33.0f, -14.0f, 14.0f, 0.0f,  500.0f, 0.0f,   5.0f,
                           {0x56, 0x00, 0x00, 0x03, 0x24, 0x07} },  /* bench 2026-11-22 */
    [MOTOR_MODEL_RS02] = { -44.0f, 44.0f, -17.0f, 17.0f, 0.0f,  500.0f, 0.0f,   5.0f,
                           {0x56, 0x00, 0x02, 0x03, 0x24, 0x07} },  /* bench 2026-11-22 */
    [MOTOR_MODEL_RS03] = { -20.0f, 20.0f, -60.0f, 60.0f, 0.0f, 5000.0f, 0.0f, 100.0f,
                           {0} },  /* not yet read — blocks boot until recorded */
    [MOTOR_MODEL_RS05] = { -50.0f, 50.0f,  -5.5f,  5.5f, 0.0f,  500.0f, 0.0f,   5.0f,
                           {0} },  /* not yet read — blocks boot until recorded */
    [MOTOR_MODEL_RS06] = { -50.0f, 50.0f, -36.0f, 36.0f, 0.0f, 5000.0f, 0.0f, 100.0f,
                           {0} },  /* not yet read — blocks boot until recorded */
};

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_RS_CONFIG_H */
