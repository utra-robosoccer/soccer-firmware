#ifndef MOTOR_RUNTIME_H
#define MOTOR_RUNTIME_H

#include "stm32f4xx_hal.h"
#include "proto_common.h"   /* protocol.h wire structs + generated motor_config.h */
#include "cmd_seq_track.h"  /* CmdSeqTrack — reply-window pairing for last_applied_seq */
#include <stdint.h>

/* Loop period, watchdog and timeout constants are GENERATED into motor_config.h
   (derived from configs/<setup>/system.yaml rates); see proto_common.h. */

/* Per-motor runtime state. `state` is a MotorLifecycle (LIFE_*). */
typedef struct {
    const MotorConfig *cfg;
    uint8_t            state;         /* MotorLifecycle (LIFE_*)                  */
    float              pos;           /* wrapped to [-pi, pi] (home frame)        */
    float              vel;
    float              tau;
    float              temp;
    uint8_t            motor_mode;    /* RS Type-2 status (0 reset/1 cal/2 normal) */
    float              hold_pos;      /* home-frame setpoint                      */
    float              hold_vel;      /* velocity feedforward                     */
    float              cmd_kp;        /* gains for the active armed mode          */
    float              cmd_kd;
    float              pos_offset;    /* raw - wrapped (the 2*pi multiple adopted) */
    uint8_t            motor_fault;   /* packed Type-2 fault bits                 */
    uint32_t           fault_word;    /* telemetry mirror of the latched 0x3022   */
    uint32_t           last_fb_ms;    /* snapshot's last-Type-2 stamp, for fb_age */
    uint32_t           armed_ms;      /* when arm_enable ran — CAN-timeout grace baseline */
    uint8_t            mon_verdict;   /* latest EnableMonVerdict (set on feedback, read by service) */
    uint8_t            cause;         /* MotorFaultCause — LATCHED on trip         */
    uint8_t            last_apply_clamp; /* clamp bits from the most recent apply  */
    uint8_t            cmd_flags_latched; /* clamp|stale bits assembled each tick for tele */
    uint8_t            got_fresh_cmd; /* a fresh MIT setpoint arrived this tick    */
    uint8_t            req_rejected;  /* last mode request was illegal (sticky to next tele) */
    uint8_t            to_zero_arrived; /* TO_ZERO has settled at home            */
    uint8_t            saturated;     /* a commanded fixed-point field saturated   */
    uint32_t           watchdog_ms;
    float              zero_best_abs;    /* TO_ZERO: smallest |pos| reached          */
    uint32_t           zero_progress_ms; /* TO_ZERO: last tick |pos| improved        */
    uint16_t           zero_settle;      /* TO_ZERO: consecutive in-tolerance ticks  */
    uint8_t            alive;
    /* Enable monitor: fault if an armed motor reports not-running for K frames. */
    uint8_t            mon_not_enabled;
    uint32_t           mon_prev_fb_count;
    uint16_t           target_cmd_seq;   /* host cmd_seq of the current MIT target */
    CmdSeqTrack        cmd_track;        /* reply-window pairing → last_applied_seq */
} MotorRuntime;

extern MotorRuntime motors_rt[N_MOTORS];

/* Lifecycle */
void motor_runtime_init(void);

/* Feedback-driven: call when a fresh CAN Type-2 arrives. Mirrors motor state, steps
   the enable monitor, and pairs cmd_seq (last_applied) — so telemetry staged right
   after carries the freshest state + this cycle's confirmation. */
void motor_runtime_on_feedback(uint32_t now_ms);

/* Service (send): call once per master cycle (forward on a valid exchange, else the
   fallback tick). Runs the fault detectors on the mirrored state and drives CAN output
   for the current per-motor lifecycle. */
void motor_runtime_update(uint32_t now_ms);

/* Apply one motor's level-triggered mode request + targets for this cycle (the
   mode-request state machine). `cmd_seq` is the robot-level command sequence from
   the SPI header (stamped onto MIT targets for the last_applied echo). Honoured
   only for CMD_FLAG_VALID slots; an illegal transition leaves the state unchanged
   and sets the request_rejected telemetry flag. HOLD is the only arm-from-IDLE
   transition, and it is the ONLY place the motor is enabled. */
void motor_runtime_apply_cmd(uint8_t idx, const cmd_motor_t *m, uint16_t cmd_seq);

/* Refresh the master-link watchdog (any valid SPI frame proves the link alive). */
void motor_runtime_refresh_watchdog(uint8_t idx);

/* Project one motor's live state into the wire tele_motor_t (fixed-point encoded). */
void motor_runtime_sample(tele_motor_t *out, uint8_t idx);

/* Aggregate alive bitmask (bit i = motor i alive) */
uint8_t motor_runtime_motors_alive(void);

#endif /* MOTOR_RUNTIME_H */
