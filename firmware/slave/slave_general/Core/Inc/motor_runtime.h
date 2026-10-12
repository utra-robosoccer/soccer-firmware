#ifndef MOTOR_RUNTIME_H
#define MOTOR_RUNTIME_H

#include "stm32f4xx_hal.h"
#include "proto_common.h"   /* protocol.h wire structs + generated motor_config.h */
#include "cmd_seq_track.h"  /* CmdSeqTrack — reply-window pairing for last_applied_seq */
#include <stdint.h>

/* Loop period, watchdog and timeout constants are GENERATED into motor_config.h
   (derived from configs/<setup>/system.yaml rates); see proto_common.h. */

/* Per-motor runtime state. `state` is a DriveState (DRV_*), `fault` a DriveFault
   (DRVF_*, latched at detection, first wins). */
typedef struct {
    const MotorConfig *cfg;
    uint8_t            state;         /* DriveState (DRV_*)                       */
    uint8_t            fault;         /* DriveFault (DRVF_*) — latched at detection */
    float              pos;           /* wrapped to [-pi, pi] (home frame)        */
    float              vel;
    float              tau;
    float              temp;
    uint8_t            motor_mode;    /* RS Type-2 status (0 reset/1 cal/2 run)   */
    float              hold_pos;      /* home-frame setpoint (live tuple)         */
    float              hold_vel;      /* velocity feedforward                     */
    float              cmd_kp;        /* gains of the live tuple                  */
    float              cmd_kd;
    float              cmd_tau_ff;    /* torque feedforward, clamped to ±max_tau  */
    uint8_t            have_tuple;    /* a VALID tuple arrived since ENABLE —
                                         until then ENABLED is limp (zero poll)   */
    float              pos_offset;    /* raw - wrapped (the 2*pi multiple adopted) */
    uint8_t            motor_fault;   /* packed Type-2 fault bits (rs_fault_bits) */
    uint32_t           fault_word;    /* telemetry mirror of the latched 0x3022   */
    uint32_t           last_fb_ms;    /* snapshot's last-Type-2 stamp, for fb_age */
    uint32_t           armed_ms;      /* ENABLED entry — CAN-timeout grace baseline */
    uint32_t           state_ms;      /* entry time of the current transient state
                                         (ENABLING / CLEARING / STOPPING / ABSENT) */
    uint32_t           enable_sent_ms;/* when ENABLING's Type-3 enable went out    */
    uint32_t           clear_fb_base; /* fb_count at CLEARING entry (reply detect) */
    uint8_t            mon_verdict;   /* latest EnableMonVerdict (set on feedback) */
    uint8_t            last_apply_clamp; /* clamp bits from the most recent apply  */
    uint8_t            cmd_flags_latched; /* clamp|stale bits assembled each tick  */
    uint8_t            got_fresh_cmd; /* a fresh MIT tuple arrived this tick       */
    uint8_t            req_rejected;  /* last drive request was illegal (sticky)   */
    uint32_t           watchdog_ms;   /* last fresh ROBOT_CMD (master-loss clock)  */
    /* Enable monitor: fault if an enabled motor reports not-Run for K frames. */
    uint8_t            mon_not_enabled;
    uint32_t           mon_prev_fb_count;
    uint16_t           target_cmd_seq;   /* host cmd_seq of the current MIT target */
    CmdSeqTrack        cmd_track;        /* reply-window pairing → last_applied_seq */
    uint8_t            arm_phase;        /* ENABLING CAN sequencing: ARM_PHASE_*    */
    uint8_t            arm_wait;         /* cycles left before the next phase send  */
} MotorRuntime;

extern MotorRuntime motors_rt[N_MOTORS];

/* Lifecycle */
void motor_runtime_init(void);

/* Feedback-driven: call when a fresh CAN Type-2 arrives. Mirrors motor state, steps
   the enable monitor, and pairs cmd_seq (last_applied) — so telemetry staged right
   after carries the freshest state + this cycle's confirmation. */
void motor_runtime_on_feedback(uint32_t now_ms);

/* Service (send): call once per master cycle. cmd_fresh=1 on a forward service (a
   fresh ROBOT_CMD was applied this cycle); cmd_fresh=0 on the fallback tick. Runs
   fault detection, the master-loss reaction, and the per-state service (drive_sm
   ticks + CAN sends). */
void motor_runtime_update(uint32_t now_ms, uint8_t cmd_fresh);

/* Apply one motor's drive request + MIT tuple for this cycle (protocol v9). The
   request byte is level-triggered and always applies; the tuple applies only under
   CMD_FLAG_VALID and only in ENABLED (or is latched during ENABLING and applied on
   the ENABLING→ENABLED edge). An illegal request leaves the state unchanged and
   sets the request_rejected telemetry flag. */
void motor_runtime_apply_cmd(uint8_t idx, const cmd_motor_t *m, uint16_t cmd_seq);

/* Refresh the master-link watchdog (any valid SPI frame proves the link alive). */
void motor_runtime_refresh_watchdog(uint8_t idx);

/* Project one motor's live state into the wire tele_motor_t (fixed-point encoded). */
void motor_runtime_sample(tele_motor_t *out, uint8_t idx);

/* Aggregate alive bitmask (bit i = motor i present: state past ABSENT). */
uint8_t motor_runtime_motors_alive(void);

#endif /* MOTOR_RUNTIME_H */
