#ifndef MOTOR_RUNTIME_H
#define MOTOR_RUNTIME_H

#include "stm32f4xx_hal.h"
#include "proto_common.h"   /* protocol.h wire structs + generated motor_user_config.h */
#include "cmd_seq_track.h"  /* CmdSeqTrack — reply-window pairing for last_applied_seq */
#include <stdint.h>

/* Loop period, watchdog and timeout constants are GENERATED into motor_user_config.h
   (derived from configs/<setup>/system.yaml rates); see proto_common.h. */

/* Per-motor runtime state. `state` is a MotorState, `fault` a MotorFault
   (latched at detection, first wins; cleared only by the slave-level reset). */
typedef struct {
    const MotorUserConfig *cfg;
    uint8_t            state;         /* MotorState                               */
    uint8_t            fault;         /* MotorFault — latched at detection        */
    float              pos;           /* single-turn frame, zero_sta-verified     */
    float              vel;
    float              tau;
    float              temp;
    uint8_t            motor_mode;    /* RS Type-2 status (0 reset/1 cal/2 run)   */
    float              hold_pos;      /* mit of the current command (decoded)     */
    float              hold_vel;
    float              cmd_kp;
    float              cmd_kd;
    float              cmd_tau_ff;    /* clamped to ±max_tau                      */
    uint8_t            motor_fault;   /* packed Type-2 fault bits (rs fault_bits) */
    uint32_t           fault_word;    /* telemetry mirror of the latched 0x3022   */
    uint32_t           last_fb_ms;    /* snapshot's last-Type-2 stamp, for fb_age */
    uint32_t           last_fb_us;    /* µs stamp of that Type-2 (rs.time_slave_us) */
    uint8_t            fb_seen;       /* a real Type-2 arrived since boot (the
                                         mode/fault mirrors hold defaults until) */
    uint32_t           armed_ms;      /* ENABLED entry — fb-timeout grace baseline */
    uint32_t           state_ms;      /* entry time of the current transient state
                                         (ENABLING / STOPPING)                    */
    uint32_t           enable_sent_ms;/* when ENABLING's Type-3 enable went out    */
    uint8_t            mon_verdict;   /* latest EnableMonVerdict (set on feedback) */
    uint8_t            last_apply_clamp; /* clamp bits from the most recent apply  */
    uint8_t            cmd_flags_latched; /* clamp|stale bits assembled each tick  */
    uint8_t            got_fresh_cmd; /* a fresh command arrived this tick         */
    uint8_t            req_rejected;  /* last request was illegal (sticky)         */
    uint32_t           watchdog_ms;   /* last fresh command (master-loss clock)    */
    /* Enable monitor: fault if an enabled motor reports not-Run for K frames. */
    uint8_t            mon_not_enabled;
    uint32_t           mon_prev_fb_count;
    uint16_t           target_cmd_seq;   /* host cmd_seq of the current mit target */
    CmdSeqTrack        cmd_track;        /* reply-window pairing → last_applied_seq */
    uint8_t            arm_phase;        /* ENABLING CAN sequencing: ARM_PHASE_*    */
    uint8_t            arm_wait;         /* cycles left before the next phase send  */
} MotorRuntime;

extern MotorRuntime motors_rt[N_MOTORS];

/* Lifecycle: the blocking boot checks (paced id sweep, zero_sta, fw version). */
void motor_runtime_init(void);

/* Feedback-driven: call when a fresh CAN Type-2 arrives. Mirrors motor state, steps
   the enable monitor, and pairs cmd_seq (last_applied) — so telemetry staged right
   after carries the freshest state + this cycle's confirmation. */
void motor_runtime_on_feedback(uint32_t now_ms);

/* Service (send): call once per master cycle. cmd_fresh=1 on a forward service (a
   fresh host command was applied this cycle); cmd_fresh=0 on a NOP or the fallback
   tick. Runs fault detection, the master-loss reaction, and the per-state service
   (drive_sm ticks + CAN sends). */
void motor_runtime_update(uint32_t now_ms, uint8_t cmd_fresh);

/* Apply one motor's request + mit for this cycle. The request byte is
   level-triggered and always applies; the mit is decoded and applied every cycle
   while ENABLED (all-zero = limp), or latched during ENABLING and applied on the
   ENABLING→ENABLED edge. An illegal request leaves the state unchanged and sets
   the MOTOR_FLAG_REQ_REJECTED telemetry flag. */
void motor_runtime_apply_cmd(uint8_t idx, const cmd_motor_t *m, uint16_t cmd_seq);

/* Refresh the master-link watchdog (a fresh command proves the link alive). */
void motor_runtime_refresh_watchdog(uint8_t idx);

/* Project one motor's live state into the wire tele_motor_t (fixed-point encoded). */
void motor_runtime_sample(tele_motor_t *out, uint8_t idx);

/* Aggregate alive bitmask (bit i = motor i present: state past ABSENT). */
uint8_t motor_runtime_motors_alive(void);

/* Boot sweep + verify verdict for telemetry (status_slave_t.boot). */
void motor_runtime_boot_result(boot_result_t *out);

/* 1 = the id sweep, zero_sta and fw-version checks all passed (SLAVE_READY). */
uint8_t motor_runtime_boot_ok(void);

/* 1 = any motor is armed (ENABLING/ENABLED/STOPPING) — SLAVE_REQ_RESET is
   refused while this holds. */
uint8_t motor_runtime_any_armed(void);

/* One BLOCKING reset round (SLAVE_REQ_RESET): stop+clear every motor, re-arm
   the boot captures, re-run the sweep + zero_sta + fw verify, re-judge every
   motor (clean → DISABLED fault NONE; still dirty → MOTOR_FAULT; silent →
   ABSENT). Takes seconds; call only with nothing armed and the RESETTING
   telemetry frame pre-staged. */
void motor_runtime_reset_round(void);

/* 1 = a further reset round would be a no-op: boot checks pass and every motor
   is DISABLED with clean fault bits. */
uint8_t motor_runtime_reset_clean(void);

#endif /* MOTOR_RUNTIME_H */
