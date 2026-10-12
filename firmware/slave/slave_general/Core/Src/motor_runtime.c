#include "motor_runtime.h"
#include "motor_chain.h"
#include "robostride.h"
#include "enable_monitor.h"
#include "../../../../common/include/drive_sm.h"           /* protocol-v9 drive machine */
#include "../../../../common/include/master_watchdog.h"   /* master-loss grace phase */
#include "../../../../common/include/soft_limit.h"         /* one-sided soft-limit clamp */
#include "../../../../common/include/arm_sm.h"              /* ENABLING CAN sequencing */
#include <math.h>
#include <string.h>

#define TWO_PI 6.28318530717958647692f

MotorRuntime motors_rt[N_MOTORS];

/* ── helpers ─────────────────────────────────────────────────────────────── */

/* Wrap an angle into [-pi, pi]. */
static float wrap_pi(float a)
{
    return a - TWO_PI * roundf(a / TWO_PI);
}

/* Wrap/stale-safe elapsed-time test: signed difference so a timestamp stamped
   after `now` was captured reads negative ("not yet"), not huge-unsigned. */
static inline int32_t ms_since(uint32_t now, uint32_t t) { return (int32_t)(now - t); }

/* Send an MIT command whose position is given in the home frame. pos_offset (the
   2*pi multiple the motor adopted at power-up) is added back so the controller
   drives the short path. Opens the cmd_seq reply window for this motor. */
static void send_mit(uint8_t idx, float torque, float home_pos, float vel,
                     float kp, float kd)
{
    can_mit_control_set(motor_configs[idx].can_id, torque,
                        home_pos + motors_rt[idx].pos_offset, vel, kp, kd);
    cmd_seq_on_mit_frame(&motors_rt[idx].cmd_track, motors_rt[idx].target_cmd_seq);
}

/* Reset the enable monitor: clear the not-Run count, seed the fresh-frame
   reference to the motor's current fb_count. Called on each ENABLED entry. */
static void enable_monitor_reset(uint8_t idx)
{
    motor_t s;
    motor_get_snapshot(idx, &s);
    motors_rt[idx].mon_not_enabled   = 0u;
    motors_rt[idx].mon_prev_fb_count = s.fb_count;
}

static uint8_t pack_faults(const motor_t *m)
{
    uint8_t f = 0;
    f |= (uint8_t)m->motor_errors.undervoltage;
    f |= (uint8_t)(m->motor_errors.driver_fault   << 1u);
    f |= (uint8_t)(m->motor_errors.overheat        << 2u);
    f |= (uint8_t)(m->motor_errors.encoder_fault   << 3u);
    f |= (uint8_t)(m->motor_errors.stall_overload  << 4u);
    f |= (uint8_t)(m->motor_errors.uncalibrated    << 5u);
    return f;
}

/* Soft-limit clamp on a commanded (pos, vel, tau_ff). One-sided about the shaft's ACTUAL
   position (soft_clamp_mit): a joint parked outside [lo,hi] can be held or driven back,
   never further out — so enabling/commanding it never fights the shaft into an overtorque.
   Outward feedforward (vel AND tau_ff) past a clamped position is cancelled, |vel| is
   bounded to cfg->max_vel, and |tau_ff| to cfg->max_tau (the overtorque trip threshold).
   Returns TELE_FLAG_* bits. */
static uint8_t apply_soft_clamp(const MotorRuntime *r, float *pos, float *vel, float *tau_ff)
{
    uint8_t c = soft_clamp_mit(pos, vel, tau_ff, r->pos,
                               r->cfg->soft_min, r->cfg->soft_max,
                               r->cfg->max_vel, r->cfg->max_tau);
    uint8_t flags = 0u;
    if (c & SOFT_CLAMPED_POS) flags |= TELE_FLAG_CLAMPED_POS;
    if (c & SOFT_CLAMPED_FF)  flags |= TELE_FLAG_CLAMPED_TAU;
    return flags;
}

/* Clamp wire-sourced gains to this motor's per-model CAN encode range. The CAN encoder
   (can_mit_control_set) would saturate them silently; clamping here keeps the runtime
   state consistent with what is actually sent and reports it (TELE_FLAG_CLAMPED_GAIN).
   Config-default gains (USE_CONFIG_GAINS) bypass this — they are validated at generation. */
static uint8_t clamp_gains(uint8_t can_id, float *kp, float *kd)
{
    const MotorCanRange *rg = motor_can_range_by_id(can_id);
    uint8_t c = 0u;
    c |= soft_clamp_range(kp, rg ? rg->kp_min : KP_MIN, rg ? rg->kp_max : KP_MAX);
    c |= soft_clamp_range(kd, rg ? rg->kd_min : KD_MIN, rg ? rg->kd_max : KD_MAX);
    return c ? (uint8_t)TELE_FLAG_CLAMPED_GAIN : 0u;
}

/* Enter a new drive state, stamping the transient-state clock. */
static void enter_state(uint8_t idx, uint8_t state, uint32_t now_ms)
{
    motors_rt[idx].state    = state;
    motors_rt[idx].state_ms = now_ms;
}

/* ENABLING→ENABLED edge: the limp window starts (or closes) here. If a VALID tuple
   was streamed during ENABLING it is applied THIS cycle; otherwise the motor stays
   limp (zero-gain poll in the per-state service) until the first tuple arrives.
   No hold-at-capture: the host owns the first word (docs/protocol_v9.md). */
static void enter_enabled(uint8_t idx, uint32_t now_ms)
{
    MotorRuntime *r = &motors_rt[idx];
    arm_sm_cancel(&r->arm_phase, &r->arm_wait);  /* Run confirmed — sends are done */
    enable_monitor_reset(idx);
    r->watchdog_ms = now_ms;
    r->armed_ms    = now_ms;   /* CAN-timeout grace: a just-disabled motor has a
                                  stale last_fb; don't trip until it can reply. */
    enter_state(idx, DRV_ENABLED, now_ms);
    if (r->have_tuple) {
        send_mit(idx, r->cmd_tau_ff, r->hold_pos, r->hold_vel, r->cmd_kp, r->cmd_kd);
    }
}

/* A fault was DETECTED: let the pure machine pick the reaction (ramp vs immediate),
   latch first-wins, and apply the actions. */
static void fault_detected(uint8_t idx, uint8_t fault, uint32_t now_ms)
{
    MotorRuntime *r = &motors_rt[idx];
    DriveFaultDecision f = drive_sm_on_fault(r->state, fault,
                                             (uint8_t)(r->fault != DRVF_NONE));
    if (f.latch_fault != DRVF_NONE) r->fault = f.latch_fault;
    if (f.abort_enable) arm_sm_cancel(&r->arm_phase, &r->arm_wait);
    if (f.send_disable) can_disable_motor(motor_configs[idx].can_id, CAN_MASTER_ID);
    if (f.begin_stop) {
        r->hold_vel   = 0.0f;
        r->cmd_tau_ff = 0.0f;
    }
    if (f.next_state != r->state) enter_state(idx, f.next_state, now_ms);
}

/* Discover one motor: ping and wait up to 300 ms for a response. Ends DISABLED
   (present) or ABSENT (silent — the runtime keeps pinging it at ABSENT_PING_MS). */
static void discover(uint8_t idx)
{
    uint8_t cid = motor_configs[idx].can_id;
    motors_rt[idx].state = DRV_DISCOVERING;

    for (uint8_t attempt = 0; attempt < 3u; attempt++) {
        can_rx_flag = 0;
        if (can_get_motor_id(cid, CAN_MASTER_ID) != HAL_OK) {
            HAL_Delay(100u);
            continue;
        }
        uint32_t t = HAL_GetTick();
        while (can_rx_flag == 0 && (HAL_GetTick() - t) < 300u) {}
        if (can_rx_flag) {
            enter_state(idx, DRV_DISABLED, HAL_GetTick());
            HAL_Delay(20u);
            return;
        }
        HAL_Delay(50u);
    }
    enter_state(idx, DRV_ABSENT, HAL_GetTick());
}

/* ENABLING CAN-send pacing (cycles): settle after the run-mode write and after the
   enable. Completion is NOT wait-based — the ENABLING→ENABLED edge fires on the
   first feedback showing rs_mode == Run after the enable went out (or times out
   into DRVF_ENABLE_FAILED), so a fast ACK advances early. */
#define ARM_CLEAR_WAIT_CYCLES  ((uint8_t)((5u  * SLAVE_TICK_HZ + 999u) / 1000u))
#define ARM_SETTLE_WAIT_CYCLES ((uint8_t)((20u * SLAVE_TICK_HZ + 999u) / 1000u))

/* ── public API ──────────────────────────────────────────────────────────── */

void motor_runtime_init(void)
{
    memset(motors_rt, 0, sizeof(motors_rt));
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        motors_rt[i].cfg   = &motor_configs[i];
        motors_rt[i].state = DRV_BOOT;
        motors_rt[i].fault = DRVF_NONE;
    }
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        discover(i);
    }
}

void motor_runtime_apply_cmd(uint8_t idx, const cmd_motor_t *m, uint16_t cmd_seq)
{
    if (idx >= N_MOTORS || m == NULL) return;

    MotorRuntime *r = &motors_rt[idx];
    uint8_t cid = motor_configs[idx].can_id;
    uint32_t now_ms = HAL_GetTick();

    /* Wound guard is consulted only on a DISABLED→ENABLING transition; compute it
       from a fresh snapshot so the pure SM can decide. */
    uint8_t wound = 0u;
    if (m->drive_req == DRV_REQ_ENABLE && r->state == DRV_DISABLED) {
        motor_t snap;
        motor_get_snapshot(idx, &snap);
        float off = snap.pos - wrap_pi(snap.pos);
        wound = (off > MOTOR_WOUND_OFFSET_MAX || off < -MOTOR_WOUND_OFFSET_MAX) ? 1u : 0u;
    }

    DriveReqDecision d = drive_sm_on_request(r->state, m->drive_req, wound,
                                             (uint8_t)(r->fault != DRVF_NONE));
    r->req_rejected = d.rejected;

    if (d.abort_enable) arm_sm_cancel(&r->arm_phase, &r->arm_wait);
    if (d.send_disable) can_disable_motor(cid, CAN_MASTER_ID);
    if (d.send_clear) {
        /* CLEARING entry: the wire-level Type-4 Byte0=1 both stops the motor and
           clears the RS driver's latched fault; reset our mirrors with it. The
           judgement (clean reply / still dirty / silent) runs in the service. */
        can_clear_fault(cid, CAN_MASTER_ID);
        motor_set_fault_word(idx, 0u);
        r->fault_word = 0u;
    }
    if (d.clear_fault) r->fault = DRVF_NONE;
    if (d.latch_fault != DRVF_NONE && r->fault == DRVF_NONE) r->fault = d.latch_fault;
    if (d.begin_enable) {
        arm_sm_begin(&r->arm_phase, &r->arm_wait, 0u);  /* no CLEAR phase: CLEARING
                                                           owns fault clears in v9 */
        r->have_tuple     = 0u;
        r->enable_sent_ms = 0u;
    }
    if (d.begin_stop) {
        r->hold_vel   = 0.0f;
        r->cmd_tau_ff = 0.0f;
    }
    if (d.reset_cmd_seq) {
        cmd_seq_reset(&r->cmd_track);
        r->target_cmd_seq = CMD_SEQ_NONE;
    }
    if (d.next_state != r->state) enter_state(idx, d.next_state, now_ms);

    /* The MIT tuple travels independently of the request: applied when ENABLED,
       or latched during ENABLING and applied on the ENABLING→ENABLED edge. */
    if ((m->flags & CMD_FLAG_VALID) &&
        (r->state == DRV_ENABLED || r->state == DRV_ENABLING)) {
        uint8_t use_cfg = (m->flags & CMD_FLAG_USE_CONFIG_GAINS) != 0u;
        r->hold_pos   = proto_i16_to_f(m->pos, PROTO_POS_SCALE);
        r->hold_vel   = proto_i16_to_f(m->vel, PROTO_VEL_SCALE);
        r->cmd_tau_ff = proto_i16_to_f(m->tau_ff, PROTO_TAU_SCALE);
        r->last_apply_clamp = apply_soft_clamp(r, &r->hold_pos, &r->hold_vel, &r->cmd_tau_ff);
        r->cmd_kp = use_cfg ? r->cfg->default_kp : proto_u16_to_f(m->kp, PROTO_KP_SCALE);
        r->cmd_kd = use_cfg ? r->cfg->default_kd : proto_u16_to_f(m->kd, PROTO_KD_SCALE);
        if (!use_cfg) r->last_apply_clamp |= clamp_gains(cid, &r->cmd_kp, &r->cmd_kd);
        r->target_cmd_seq = cmd_seq;
        r->got_fresh_cmd  = 1u;
        r->have_tuple     = 1u;
    }
}

/* Feedback-driven: mirror the latest Type-2 into motors_rt, step the enable monitor,
   and pair cmd_seq (last_applied). Run when fresh feedback arrives, BEFORE telemetry is
   staged, so the staged frame carries the freshest state + this cycle's confirmation. */
void motor_runtime_on_feedback(uint32_t now_ms)
{
    (void)now_ms;
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        motor_t snap;
        motor_get_snapshot(i, &snap);

        float wrapped            = wrap_pi(snap.pos);
        motors_rt[i].pos         = wrapped;
        motors_rt[i].pos_offset  = snap.pos - wrapped;
        motors_rt[i].vel         = snap.rpm;
        motors_rt[i].tau         = snap.torq;
        motors_rt[i].temp        = snap.temperature;
        motors_rt[i].motor_mode  = (uint8_t)snap.status;
        motors_rt[i].motor_fault = pack_faults(&snap);
        motors_rt[i].fault_word  = snap.fault_word;
        motors_rt[i].last_fb_ms  = snap.last_fb_ms;

        uint8_t driving = (motors_rt[i].state == DRV_ENABLED ||
                           motors_rt[i].state == DRV_STOPPING);

        uint8_t mon_fresh = (snap.fb_count != motors_rt[i].mon_prev_fb_count);
        motors_rt[i].mon_verdict = (uint8_t)enable_monitor_step(
            driving, mon_fresh, (uint8_t)(snap.status == RS_MODE_NORMAL),
            MOTOR_ENABLE_MON_K, &motors_rt[i].mon_not_enabled);
        if (mon_fresh) {
            motors_rt[i].mon_prev_fb_count = snap.fb_count;
            cmd_seq_on_reply(&motors_rt[i].cmd_track);   /* pairs this reply with its MIT */
        }
    }
}

void motor_runtime_update(uint32_t now_ms, uint8_t cmd_fresh)
{
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        MotorRuntime *r = &motors_rt[i];
        uint8_t cid = motor_configs[i].can_id;

        /* ── fault detection on the mirrored feedback state (first trip latches).
           CAN-timeout is time-based (feedback absence) so it belongs here in the
           per-cycle service; the armed_ms grace covers the first cycles after an
           enable (a just-disabled motor's last_fb is stale). ─────────────────── */
        uint8_t live = (uint8_t)(r->state == DRV_ENABLED || r->state == DRV_STOPPING);
        if (live &&
            ms_since(now_ms, r->last_fb_ms) >= (int32_t)MOTOR_CAN_FB_TIMEOUT_MS &&
            ms_since(now_ms, r->armed_ms)   >= (int32_t)MOTOR_CAN_FB_TIMEOUT_MS) {
            fault_detected(i, DRVF_CAN_TIMEOUT, now_ms);
        } else if ((live || r->state == DRV_ENABLING) && r->motor_fault != 0u) {
            fault_detected(i, DRVF_MOTOR_REPORTED, now_ms);
            motor_set_fault_word(i, 0xFFFFFFFFu);
            r->fault_word = 0xFFFFFFFFu;
            can_read_single_param(cid, CAN_MASTER_ID, 0x3022u);
        } else if (r->state == DRV_ENABLED &&
                   r->mon_verdict == ENABLE_MON_FAULT_NOT_ENABLED) {
            fault_detected(i, DRVF_NOT_ENABLED, now_ms);
        } else if (r->state == DRV_ENABLED &&
                   fabsf(r->tau) > r->cfg->max_tau) {
            fault_detected(i, DRVF_OVERTORQUE, now_ms);
        } else if (r->state == DRV_DISABLED && MOTOR_FAULT_CHECK_DISABLED &&
                   (r->motor_fault != 0u || r->motor_mode == RS_MODE_CALI)) {
            /* Vendor bits / Cali at rest — masked off by default until bench
               boot-rest data is clean (docs/protocol_v9.md §8). */
            fault_detected(i, DRVF_MOTOR_REPORTED, now_ms);
        }

        /* ── master loss: no fresh ROBOT_CMD this cycle. ENABLING has nothing to
           ramp → immediate fault; ENABLED gets the hold grace (a brief SPI hiccup
           recovers transparently), then the autonomous stop ramp → FAULT. STOPPING
           and CLEARING run to completion on their own clocks. ─────────────────── */
        if (!cmd_fresh) {
            if (r->state == DRV_ENABLING) {
                fault_detected(i, DRVF_MASTER_LOST, now_ms);
            } else if (r->state == DRV_ENABLED) {
                uint32_t since = (uint32_t)ms_since(now_ms, r->watchdog_ms);
                if (master_loss_phase(since, MASTER_LOST_GRACE_MS, MASTER_LOST_DAMP_MS)
                        == MASTER_HOLD_GRACE) {
                    /* Hold the last setpoint with v_des/tau_ff zeroed; a limp motor
                       (no tuple yet) just keeps the zero-gain poll below. */
                    if (r->have_tuple) {
                        send_mit(i, 0.0f, r->hold_pos, 0.0f, r->cmd_kp, r->cmd_kd);
                    } else {
                        can_mit_control_set(cid, 0.0f, r->pos + r->pos_offset,
                                            0.0f, 0.0f, 0.0f);
                    }
                    continue;   /* grace frame sent; skip the normal service */
                }
                fault_detected(i, DRVF_MASTER_LOST, now_ms);  /* → STOPPING ramp */
            }
        }

        /* ── per-state service: one CAN frame per motor per cycle + drive_sm tick. */
        DriveTickInputs in;
        memset(&in, 0, sizeof(in));
        in.fault_latched = (uint8_t)(r->fault != DRVF_NONE);

        switch (r->state) {
        case DRV_ENABLING:
            if (arm_sm_active(r->arm_phase)) {
                uint8_t act = arm_sm_step(&r->arm_phase, &r->arm_wait,
                                          ARM_CLEAR_WAIT_CYCLES, ARM_SETTLE_WAIT_CYCLES);
                if (act == ARM_ACT_SEND_MODE) {
                    can_change_motor_mode(cid, CAN_MASTER_ID, MIT_MODE);
                } else if (act == ARM_ACT_SEND_ENABLE) {
                    can_enable_motor(cid, CAN_MASTER_ID);
                    r->enable_sent_ms = now_ms;
                }
                /* ARM_ACT_COMPLETE: sends exhausted; Run confirmation decides. */
            } else {
                can_read_motor_state(cid);   /* poll for the Run confirmation */
            }
            in.run_seen = (uint8_t)(r->enable_sent_ms != 0u &&
                                    r->motor_mode == RS_MODE_NORMAL &&
                                    (int32_t)(r->last_fb_ms - r->enable_sent_ms) >= 0);
            in.enable_timeout =
                (uint8_t)(ms_since(now_ms, r->state_ms) >= (int32_t)MOTOR_ENABLE_TIMEOUT_MS);
            break;

        case DRV_ENABLED:
            if (r->have_tuple) {
                send_mit(i, r->cmd_tau_ff, r->hold_pos, r->hold_vel,
                         r->cmd_kp, r->cmd_kd);
            } else {
                /* Limp poll: zero gains and zero torque apply no effort but keep
                   the Type-2 feedback, enable monitor and CAN timeout alive while
                   the host has not yet sent its first tuple. */
                can_mit_control_set(cid, 0.0f, r->pos + r->pos_offset,
                                    0.0f, 0.0f, 0.0f);
            }
            break;

        case DRV_STOPPING:
            /* Backdrive-safe damp (Kp=0, config Kd) for the damp dwell, then the
               ramp owns its disable; drive_sm picks DISABLED vs FAULT. */
            if (ms_since(now_ms, r->state_ms) >= (int32_t)MASTER_LOST_DAMP_MS) {
                can_disable_motor(cid, CAN_MASTER_ID);
                in.stop_done = 1u;
            } else {
                send_mit(i, 0.0f, r->pos, 0.0f, 0.0f, r->cfg->default_kd);
            }
            break;

        case DRV_CLEARING:
            can_read_motor_state(cid);       /* provoke the judging Type-2 */
            in.clear_reply = (uint8_t)((int32_t)(r->last_fb_ms - r->state_ms) >= 0);
            in.clear_dirty = (uint8_t)(r->motor_fault != 0u ||
                                       r->motor_mode == RS_MODE_CALI);
            in.clear_timeout =
                (uint8_t)(ms_since(now_ms, r->state_ms) >= (int32_t)MOTOR_CLEAR_TIMEOUT_MS);
            break;

        case DRV_DISABLED:
            can_read_motor_state(cid);       /* keep presence + mirrors fresh */
            in.fb_timeout =
                (uint8_t)(ms_since(now_ms, r->last_fb_ms) >= (int32_t)MOTOR_CAN_FB_TIMEOUT_MS);
            break;

        case DRV_ABSENT:
            /* state_ms doubles as the ping clock while ABSENT. */
            if (ms_since(now_ms, r->state_ms) >= (int32_t)MOTOR_ABSENT_PING_MS) {
                can_read_motor_state(cid);
                r->state_ms = now_ms;
            }
            in.fb_fresh = (uint8_t)(ms_since(now_ms, r->last_fb_ms) >= 0 &&
                                    ms_since(now_ms, r->last_fb_ms) <
                                        (int32_t)MOTOR_CAN_FB_TIMEOUT_MS);
            break;

        default:   /* DRV_BOOT / DRV_DISCOVERING / DRV_FAULT: nothing to service */
            break;
        }

        /* Step the pure machine on the summarised inputs. */
        {
            DriveTickDecision t = drive_sm_on_tick(r->state, &in);
            if (t.latch_fault != DRVF_NONE && r->fault == DRVF_NONE)
                r->fault = t.latch_fault;
            if (t.send_disable) can_disable_motor(cid, CAN_MASTER_ID);
            if (t.enter_enabled) {
                enter_enabled(i, now_ms);
            } else if (t.next_state != r->state) {
                enter_state(i, t.next_state, now_ms);
            }
        }

        /* Assemble the per-tick telemetry cmd flags (clamp while fresh, else stale). */
        {
            uint8_t cf = 0u;
            if (r->state == DRV_ENABLED && r->have_tuple) {
                cf = r->got_fresh_cmd ? r->last_apply_clamp
                                      : (uint8_t)TELE_FLAG_CMD_STALE;
            }
            r->got_fresh_cmd     = 0u;
            r->cmd_flags_latched = cf;
        }
    }
}

void motor_runtime_refresh_watchdog(uint8_t idx)
{
    if (idx < N_MOTORS) {
        motors_rt[idx].watchdog_ms = HAL_GetTick();
    }
}

void motor_runtime_sample(tele_motor_t *out, uint8_t idx)
{
    if (idx >= N_MOTORS || out == NULL) return;
    const MotorRuntime *r = &motors_rt[idx];

    uint8_t sat = 0u;
    out->pos = proto_f_to_i16(r->pos, PROTO_POS_SCALE, &sat);
    out->vel = proto_f_to_i16(r->vel, PROTO_VEL_SCALE, &sat);
    out->tau = proto_f_to_i16(r->tau, PROTO_TAU_SCALE, &sat);
    out->temp_c = (r->temp < 0.0f) ? 0u : (uint8_t)r->temp;
    out->drive_state   = r->state;
    out->drive_fault   = r->fault;
    out->rs_mode       = r->motor_mode;
    out->rs_fault_bits = r->motor_fault;

    uint8_t flags = r->cmd_flags_latched;
    if (r->req_rejected) flags |= TELE_FLAG_REQUEST_REJECTED;
    if (sat)             flags |= TELE_FLAG_SATURATED;
    out->flags = flags;

    uint32_t age = HAL_GetTick() - r->last_fb_ms;
    out->fb_age_ms = (age > 255u) ? 255u : (uint8_t)age;
    out->rs_fault_word    = r->fault_word;
    out->last_applied_seq = r->cmd_track.last_applied;
    out->reserved         = 0u;
}

uint8_t motor_runtime_motors_alive(void)
{
    uint8_t mask = 0u;
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        if (motors_rt[i].state > DRV_ABSENT) mask |= (uint8_t)(1u << i);
    }
    return mask;
}
