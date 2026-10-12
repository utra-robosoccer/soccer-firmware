#include "motor_runtime.h"
#include "motor_chain.h"
#include "robostride.h"
#include "enable_monitor.h"
#include "mode_sm.h"
#include "../../../../common/include/master_watchdog.h"   /* master-loss ramp (task 6) */
#include "../../../../common/include/soft_limit.h"         /* one-sided soft-limit clamp */
#include "../../../../common/include/arm_sm.h"              /* async-arm phase machine */
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

/* Trip a motor to a LATCHED fault: drop CAN output, LIFE_FAULT, latch the cause.
   Clearing requires an IDLE request (then re-arm) or the fault_reset flag. */
static void fault_to(uint8_t idx, uint8_t cause)
{
    can_disable_motor(motor_configs[idx].can_id, CAN_MASTER_ID);
    motors_rt[idx].state    = LIFE_FAULT;
    motors_rt[idx].hold_vel = 0.0f;
    motors_rt[idx].cause    = cause;
}

/* Reset the enable monitor: clear the not-NORMAL count, seed the fresh-frame
   reference to the motor's current fb_count. Called on each HOLD-arm. */
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
   never further out — so arming/commanding it never fights the shaft into an overtorque.
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

/* Discover one motor: ping and wait up to 300 ms for a response. */
static void discover(uint8_t idx)
{
    uint8_t cid = motor_configs[idx].can_id;
    motors_rt[idx].state = LIFE_DISCOVERING;

    for (uint8_t attempt = 0; attempt < 3u; attempt++) {
        can_rx_flag = 0;
        if (can_get_motor_id(cid, CAN_MASTER_ID) != HAL_OK) {
            HAL_Delay(100u);
            continue;
        }
        uint32_t t = HAL_GetTick();
        while (can_rx_flag == 0 && (HAL_GetTick() - t) < 300u) {}
        if (can_rx_flag) {
            motors_rt[idx].alive = 1u;
            motors_rt[idx].state = LIFE_IDLE;
            HAL_Delay(20u);
            return;
        }
        HAL_Delay(50u);
    }
    motors_rt[idx].alive = 0u;
    motors_rt[idx].state = LIFE_FAULT;
    motors_rt[idx].cause = CAUSE_NONE;
}

/* Async-arm pacing (cycles). Sized to the proven blocking sequence's WORST case:
   ACK wait (≤10 ms) + 10 ms settle after each of mode-write and enable, 5 ms after a
   fault clear. Cheap to tighten later with feedback-ACK-based early advance. */
#define ARM_CLEAR_WAIT_CYCLES  ((uint8_t)((5u  * SLAVE_TICK_HZ + 999u) / 1000u))
#define ARM_SETTLE_WAIT_CYCLES ((uint8_t)((20u * SLAVE_TICK_HZ + 999u) / 1000u))

/* Start the NON-BLOCKING CAN enable handshake (arm_sm.h); the phases are serviced one
   action per cycle by motor_runtime_update's LIFE_IDLE path, and arm_finish enters HOLD.
   The caller (apply_cmd) has already cleared the wound guard via the state machine and
   guarded against restarts (arm_sm_active). Never writes mechanical zero. */
static void arm_begin(uint8_t idx, uint8_t had_fault)
{
    MotorRuntime *r = &motors_rt[idx];
    motor_t snap;
    motor_get_snapshot(idx, &snap);

    /* Reset the latched fault mirrors now; the CLEAR phase sends the CAN fault clear
       (the RS refuses enable otherwise). `had_fault` comes from the caller's pre-reset
       state — apply_cmd zeroes the driver fault_word before this runs, so the snapshot
       alone can no longer tell that a fault was latched. */
    motor_set_fault_word(idx, 0u);
    r->fault_word = 0u;
    r->cause      = CAUSE_NONE;

    arm_sm_begin(&r->arm_phase, &r->arm_wait,
                 (uint8_t)(had_fault || snap.fault_word != 0u));
}

/* Handshake complete: capture the home-frame hold point from the freshest mirrored
   feedback (motor_runtime_on_feedback keeps r->pos/pos_offset current throughout the
   handshake — the mode/enable ACKs are Type-2 frames) and enter HOLD. */
static void arm_finish(uint8_t idx, uint32_t now_ms)
{
    MotorRuntime *r = &motors_rt[idx];

    r->hold_pos   = r->pos;
    r->hold_vel   = 0.0f;
    r->cmd_tau_ff = 0.0f;
    r->cmd_kp     = r->cfg->default_kp;
    r->cmd_kd     = r->cfg->default_kd;

    /* cmd_seq re-baseline (target_cmd_seq = NONE, last_applied → 0) was done centrally
       in apply_cmd from ModeDecision.reset_cmd_seq when the arm was requested. */
    enable_monitor_reset(idx);
    r->to_zero_arrived = 0u;
    r->watchdog_ms = now_ms;
    r->armed_ms    = now_ms;   /* CAN-timeout grace: a just-disabled motor has a
                                  stale last_fb; don't trip until it can reply. */
    r->state       = LIFE_HOLD;
    send_mit(idx, 0.0f, r->hold_pos, 0.0f, r->cmd_kp, r->cmd_kd);
}

/* ── public API ──────────────────────────────────────────────────────────── */

void motor_runtime_init(void)
{
    memset(motors_rt, 0, sizeof(motors_rt));
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        motors_rt[i].cfg   = &motor_configs[i];
        motors_rt[i].state = LIFE_BOOT;
    }
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        discover(i);
    }
}

void motor_runtime_apply_cmd(uint8_t idx, const cmd_motor_t *m, uint16_t cmd_seq)
{
    if (idx >= N_MOTORS || m == NULL) return;
    if (!(m->flags & CMD_FLAG_VALID)) return;   /* no request for this slot */

    MotorRuntime *r = &motors_rt[idx];
    uint8_t cid = motor_configs[idx].can_id;

    /* Wound guard is consulted only on an IDLE→HOLD arm; compute it from a fresh
       snapshot in that case so the pure SM can decide. Checked only when STARTING a
       handshake — a level-triggered HOLD re-request during an in-progress async arm
       must not re-evaluate it (the shaft cannot wind mid-arm; re-tripping would strand
       the phase machine in FAULT). */
    uint8_t wound = 0u;
    if (m->mode_req == REQ_HOLD && r->state == LIFE_IDLE && !arm_sm_active(r->arm_phase)) {
        if (!r->alive) { r->req_rejected = 1u; return; }
        motor_t snap;
        motor_get_snapshot(idx, &snap);
        float off = snap.pos - wrap_pi(snap.pos);
        wound = (off > MOTOR_WOUND_OFFSET_MAX || off < -MOTOR_WOUND_OFFSET_MAX) ? 1u : 0u;
    }

    /* An IDLE request mid-handshake cancels the async arm. Disable is safe at any
       phase (a not-yet-enabled motor ignores it). Other non-HOLD requests are rejected
       by the SM below (state is still IDLE) and leave the handshake running — a host
       that races ahead to MIT gets HOLD on completion, then its MIT is accepted. */
    if (arm_sm_active(r->arm_phase) && m->mode_req == REQ_IDLE) {
        arm_sm_cancel(&r->arm_phase, &r->arm_wait);
        can_disable_motor(cid, CAN_MASTER_ID);
    }

    uint8_t was_fault = (uint8_t)(r->state == LIFE_FAULT);

    ModeDecision d = mode_sm_step(r->state, m->mode_req,
                                  (uint8_t)((m->flags & CMD_FLAG_VALID) != 0u),
                                  (uint8_t)((m->flags & CMD_FLAG_FAULT_RESET) != 0u),
                                  wound);

    r->req_rejected = d.rejected;

    if (d.clear_fault || d.do_disable) {
        motor_set_fault_word(idx, 0u);
        r->fault_word = 0u;
        r->cause      = CAUSE_NONE;
    }
    if (d.set_cause_wound) r->cause = CAUSE_WOUND;
    if (d.do_disable) {
        /* Leaving FAULT: the RS driver's own latched fault must be cleared on the wire
           or it refuses the next enable. Type-4 Byte0=1 both stops and clears, so it
           replaces the plain disable here. */
        if (was_fault) can_clear_fault(cid, CAN_MASTER_ID);
        else           can_disable_motor(cid, CAN_MASTER_ID);
        r->hold_vel        = 0.0f;
        r->to_zero_arrived = 0u;
    } else if (d.clear_fault && !d.do_arm) {
        /* fault_reset that neither disables nor arms (e.g. IDLE+reset when mode_sm has
           already stepped the state to IDLE, or a rejected request after the reset):
           still send the wire-level clear. The arm path sends its own in SEND_CLEAR. */
        can_clear_fault(cid, CAN_MASTER_ID);
    }
    if (d.do_arm && !arm_sm_active(r->arm_phase)) {
        arm_begin(idx, was_fault);       /* start the async handshake (once) */
    }
    if (d.capture_hold) {                /* re-hold without re-enabling */
        r->hold_pos   = r->pos;
        r->hold_vel   = 0.0f;
        r->cmd_tau_ff = 0.0f;
        r->cmd_kp     = r->cfg->default_kp;
        r->cmd_kd     = r->cfg->default_kd;
        r->to_zero_arrived = 0u;
    }
    if (d.enter_to_zero) {               /* init creep on entry */
        r->hold_pos         = r->pos;
        r->hold_vel         = 0.0f;
        r->cmd_tau_ff       = 0.0f;
        r->zero_best_abs    = fabsf(r->pos);
        r->zero_progress_ms = HAL_GetTick();
        r->zero_settle      = 0u;
        r->to_zero_arrived  = 0u;
    }
    /* Central cmd_seq re-baseline: any arm / re-hold / goto-zero / disable starts the next
       command stream from last_applied_seq = 0 (fixes a stale seq surviving across a
       re-hold of an already-armed motor). MIT/DAMPED below then set the live target. */
    if (d.reset_cmd_seq) {
        cmd_seq_reset(&r->cmd_track);
        r->target_cmd_seq = CMD_SEQ_NONE;
    }

    uint8_t use_cfg = (m->flags & CMD_FLAG_USE_CONFIG_GAINS) != 0u;
    if (d.next_state == LIFE_MIT) {
        r->hold_pos   = proto_i16_to_f(m->pos, PROTO_POS_SCALE);
        r->hold_vel   = proto_i16_to_f(m->vel, PROTO_VEL_SCALE);
        r->cmd_tau_ff = proto_i16_to_f(m->tau_ff, PROTO_TAU_SCALE);
        r->last_apply_clamp = apply_soft_clamp(r, &r->hold_pos, &r->hold_vel, &r->cmd_tau_ff);
        r->cmd_kp = use_cfg ? r->cfg->default_kp : proto_u16_to_f(m->kp, PROTO_KP_SCALE);
        r->cmd_kd = use_cfg ? r->cfg->default_kd : proto_u16_to_f(m->kd, PROTO_KD_SCALE);
        if (!use_cfg) r->last_apply_clamp |= clamp_gains(cid, &r->cmd_kp, &r->cmd_kd);
        r->target_cmd_seq  = cmd_seq;
        r->got_fresh_cmd   = 1u;
        r->to_zero_arrived = 0u;
    } else if (d.next_state == LIFE_DAMPED) {
        r->cmd_kp     = 0.0f;            /* backdrive-safe: no position hold */
        r->cmd_kd     = use_cfg ? r->cfg->default_kd : proto_u16_to_f(m->kd, PROTO_KD_SCALE);
        if (!use_cfg) (void)clamp_gains(cid, &r->cmd_kp, &r->cmd_kd);
        r->hold_vel   = 0.0f;
        r->cmd_tau_ff = 0.0f;
        r->to_zero_arrived = 0u;
    }

    /* The arm decision's next_state is HOLD, but the async handshake has only just
       started — the motor stays IDLE on the wire until arm_finish enters HOLD. */
    r->state = d.do_arm ? (uint8_t)LIFE_IDLE : d.next_state;
}

/* Wrap/stale-safe elapsed-time test. If `t` was stamped after `now` was captured (e.g. a
   timestamp set inside the just-run ~25 ms arm handshake), the signed difference is
   negative → "not yet elapsed", instead of a huge unsigned value that would false-trip a
   timeout. Thresholds are small ms, far from the ±2^31 wrap. */
static inline int32_t ms_since(uint32_t now, uint32_t t) { return (int32_t)(now - t); }

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

        uint8_t driving = (motors_rt[i].state == LIFE_HOLD ||
                           motors_rt[i].state == LIFE_MIT  ||
                           motors_rt[i].state == LIFE_DAMPED ||
                           motors_rt[i].state == LIFE_TO_ZERO);

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
        uint8_t cid = motor_configs[i].can_id;

        uint8_t driving = (motors_rt[i].state == LIFE_HOLD ||
                           motors_rt[i].state == LIFE_MIT  ||
                           motors_rt[i].state == LIFE_DAMPED ||
                           motors_rt[i].state == LIFE_TO_ZERO);

        /* Fault detection on the mirrored feedback state (priority; first trip latches).
           CAN-timeout is time-based (feedback absence) so it belongs here in the per-cycle
           service, not the feedback path; the armed_ms grace covers the first cycles after
           an arm (a just-disabled motor's last_fb is stale). */
        if (driving &&
            ms_since(now_ms, motors_rt[i].last_fb_ms) >= (int32_t)MOTOR_CAN_FB_TIMEOUT_MS &&
            ms_since(now_ms, motors_rt[i].armed_ms)   >= (int32_t)MOTOR_CAN_FB_TIMEOUT_MS) {
            fault_to(i, CAUSE_CAN_TIMEOUT);
        } else if (driving && motors_rt[i].motor_fault != 0u) {
            fault_to(i, CAUSE_MOTOR_FAULT);
            motor_set_fault_word(i, 0xFFFFFFFFu);
            motors_rt[i].fault_word = 0xFFFFFFFFu;
            can_read_single_param(cid, CAN_MASTER_ID, 0x3022u);
        } else if (driving && motors_rt[i].mon_verdict == ENABLE_MON_FAULT_NOT_ENABLED) {
            fault_to(i, CAUSE_NOT_ENABLED);
        } else if (driving &&
                   fabsf(motors_rt[i].tau) > motors_rt[i].cfg->max_tau) {
            fault_to(i, CAUSE_OVERTORQUE);
        }

        /* Master-loss mid-handshake: the level-triggered HOLD stream stopped, so cancel
           the async arm. The motor is not driving yet — no ramp needed, just make sure
           it is disabled (safe at any phase) and stay IDLE. */
        if (!cmd_fresh && arm_sm_active(motors_rt[i].arm_phase)) {
            arm_sm_cancel(&motors_rt[i].arm_phase, &motors_rt[i].arm_wait);
            can_disable_motor(cid, CAN_MASTER_ID);
            motors_rt[i].cause = CAUSE_MASTER_LOST;
        }

        /* Master-loss ramp (task 6): a cycle with NO fresh ROBOT_CMD (the fallback tick)
           means SPI exchanges stopped — master reset/dead. A still-armed joint ramps down
           by time since the last command (watchdog_ms, refreshed ONLY by ROBOT_CMD): hold
           the position with v_des/tau_ff zeroed for the grace, then DAMPED (config Kd),
           then IDLE. Recompute driving first — a fault above may have cleared it. */
        driving = (motors_rt[i].state == LIFE_HOLD || motors_rt[i].state == LIFE_MIT ||
                   motors_rt[i].state == LIFE_DAMPED || motors_rt[i].state == LIFE_TO_ZERO);
        if (driving && !cmd_fresh) {
            uint32_t since = (uint32_t)ms_since(now_ms, motors_rt[i].watchdog_ms);
            MasterLossPhase ph = master_loss_phase(since, MASTER_LOST_GRACE_MS,
                                                   MASTER_LOST_DAMP_MS);
            if (ph == MASTER_HOLD_GRACE) {
                send_mit(i, 0.0f, motors_rt[i].hold_pos, 0.0f,       /* v_des=0, tau_ff=0 */
                         motors_rt[i].cmd_kp, motors_rt[i].cmd_kd);
            } else if (ph == MASTER_DAMPED) {
                motors_rt[i].state  = LIFE_DAMPED;
                motors_rt[i].cause  = CAUSE_MASTER_LOST;
                motors_rt[i].cmd_kp = 0.0f;
                motors_rt[i].cmd_kd = motors_rt[i].cfg->default_kd;  /* config Kd */
                send_mit(i, 0.0f, motors_rt[i].pos, 0.0f, 0.0f, motors_rt[i].cfg->default_kd);
            } else {   /* MASTER_IDLE */
                can_disable_motor(cid, CAN_MASTER_ID);
                motors_rt[i].state          = LIFE_IDLE;
                motors_rt[i].cause          = CAUSE_MASTER_LOST;
                motors_rt[i].hold_vel       = 0.0f;
                motors_rt[i].target_cmd_seq = CMD_SEQ_NONE;
                cmd_seq_reset(&motors_rt[i].cmd_track);
            }
        } else switch (motors_rt[i].state) {
            case LIFE_IDLE:
                /* An in-progress async arm owns this motor's one-frame cycle budget:
                   step the phase machine instead of the state poll. */
                if (arm_sm_active(motors_rt[i].arm_phase)) {
                    uint8_t act = arm_sm_step(&motors_rt[i].arm_phase, &motors_rt[i].arm_wait,
                                              ARM_CLEAR_WAIT_CYCLES, ARM_SETTLE_WAIT_CYCLES);
                    if      (act == ARM_ACT_SEND_CLEAR)  can_clear_fault(cid, CAN_MASTER_ID);
                    else if (act == ARM_ACT_SEND_MODE)   can_change_motor_mode(cid, CAN_MASTER_ID, MIT_MODE);
                    else if (act == ARM_ACT_SEND_ENABLE) can_enable_motor(cid, CAN_MASTER_ID);
                    else if (act == ARM_ACT_COMPLETE)    arm_finish(i, now_ms);
                } else {
                    can_read_motor_state(cid);
                }
                break;

            case LIFE_HOLD:
            case LIFE_MIT:
                /* With a fresh command every cycle, HOLD and MIT both drive the current
                   targets; master-loss is handled by the ramp above, not here. cmd_tau_ff
                   is nonzero only in MIT (arm/re-hold/damped/to-zero all reset it). */
                send_mit(i, motors_rt[i].cmd_tau_ff, motors_rt[i].hold_pos,
                         motors_rt[i].hold_vel, motors_rt[i].cmd_kp, motors_rt[i].cmd_kd);
                break;

            case LIFE_DAMPED:
                /* Kp=0 so home_pos is inert; Kd provides damping. */
                send_mit(i, 0.0f, motors_rt[i].pos, 0.0f, 0.0f, motors_rt[i].cmd_kd);
                break;

            case LIFE_TO_ZERO: {
                float pos     = motors_rt[i].pos;
                float abs_pos = fabsf(pos);

                /* (1) progress/stall watchdog → damp + CAUSE_ZERO_TIMEOUT. */
                if (abs_pos < motors_rt[i].zero_best_abs - MOTOR_ZERO_PROGRESS_EPS) {
                    motors_rt[i].zero_best_abs    = abs_pos;
                    motors_rt[i].zero_progress_ms = now_ms;
                }
                if (ms_since(now_ms, motors_rt[i].zero_progress_ms) > (int32_t)MOTOR_ZERO_STALL_MS) {
                    send_mit(i, 0.0f, 0.0f, 0.0f, 0.0f, MOTOR_ZERO_DAMP_KD);
                    motors_rt[i].state    = LIFE_FAULT;
                    motors_rt[i].cause    = CAUSE_ZERO_TIMEOUT;
                    motors_rt[i].hold_vel = 0.0f;
                    break;
                }

                /* (2) arrival = ramp done + within tol for SETTLE_TICKS. Hold at 0
                   and set the arrived flag; STAY in TO_ZERO (never writes mech zero,
                   never disables/re-enables — the old arrival sequence is removed). */
                if (fabsf(motors_rt[i].hold_pos) < 1e-4f && abs_pos < MOTOR_ZERO_TOL) {
                    if (++motors_rt[i].zero_settle >= MOTOR_ZERO_SETTLE_TICKS) {
                        motors_rt[i].to_zero_arrived = 1u;
                    }
                    send_mit(i, 0.0f, 0.0f, 0.0f, MOTOR_ZERO_KP, MOTOR_ZERO_KD);
                    break;
                }
                motors_rt[i].zero_settle = 0u;

                /* (3) leashed creep toward 0. */
                float sign = (pos > 0.0f) ? -1.0f : 1.0f;
                float lead = motors_rt[i].hold_pos - pos;
                float vff  = 0.0f;
                if (fabsf(lead) < MOTOR_ZERO_LEASH) {
                    motors_rt[i].hold_pos += sign * MOTOR_ZERO_RATE * MOTOR_LOOP_DT_S;
                    if (sign < 0.0f && motors_rt[i].hold_pos < 0.0f) motors_rt[i].hold_pos = 0.0f;
                    if (sign > 0.0f && motors_rt[i].hold_pos > 0.0f) motors_rt[i].hold_pos = 0.0f;
                    vff = sign * MOTOR_ZERO_RATE;
                }
                send_mit(i, 0.0f, motors_rt[i].hold_pos, vff,
                         MOTOR_ZERO_KP, MOTOR_ZERO_KD);
                break;
            }

            default:   /* LIFE_BOOT / LIFE_DISCOVERING / LIFE_FAULT */
                break;
        }

        /* Assemble the per-tick telemetry cmd flags (clamp while fresh, else stale). */
        {
            uint8_t cf = 0u;
            if (motors_rt[i].state == LIFE_HOLD || motors_rt[i].state == LIFE_MIT) {
                cf = motors_rt[i].got_fresh_cmd
                         ? motors_rt[i].last_apply_clamp
                         : (uint8_t)TELE_FLAG_CMD_STALE;
            }
            motors_rt[i].got_fresh_cmd     = 0u;
            motors_rt[i].cmd_flags_latched = cf;
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
    out->state  = r->state;
    out->cause  = r->cause;
    out->motor_mode  = r->motor_mode;
    out->motor_fault = r->motor_fault;

    uint8_t flags = r->cmd_flags_latched;
    if (r->req_rejected)    flags |= TELE_FLAG_REQUEST_REJECTED;
    if (r->to_zero_arrived) flags |= TELE_FLAG_TO_ZERO_ARRIVED;
    if (sat)                flags |= TELE_FLAG_SATURATED;
    out->flags = flags;

    uint32_t age = HAL_GetTick() - r->last_fb_ms;
    out->fb_age_ms = (age > 255u) ? 255u : (uint8_t)age;
    out->fault_word       = r->fault_word;
    out->last_applied_seq = r->cmd_track.last_applied;
    out->reserved         = 0u;
}

uint8_t motor_runtime_motors_alive(void)
{
    uint8_t mask = 0u;
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        if (motors_rt[i].alive) mask |= (uint8_t)(1u << i);
    }
    return mask;
}
