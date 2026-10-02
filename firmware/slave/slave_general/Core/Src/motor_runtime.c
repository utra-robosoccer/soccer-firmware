#include "motor_runtime.h"
#include "motor_chain.h"
#include "robostride.h"
#include "enable_monitor.h"
#include "mode_sm.h"
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

/* One-sided soft-limit clamp on a commanded (pos, vel). Returns clamp flag bits
   (TELE_FLAG_CLAMPED_POS/TAU); cancels only feedforward driving further INTO the
   limit so leaving it stays smooth. */
static uint8_t apply_soft_clamp(const MotorRuntime *r, float *pos, float *vel)
{
    float lo = r->cfg->soft_min, hi = r->cfg->soft_max;
    uint8_t clamp = 0u;
    if (*pos > hi) {
        *pos = hi; clamp |= TELE_FLAG_CLAMPED_POS;
        if (*vel > 0.0f) { *vel = 0.0f; clamp |= TELE_FLAG_CLAMPED_TAU; }
    } else if (*pos < lo) {
        *pos = lo; clamp |= TELE_FLAG_CLAMPED_POS;
        if (*vel < 0.0f) { *vel = 0.0f; clamp |= TELE_FLAG_CLAMPED_TAU; }
    }
    return clamp;
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

/* Run the CAN enable handshake and capture the home-frame hold position. The ONLY
   place the motor is enabled. Never writes mechanical zero. The caller (apply_cmd)
   has already cleared the wound guard via the state machine; this just enables. */
static void arm_enable(uint8_t idx)
{
    MotorRuntime *r = &motors_rt[idx];
    uint8_t cid = motor_configs[idx].can_id;

    motor_t snap;
    motor_get_snapshot(idx, &snap);
    float wrapped = wrap_pi(snap.pos);

    /* Clear a latched motor fault first (the RS refuses enable otherwise). */
    if (snap.fault_word != 0u) {
        can_clear_fault(cid, CAN_MASTER_ID);
        HAL_Delay(5u);
    }
    motor_set_fault_word(idx, 0u);
    r->fault_word = 0u;
    r->cause      = CAUSE_NONE;

    r->pos        = wrapped;
    r->pos_offset = snap.pos - wrapped;
    r->hold_pos   = wrapped;
    r->hold_vel   = 0.0f;
    r->cmd_kp     = r->cfg->default_kp;
    r->cmd_kd     = r->cfg->default_kd;

    /* Enable handshake: mode + enable, each with an ACK wait + 10 ms settle. */
    uint32_t ts = HAL_GetTick();
    can_rx_flag = 0;
    can_change_motor_mode(cid, CAN_MASTER_ID, MIT_MODE);
    while (can_rx_flag == 0 && (HAL_GetTick() - ts) < 10u) {}
    HAL_Delay(10u);

    ts = HAL_GetTick();
    can_rx_flag = 0;
    can_enable_motor(cid, CAN_MASTER_ID);
    while (can_rx_flag == 0 && (HAL_GetTick() - ts) < 10u) {}
    HAL_Delay(10u);

    send_mit(idx, 0.0f, r->hold_pos, 0.0f, r->cmd_kp, r->cmd_kd);

    r->target_cmd_seq = CMD_SEQ_NONE;
    cmd_seq_reset(&r->cmd_track);
    enable_monitor_reset(idx);
    r->to_zero_arrived = 0u;
    r->watchdog_ms = HAL_GetTick();
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
       snapshot in that case so the pure SM can decide. */
    uint8_t wound = 0u;
    if (m->mode_req == REQ_HOLD && r->state == LIFE_IDLE) {
        if (!r->alive) { r->req_rejected = 1u; return; }
        motor_t snap;
        motor_get_snapshot(idx, &snap);
        float off = snap.pos - wrap_pi(snap.pos);
        wound = (off > MOTOR_WOUND_OFFSET_MAX || off < -MOTOR_WOUND_OFFSET_MAX) ? 1u : 0u;
    }

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
        can_disable_motor(cid, CAN_MASTER_ID);
        r->hold_vel        = 0.0f;
        r->target_cmd_seq  = CMD_SEQ_NONE;
        cmd_seq_reset(&r->cmd_track);
        r->to_zero_arrived = 0u;
    }
    if (d.do_arm) {
        arm_enable(idx);                 /* enable + capture (handshake, once) */
    }
    if (d.capture_hold) {                /* re-hold without re-enabling */
        r->hold_pos = r->pos;
        r->hold_vel = 0.0f;
        r->cmd_kp   = r->cfg->default_kp;
        r->cmd_kd   = r->cfg->default_kd;
        r->to_zero_arrived = 0u;
    }
    if (d.enter_to_zero) {               /* init creep on entry */
        r->hold_pos         = r->pos;
        r->hold_vel         = 0.0f;
        r->zero_best_abs    = fabsf(r->pos);
        r->zero_progress_ms = HAL_GetTick();
        r->zero_settle      = 0u;
        r->to_zero_arrived  = 0u;
    }

    uint8_t use_cfg = (m->flags & CMD_FLAG_USE_CONFIG_GAINS) != 0u;
    if (d.next_state == LIFE_MIT) {
        r->hold_pos = proto_i16_to_f(m->pos, PROTO_POS_SCALE);
        r->hold_vel = proto_i16_to_f(m->vel, PROTO_VEL_SCALE);
        r->last_apply_clamp = apply_soft_clamp(r, &r->hold_pos, &r->hold_vel);
        r->cmd_kp = use_cfg ? r->cfg->default_kp : proto_u16_to_f(m->kp, PROTO_KP_SCALE);
        r->cmd_kd = use_cfg ? r->cfg->default_kd : proto_u16_to_f(m->kd, PROTO_KD_SCALE);
        r->target_cmd_seq  = cmd_seq;
        r->got_fresh_cmd   = 1u;
        r->to_zero_arrived = 0u;
    } else if (d.next_state == LIFE_DAMPED) {
        r->cmd_kp   = 0.0f;              /* backdrive-safe: no position hold */
        r->cmd_kd   = use_cfg ? r->cfg->default_kd : proto_u16_to_f(m->kd, PROTO_KD_SCALE);
        r->hold_vel = 0.0f;
        r->to_zero_arrived = 0u;
    }

    r->state = d.next_state;
}

void motor_runtime_update(uint32_t now_ms)
{
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        uint8_t cid = motor_configs[i].can_id;

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

        /* Enable monitor + cmd_seq reply pairing, both on a FRESH feedback frame. */
        uint8_t mon_fresh = (snap.fb_count != motors_rt[i].mon_prev_fb_count);
        EnableMonVerdict mon = enable_monitor_step(
            driving, mon_fresh, (uint8_t)(snap.status == RS_MODE_NORMAL),
            MOTOR_ENABLE_MON_K, &motors_rt[i].mon_not_enabled);
        if (mon_fresh) {
            motors_rt[i].mon_prev_fb_count = snap.fb_count;
            cmd_seq_on_reply(&motors_rt[i].cmd_track);   /* pairs with last tick's MIT */
        }

        /* Fault detection while driving (priority order; first trip latches FAULT). */
        if (driving &&
            (uint32_t)(now_ms - snap.last_fb_ms) >= MOTOR_CAN_FB_TIMEOUT_MS) {
            fault_to(i, CAUSE_CAN_TIMEOUT);
        } else if (driving && motors_rt[i].motor_fault != 0u) {
            fault_to(i, CAUSE_MOTOR_FAULT);
            motor_set_fault_word(i, 0xFFFFFFFFu);
            motors_rt[i].fault_word = 0xFFFFFFFFu;
            can_read_single_param(cid, CAN_MASTER_ID, 0x3022u);
        } else if (driving && mon == ENABLE_MON_FAULT_NOT_ENABLED) {
            fault_to(i, CAUSE_NOT_ENABLED);
        } else if (driving &&
                   fabsf(motors_rt[i].tau) > motors_rt[i].cfg->max_tau) {
            fault_to(i, CAUSE_OVERTORQUE);
        }

        switch (motors_rt[i].state) {
            case LIFE_IDLE:
                can_read_motor_state(cid);
                break;

            case LIFE_HOLD:
                send_mit(i, 0.0f, motors_rt[i].hold_pos, motors_rt[i].hold_vel,
                         motors_rt[i].cmd_kp, motors_rt[i].cmd_kd);
                if ((now_ms - motors_rt[i].watchdog_ms) > MOTOR_WATCHDOG_MS) {
                    can_disable_motor(cid, CAN_MASTER_ID);
                    motors_rt[i].state    = LIFE_IDLE;
                    motors_rt[i].hold_vel = 0.0f;
                    motors_rt[i].cause    = CAUSE_WATCHDOG;
                }
                break;

            case LIFE_MIT:
                send_mit(i, 0.0f, motors_rt[i].hold_pos, motors_rt[i].hold_vel,
                         motors_rt[i].cmd_kp, motors_rt[i].cmd_kd);
                if ((now_ms - motors_rt[i].watchdog_ms) > MOTOR_WATCHDOG_MS) {
                    /* MIT stream stopped — lock position and fall back to HOLD. */
                    motors_rt[i].hold_pos = motors_rt[i].pos;
                    motors_rt[i].hold_vel = 0.0f;
                    motors_rt[i].cmd_kp   = motors_rt[i].cfg->default_kp;
                    motors_rt[i].cmd_kd   = motors_rt[i].cfg->default_kd;
                    motors_rt[i].state    = LIFE_HOLD;
                }
                break;

            case LIFE_DAMPED:
                /* Kp=0 so home_pos is inert; Kd provides damping. */
                send_mit(i, 0.0f, motors_rt[i].pos, 0.0f, 0.0f, motors_rt[i].cmd_kd);
                if ((now_ms - motors_rt[i].watchdog_ms) > MOTOR_WATCHDOG_MS) {
                    can_disable_motor(cid, CAN_MASTER_ID);
                    motors_rt[i].state = LIFE_IDLE;
                    motors_rt[i].cause = CAUSE_WATCHDOG;
                }
                break;

            case LIFE_TO_ZERO: {
                float pos     = motors_rt[i].pos;
                float abs_pos = fabsf(pos);

                /* (1) progress/stall watchdog → damp + CAUSE_ZERO_TIMEOUT. */
                if (abs_pos < motors_rt[i].zero_best_abs - MOTOR_ZERO_PROGRESS_EPS) {
                    motors_rt[i].zero_best_abs    = abs_pos;
                    motors_rt[i].zero_progress_ms = now_ms;
                }
                if ((now_ms - motors_rt[i].zero_progress_ms) > MOTOR_ZERO_STALL_MS) {
                    send_mit(i, 0.0f, 0.0f, 0.0f, 0.0f, MOTOR_ZERO_DAMP_KD);
                    motors_rt[i].state    = LIFE_FAULT;
                    motors_rt[i].cause    = CAUSE_ZERO_TIMEOUT;
                    motors_rt[i].hold_vel = 0.0f;
                    break;
                }

                /* (2) master-link watchdog. */
                if ((now_ms - motors_rt[i].watchdog_ms) > MOTOR_WATCHDOG_MS) {
                    can_disable_motor(cid, CAN_MASTER_ID);
                    motors_rt[i].state = LIFE_IDLE;
                    motors_rt[i].cause = CAUSE_WATCHDOG;
                    break;
                }

                /* (3) arrival = ramp done + within tol for SETTLE_TICKS. Hold at 0
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

                /* (4) leashed creep toward 0. */
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
