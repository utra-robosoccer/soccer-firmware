#include "motor_runtime.h"
#include "motor_chain.h"
#include "robostride.h"
#include "enable_monitor.h"
#include <math.h>
#include <string.h>

#define TWO_PI 6.28318530717958647692f

MotorRuntime motors_rt[N_MOTORS];

/* ── helpers ─────────────────────────────────────────────────────────────── */

/* Wrap an angle into [-pi, pi]. The RobStride reports its single-turn absolute
   angle after power-up, so a slightly-negative joint can come back reading
   ~+2pi. Joints are constrained to within +-pi of home, so the nearest
   representative is unambiguous. */
static float wrap_pi(float a)
{
    return a - TWO_PI * roundf(a / TWO_PI);
}

/* Send an MIT command whose position is given in the home frame ([-pi, pi]).
   pos_offset (the 2*pi multiple the motor adopted at power-up) is added back so
   the motor's controller drives the short path to the home-frame target instead
   of unwinding a full turn. */
static void send_mit(uint8_t idx, float torque, float home_pos, float vel,
                     float kp, float kd)
{
    can_mit_control_set(motor_configs[idx].can_id, torque,
                        home_pos + motors_rt[idx].pos_offset, vel, kp, kd);
}

/* Drop a single motor's output and return it to IDLE. Disabling an already-idle
   motor is harmless, so this is safe to call from any state. */
static void idle_motor(uint8_t idx)
{
    can_disable_motor(motor_configs[idx].can_id, CAN_MASTER_ID);
    motors_rt[idx].state    = MOTOR_IDLE;
    motors_rt[idx].hold_vel = 0.0f;
}

/* Reset the enable monitor to a clean slate: clear the not-NORMAL count and seed the
   fresh-frame reference to the motor's current fb_count, so only genuinely new
   (post-enable) feedback is judged. Takes its own snapshot so it is independent of any
   diagnostics. Called on each ARM / GOTO_ZERO entry and when the ZEROING-arrival
   re-enable window ends. */
static void enable_monitor_reset(uint8_t idx)
{
    motor_t s;
    motor_get_snapshot(idx, &s);
    motors_rt[idx].mon_not_enabled   = 0u;
    motors_rt[idx].mon_prev_fb_count = s.fb_count;
    motors_rt[idx].mon_suspended     = 0u;
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

/* Discover one motor: ping and wait up to 200 ms for response */
static void discover(uint8_t idx)
{
    uint8_t cid = motor_configs[idx].can_id;
    motors_rt[idx].state = MOTOR_DISCOVERING;

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
            motors_rt[idx].state = MOTOR_IDLE;
            HAL_Delay(20u);
            return;
        }
        HAL_Delay(50u);  /* brief pause before retry */
    }

    motors_rt[idx].alive = 0u;
    motors_rt[idx].state = MOTOR_FAULT;
}

/* ── public API ──────────────────────────────────────────────────────────── */

void motor_runtime_init(void)
{
    memset(motors_rt, 0, sizeof(motors_rt));
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        motors_rt[i].cfg   = &motor_configs[i];
        motors_rt[i].state = MOTOR_BOOT;
    }
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        discover(i);
    }
}

void motor_runtime_update(uint32_t now_ms)
{
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        uint8_t cid = motor_configs[i].can_id;

        /* ONE coherent snapshot per motor per tick — feeds both the control logic
           below AND the telemetry mirror cached into motors_rt[], so what we
           report is exactly the state the logic acted on. pack_tele() then reads
           only motors_rt[] (no second read of the live motors[]). */
        motor_t snap;
        motor_get_snapshot(i, &snap);

        float wrapped            = wrap_pi(snap.pos);
        motors_rt[i].pos         = wrapped;
        motors_rt[i].pos_offset  = snap.pos - wrapped;
        motors_rt[i].vel         = snap.rpm;
        motors_rt[i].tau         = snap.torq;
        motors_rt[i].temp        = snap.temperature;
        motors_rt[i].motor_fault = pack_faults(&snap);
        motors_rt[i].fault_word  = snap.fault_word;   /* telemetry mirror */
        motors_rt[i].last_fb_ms  = snap.last_fb_ms;   /* fb_age basis      */

        /* ── Fault detection while driving (hold, live MIT, or homing). Any trip
           idles the motor and LATCHES a cause (cleared only on re-arm/disable).
           Checked in priority order; the first to fire idles the motor, so the
           chained else-ifs then see IDLE and skip.

           CAN_TIMEOUT first: if this motor's feedback is stale, its tau and fault
           bits are stale too and must not be trusted. OVERTORQUE guards ZEROING
           as well — the creep waypoint marches toward 0 even when the joint is
           blocked, so a jam would otherwise build torque unbounded; free creep
           peaks well under max_tau so this does not false-trip a normal homing.
           MOTOR_FAULT means the RS motor's own protection fired (it isn't running
           MIT anyway) — idle, latch the cause, and one-shot read 0x3022 for the
           detail (ISR latches the reply into the live fault_word). All are
           one-shot: once idled the motor is no longer "driving", so no re-fire. */
        uint8_t driving = (motors_rt[i].state == MOTOR_ARMED_HOLD ||
                           motors_rt[i].state == MOTOR_ARMED_MIT  ||
                           motors_rt[i].state == MOTOR_ZEROING);

        /* Enable monitor (pure decision in enable_monitor.c). Runs every tick but
           only accumulates on a FRESH feedback frame (per-motor fb_count changed);
           NORMAL resets it, K not-NORMAL fresh frames → NOT_ENABLED. Silence is left
           to CAN_TIMEOUT below (a stale tick just holds the count). Suspended during
           the firmware's own disable→set-zero→re-enable window (ZEROING arrival). */
        uint8_t mon_fresh = (snap.fb_count != motors_rt[i].mon_prev_fb_count);
        EnableMonVerdict mon = enable_monitor_step(
            (uint8_t)(driving && !motors_rt[i].mon_suspended), mon_fresh,
            (uint8_t)(snap.status == RS_MODE_NORMAL), MOTOR_ENABLE_MON_K,
            &motors_rt[i].mon_not_enabled);
        if (mon_fresh) motors_rt[i].mon_prev_fb_count = snap.fb_count;

        if (driving &&
            (uint32_t)(now_ms - snap.last_fb_ms) >= MOTOR_CAN_FB_TIMEOUT_MS) {
            idle_motor(i);
            motors_rt[i].cause = CAUSE_CAN_TIMEOUT;
        } else if (driving && motors_rt[i].motor_fault != 0u) {
            idle_motor(i);
            motors_rt[i].cause = CAUSE_MOTOR_FAULT;
            motor_set_fault_word(i, 0xFFFFFFFFu);   /* sentinel in live motors[] */
            motors_rt[i].fault_word = 0xFFFFFFFFu;   /* mirror it this tick too   */
            can_read_single_param(cid, CAN_MASTER_ID, 0x3022u);
        } else if (driving && mon == ENABLE_MON_FAULT_NOT_ENABLED) {
            idle_motor(i);
            motors_rt[i].cause = CAUSE_NOT_ENABLED;
        } else if (driving &&
                   fabsf(motors_rt[i].tau) > motors_rt[i].cfg->max_tau) {
            idle_motor(i);
            motors_rt[i].cause = CAUSE_OVERTORQUE;
        }

        switch (motors_rt[i].state) {
            case MOTOR_IDLE:
                can_read_motor_state(cid);
                break;

            case MOTOR_ARMED_HOLD:
                send_mit(i, 0.0f,
                         motors_rt[i].hold_pos,
                         motors_rt[i].hold_vel,
                         motors_rt[i].cfg->default_kp,
                         motors_rt[i].cfg->default_kd);
                if ((now_ms - motors_rt[i].watchdog_ms) > MOTOR_WATCHDOG_MS) {
                    can_disable_motor(cid, CAN_MASTER_ID);
                    motors_rt[i].state    = MOTOR_IDLE;
                    motors_rt[i].hold_vel = 0.0f;
                    motors_rt[i].cause    = CAUSE_WATCHDOG;
                }
                break;

            case MOTOR_ZEROING: {
                float pos     = motors_rt[i].pos;
                float abs_pos = fabsf(pos);

                /* (1) Progress/stall watchdog. A "stall" is *no progress toward
                   home* for MOTOR_ZERO_STALL_MS — which correctly catches a
                   leash-pause that never clears (a blocked joint) WITHOUT
                   false-tripping a slow-but-converging creep or a motor already
                   sitting at home. (An absolute-time budget can't tell "slow" from
                   "stuck", and collapsed to ~1 s whenever homing started near
                   zero.) Any improvement of |pos| by MOTOR_ZERO_PROGRESS_EPS
                   refreshes the timer. On trip, damp the joint (Kp=0, Kd high,
                   tau=0) so a partially-homed link is held compliant rather than
                   dropped, then latch FAULT; damping is one-shot (FAULT stops
                   further MIT, so the motor holds it until its own CAN timeout). */
                if (abs_pos < motors_rt[i].zero_best_abs - MOTOR_ZERO_PROGRESS_EPS) {
                    motors_rt[i].zero_best_abs    = abs_pos;
                    motors_rt[i].zero_progress_ms = now_ms;
                }
                if ((now_ms - motors_rt[i].zero_progress_ms) > MOTOR_ZERO_STALL_MS) {
                    send_mit(i, 0.0f, 0.0f, 0.0f, 0.0f, MOTOR_ZERO_DAMP_KD);
                    motors_rt[i].state    = MOTOR_FAULT;
                    motors_rt[i].cause    = CAUSE_ZERO_TIMEOUT;
                    motors_rt[i].hold_vel = 0.0f;
                    break;
                }

                /* (2) Watchdog — bail to IDLE if the host stopped commanding
                   mid-creep. (Only reached while still ZEROING; the arrival branch
                   below transitions out and breaks before this, so its blocking
                   re-enable can't underflow the freshly-set watchdog_ms.) */
                if ((now_ms - motors_rt[i].watchdog_ms) > MOTOR_WATCHDOG_MS) {
                    can_disable_motor(cid, CAN_MASTER_ID);
                    motors_rt[i].state = MOTOR_IDLE;
                    motors_rt[i].cause = CAUSE_WATCHDOG;
                    break;
                }

                /* (3) Arrival = SETTLED, not merely near. The waypoint ramp must
                   have finished (hold_pos clamped to 0) AND the joint held within
                   MOTOR_ZERO_TOL of it for MOTOR_ZERO_SETTLE_TICKS in a row. We
                   gate on ramp-done + position, NOT instantaneous velocity: at the
                   soft homing gains the RS velocity-feedback noise floor (~0.2
                   rad/s) sits right against the 0.3 rad/s creep rate, so no vel
                   threshold separates a true hold from a slewing creep. "Ramp
                   finished and still within tol" is the reliable settled signal —
                   you cannot be passing through fast once the commanded waypoint
                   has stopped at 0 and you've tracked it for the settle window. */
                if (fabsf(motors_rt[i].hold_pos) < 1e-4f && abs_pos < MOTOR_ZERO_TOL) {
                    if (++motors_rt[i].zero_settle < MOTOR_ZERO_SETTLE_TICKS) {
                        /* Hold at home while the joint settles. */
                        send_mit(i, 0.0f, 0.0f, 0.0f, MOTOR_ZERO_KP, MOTOR_ZERO_KD);
                        break;
                    }
                    /* Settled → arrival. Reset the motor's mechanical zero HERE so
                       its multi-turn frame is 0 at home. Without this a motor that
                       has wound up sits near the ±4π position-report limit, where
                       the 16-bit position feedback wraps; the short-path pos_offset
                       then flips sign mid-motion and MIT commands run away.
                       Resetting at home removes the winding while keeping the
                       physical home reference. Home therefore drifts by the settled
                       residual (< MOTOR_ZERO_TOL) each re-zero — reference
                       calibration is a separate manual procedure (docs/command.md). */
                    uint32_t ts;
                    motors_rt[i].mon_suspended = 1u;  /* intentional disable window */
                    can_disable_motor(cid, CAN_MASTER_ID);
                    HAL_Delay(5u);
                    can_set_mech_zero(cid, CAN_MASTER_ID);
                    HAL_Delay(5u);
                    /* Re-establish MIT with the SAME ACK-wait + 10 ms settle as
                       motor_runtime_arm. The RS02 needs the longer settle after
                       disable/set-zero — with only ~2 ms the enable is missed and
                       the motor silently ignores MIT (holds at 0, tau 0). */
                    ts = HAL_GetTick(); can_rx_flag = 0;
                    can_change_motor_mode(cid, CAN_MASTER_ID, MIT_MODE);
                    while (can_rx_flag == 0 && (HAL_GetTick() - ts) < 10u) {}
                    HAL_Delay(10u);
                    ts = HAL_GetTick(); can_rx_flag = 0;
                    can_enable_motor(cid, CAN_MASTER_ID);
                    while (can_rx_flag == 0 && (HAL_GetTick() - ts) < 10u) {}
                    HAL_Delay(10u);
                    motors_rt[i].pos         = 0.0f;
                    motors_rt[i].pos_offset  = 0.0f;
                    motors_rt[i].hold_pos    = 0.0f;
                    motors_rt[i].hold_vel    = 0.0f;
                    motors_rt[i].watchdog_ms = HAL_GetTick(); /* post-delay, not stale now_ms */
                    /* Resume the enable monitor from a clean slate: if THIS re-enable
                       silently failed, the check now catches it (NOT_ENABLED). */
                    enable_monitor_reset(i);
                    motors_rt[i].state       = MOTOR_ARMED_HOLD;
                    break;
                }
                motors_rt[i].zero_settle = 0u;   /* left the tolerance band — restart the settle count */

                /* (4) Leashed creep. Advance the waypoint toward 0 at
                   MOTOR_ZERO_RATE (scaled by the real loop period so the speed is
                   loop-frequency-independent) ONLY while it isn't already leading
                   the joint by more than MOTOR_ZERO_LEASH. If the joint lags
                   further (obstruction/binding), pause the ramp and zero the
                   velocity feed-forward so the commanded error — and thus the
                   force — stays bounded at ~Kp*leash instead of winding up. */
                float sign = (pos > 0.0f) ? -1.0f : 1.0f;
                float lead = motors_rt[i].hold_pos - pos;   /* both in home frame */
                float vff  = 0.0f;
                if (fabsf(lead) < MOTOR_ZERO_LEASH) {
                    motors_rt[i].hold_pos += sign * MOTOR_ZERO_RATE * MOTOR_LOOP_DT_S;
                    /* Clamp — don't overshoot zero */
                    if (sign < 0.0f && motors_rt[i].hold_pos < 0.0f) motors_rt[i].hold_pos = 0.0f;
                    if (sign > 0.0f && motors_rt[i].hold_pos > 0.0f) motors_rt[i].hold_pos = 0.0f;
                    vff = sign * MOTOR_ZERO_RATE;
                }
                send_mit(i, 0.0f, motors_rt[i].hold_pos, vff,
                         MOTOR_ZERO_KP, MOTOR_ZERO_KD);
                break;
            }

            case MOTOR_ARMED_MIT:
                send_mit(i, 0.0f,
                         motors_rt[i].hold_pos,
                         motors_rt[i].hold_vel,
                         motors_rt[i].cfg->default_kp,
                         motors_rt[i].cfg->default_kd);
                if ((now_ms - motors_rt[i].watchdog_ms) > MOTOR_WATCHDOG_MS) {
                    /* MIT commands stopped — lock position and fall back to hold */
                    motors_rt[i].hold_pos = motors_rt[i].pos;
                    motors_rt[i].hold_vel = 0.0f;
                    motors_rt[i].state    = MOTOR_ARMED_HOLD;
                }
                break;

            default:
                break;
        }

        /* cmd_flags — recomputed every tick, never latched. Only meaningful while
           armed: CLAMPED_* reflect the most recent MIT command's clamping;
           CMD_STALE means we're executing a held/watchdog setpoint, not a fresh
           command this window. */
        {
            uint8_t cf = 0u;
            if (motors_rt[i].state == MOTOR_ARMED_HOLD ||
                motors_rt[i].state == MOTOR_ARMED_MIT) {
                cf = motors_rt[i].got_fresh_cmd
                         ? motors_rt[i].last_apply_clamp
                         : (uint8_t)SPI_CMDFLAG_CMD_STALE;
            }
            motors_rt[i].got_fresh_cmd = 0u;
            motors_rt[i].cmd_flags     = cf;
        }
    }
}

HAL_StatusTypeDef motor_runtime_arm(uint8_t idx)
{
    if (idx >= N_MOTORS)                          return HAL_ERROR;
    if (motors_rt[idx].state != MOTOR_IDLE)       return HAL_ERROR;
    if (!motors_rt[idx].alive)                    return HAL_ERROR;

    uint8_t cid = motor_configs[idx].can_id;

    motor_t snap;
    motor_get_snapshot(idx, &snap);

    /* ARM clears any latched fault. If the motor's OWN fault was latched, its
       controller refuses the Type-3 enable until cleared, so send the CAN
       fault-clear (Type-4, Byte0=1) first. */
    if (snap.fault_word != 0u) {
        can_clear_fault(cid, CAN_MASTER_ID);
        HAL_Delay(5u);
    }
    motor_set_fault_word(idx, 0u);
    motors_rt[idx].fault_word = 0u;        /* mirror the clear */
    motors_rt[idx].cause      = CAUSE_NONE;

    float wrapped = wrap_pi(snap.pos);
    motors_rt[idx].pos        = wrapped;
    motors_rt[idx].pos_offset = snap.pos - wrapped;
    motors_rt[idx].hold_pos   = wrapped;   /* hold where it is, in home frame */
    motors_rt[idx].hold_vel   = 0.0f;

    /* Set MIT mode. Wait up to 10 ms for CAN ACK, then 10 ms settle so the
       motor completes its internal mode switch before the enable arrives.
       Budget per motor: 2 x (10+10) = 40 ms.  5 motors = 200 ms total, which
       keeps every already-zeroing motor's watchdog delta at ≤160 ms < 200 ms. */
    uint32_t ts = HAL_GetTick();
    can_rx_flag = 0;
    can_change_motor_mode(cid, CAN_MASTER_ID, MIT_MODE);
    while (can_rx_flag == 0 && (HAL_GetTick() - ts) < 10u) {}
    HAL_Delay(10u);

    /* Enable */
    ts = HAL_GetTick();
    can_rx_flag = 0;
    can_enable_motor(cid, CAN_MASTER_ID);
    while (can_rx_flag == 0 && (HAL_GetTick() - ts) < 10u) {}
    HAL_Delay(10u);

    /* First hold command */
    send_mit(idx, 0.0f, motors_rt[idx].hold_pos, 0.0f,
             motors_rt[idx].cfg->default_kp,
             motors_rt[idx].cfg->default_kd);

    /* Start the enable monitor for this armed session (catches a failed entry enable). */
    enable_monitor_reset(idx);
    motors_rt[idx].state       = MOTOR_ARMED_HOLD;
    motors_rt[idx].watchdog_ms = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef motor_runtime_disable(uint8_t idx)
{
    if (idx >= N_MOTORS) return HAL_ERROR;
    idle_motor(idx);
    motors_rt[idx].cause = CAUSE_NONE;      /* commanded stop, not a fault */
    motor_set_fault_word(idx, 0u);
    motors_rt[idx].fault_word = 0u;         /* mirror the clear */
    return HAL_OK;
}

HAL_StatusTypeDef motor_runtime_goto_zero(uint8_t idx)
{
    if (idx >= N_MOTORS)                    return HAL_ERROR;
    if (motors_rt[idx].state != MOTOR_IDLE) return HAL_ERROR;
    if (!motors_rt[idx].alive)              return HAL_ERROR;

    /* Gate 5a: home (0 rad) must lie inside the joint's soft limits, or zeroing
       would creep straight into a limit it can never satisfy. Refuse up front —
       a joint whose travel excludes 0 must be homed to a reachable reference by a
       separate procedure, not by GOTO_ZERO. */
    if (motor_configs[idx].soft_min > 0.0f || motor_configs[idx].soft_max < 0.0f)
        return HAL_ERROR;

    /* Gate 5b: GOTO_ZERO clears a latched fault as a side effect, but only for
       transient link causes (WATCHDOG, CAN_TIMEOUT) and a clean motor. A
       hardware/overtravel latch (OVERTORQUE, MOTOR_FAULT) must be acknowledged by
       an explicit ARM first — auto-clearing it here would let a jammed joint be
       re-driven blindly. Refuse; the reject is counted by the caller. */
    if (motors_rt[idx].cause == CAUSE_OVERTORQUE ||
        motors_rt[idx].cause == CAUSE_MOTOR_FAULT)
        return HAL_ERROR;

    uint8_t cid = motor_configs[idx].can_id;

    /* Snapshot for the fault-word check and the unwind decision below. A second
       snapshot is taken AFTER the mech-zero/enable sequence for the waypoint
       init, because those operations change the motor's reported angle. */
    motor_t snap;
    motor_get_snapshot(idx, &snap);

    /* Clear the (whitelisted) latched cause + fault word. GOTO_ZERO also enables
       the motor, so send the CAN fault-clear first when the motor's own fault was
       latched, or it refuses the enable. */
    if (snap.fault_word != 0u) {
        can_clear_fault(cid, CAN_MASTER_ID);
        HAL_Delay(5u);
    }
    motor_set_fault_word(idx, 0u);
    motors_rt[idx].fault_word = 0u;         /* mirror the clear */
    motors_rt[idx].cause      = CAUSE_NONE;

    /* Unwind a motor pinned at the ±4π position-report limit BEFORE trying to
       creep it home. Such a motor can't be controlled — commands saturate at
       the edge and the 16-bit feedback wraps. A motor wound by whole turns sits
       at the same physical shaft angle as home, so resetting its mechanical zero
       in place loses nothing and brings the frame back near 0. (|offset| ≳ 3π
       targets the ±4π case only; a ±2π winding is still safely controllable.) */
    {
        float off = snap.pos - wrap_pi(snap.pos);
        if (off > 9.0f || off < -9.0f) {
            can_set_mech_zero(cid, CAN_MASTER_ID);
            HAL_Delay(5u);
        }
    }

    /* Set MIT mode — same settle strategy as motor_runtime_arm */
    uint32_t ts = HAL_GetTick();
    can_rx_flag = 0;
    can_change_motor_mode(cid, CAN_MASTER_ID, MIT_MODE);
    while (can_rx_flag == 0 && (HAL_GetTick() - ts) < 10u) {}
    HAL_Delay(10u);

    /* Enable */
    ts = HAL_GetTick();
    can_rx_flag = 0;
    can_enable_motor(cid, CAN_MASTER_ID);
    while (can_rx_flag == 0 && (HAL_GetTick() - ts) < 10u) {}
    HAL_Delay(10u);

    /* Initialise waypoint at current position so the rate-limited ramp starts
       correctly. Re-snapshot here: the mech-zero/enable above may have changed
       the reported angle. Wrap to the home frame so zeroing creeps the short way
       toward 0 instead of unwinding a full turn from a wrapped power-up reading. */
    motor_t snap_after;
    motor_get_snapshot(idx, &snap_after);
    {
        float wrapped = wrap_pi(snap_after.pos);
        motors_rt[idx].pos        = wrapped;
        motors_rt[idx].pos_offset = snap_after.pos - wrapped;
    }
    motors_rt[idx].hold_pos = motors_rt[idx].pos;
    motors_rt[idx].hold_vel = 0.0f;


    /* Arm the progress/stall watchdog: best distance = the start distance, timer
       = now. It trips only if |pos| fails to close on home for
       MOTOR_ZERO_STALL_MS, so a slow (leash-paced) but converging creep never
       false-fires — unlike an absolute deadline. */
    motors_rt[idx].zero_best_abs    = fabsf(motors_rt[idx].hold_pos);
    motors_rt[idx].zero_progress_ms = HAL_GetTick();
    motors_rt[idx].zero_settle      = 0u;

    /* First command: hold current position before the update loop starts stepping */
    send_mit(idx, 0.0f, motors_rt[idx].hold_pos, 0.0f,
             MOTOR_ZERO_KP, MOTOR_ZERO_KD);

    /* Start the enable monitor for this zeroing session (catches a failed entry enable
       instead of waiting out the 1.5 s ZERO_TIMEOUT). */
    enable_monitor_reset(idx);
    motors_rt[idx].state        = MOTOR_ZEROING;
    motors_rt[idx].watchdog_ms  = HAL_GetTick();
    return HAL_OK;
}

void motor_runtime_refresh_watchdog(uint8_t idx)
{
    if (idx < N_MOTORS) {
        motors_rt[idx].watchdog_ms = HAL_GetTick();
    }
}

void motor_runtime_apply_mit(uint8_t idx, float pos, float vel)
{
    if (idx >= N_MOTORS) return;

    MotorRuntime *r = &motors_rt[idx];
    /* Streamed MIT setpoints are only honoured while the motor is armed. */
    if (r->state != MOTOR_ARMED_HOLD && r->state != MOTOR_ARMED_MIT) return;

    /* Enforce the per-motor soft angle limits here on the slave: the host may
       command past them (e.g. a ±90° sine), but the motor must stop at its
       limit. Clamp the position, and use a ONE-SIDED velocity clamp: only
       cancel feedforward velocity that drives further INTO the limit. Velocity
       that moves the joint back toward range is preserved, so leaving the limit
       is smooth — no discontinuous jump in the kd term at the peak. */
    float lo = r->cfg->soft_min;
    float hi = r->cfg->soft_max;
    uint8_t clamp = 0u;
    if (pos > hi) {
        pos = hi;
        clamp |= SPI_CMDFLAG_CLAMPED_POS;
        if (vel > 0.0f) { vel = 0.0f; clamp |= SPI_CMDFLAG_CLAMPED_TAU; }
    } else if (pos < lo) {
        pos = lo;
        clamp |= SPI_CMDFLAG_CLAMPED_POS;
        if (vel < 0.0f) { vel = 0.0f; clamp |= SPI_CMDFLAG_CLAMPED_TAU; }
    }

    /* Record clamp result for this tick's cmd_flags. CLAMPED_TAU marks the
       velocity feedforward (the kd torque term) being cancelled at the limit. */
    r->last_apply_clamp = clamp;
    r->got_fresh_cmd    = 1u;

    r->hold_pos = pos;
    r->hold_vel = vel;
    r->state    = MOTOR_ARMED_MIT;
}

void motor_runtime_sample(MotorSample *out, uint8_t idx)
{
    if (idx >= N_MOTORS || out == NULL) return;
    /* Reads ONLY motors_rt[], populated from the single per-tick snapshot in
       motor_runtime_update() — so telemetry reflects exactly the state the
       control logic acted on. Physical units only; spi_proto does the wire
       scaling/packing. */
    const MotorRuntime *r = &motors_rt[idx];

    out->pos         = r->pos;
    out->vel         = r->vel;
    out->tau         = r->tau;
    out->temp        = r->temp;
    out->life        = r->state;
    out->cause       = r->cause;
    out->motor_fault = r->motor_fault;
    out->cmd_flags   = r->cmd_flags;
    out->fault_word  = r->fault_word;
    /* Raw ms since this motor's last Type-2 feedback; the codec saturates to u8. */
    out->fb_age_ms   = HAL_GetTick() - r->last_fb_ms;
}

uint8_t motor_runtime_motors_alive(void)
{
    uint8_t mask = 0u;
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        if (motors_rt[i].alive) mask |= (uint8_t)(1u << i);
    }
    return mask;
}
