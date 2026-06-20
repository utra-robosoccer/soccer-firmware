#include "motor_runtime.h"
#include "motor_chain.h"
#include "robostride.h"
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

static uint16_t f_to_u16(float x, float lo, float hi)
{
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    return (uint16_t)((x - lo) * 65535.0f / (hi - lo));
}

static uint16_t pack_faults(const motor_t *m)
{
    uint16_t f = 0;
    f |= (uint16_t)m->motor_errors.undervoltage;
    f |= (uint16_t)m->motor_errors.driver_fault   << 1u;
    f |= (uint16_t)m->motor_errors.overheat        << 2u;
    f |= (uint16_t)m->motor_errors.encoder_fault   << 3u;
    f |= (uint16_t)m->motor_errors.stall_overload  << 4u;
    f |= (uint16_t)m->motor_errors.uncalibrated    << 5u;
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
        const motor_t *m = get_motor_by_id(cid);
        if (m != NULL) {
            float wrapped           = wrap_pi(m->pos);
            motors_rt[i].pos        = wrapped;
            motors_rt[i].pos_offset = m->pos - wrapped;
            motors_rt[i].vel        = m->rpm;
            motors_rt[i].tau        = m->torq;
            motors_rt[i].temp       = m->temperature;
            motors_rt[i].fault_flags = pack_faults(m);
        }

        /* Per-motor torque trip: while actively driving (hold or live MIT), if
           the measured torque exceeds this motor's max_tau, cut it to IDLE.
           apply_mit() is a no-op once idle, so the trip latches until the host
           re-arms the motor. */
        if ((motors_rt[i].state == MOTOR_ARMED_HOLD ||
             motors_rt[i].state == MOTOR_ARMED_MIT) &&
            fabsf(motors_rt[i].tau) > motors_rt[i].cfg->max_tau) {
            idle_motor(i);
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
                    motors_rt[i].state = MOTOR_IDLE;
                }
                break;

            case MOTOR_ZEROING: {
                float pos = motors_rt[i].pos;
                float abs_pos = pos < 0.0f ? -pos : pos;
                if (abs_pos < MOTOR_ZERO_TOL) {
                    /* Arrived at home. Reset the motor's mechanical zero HERE so
                       its multi-turn frame is 0 at home. Without this a motor
                       that has wound up sits near the ±4π position-report limit,
                       where the 16-bit position feedback wraps; the short-path
                       pos_offset then flips sign mid-motion and MIT commands run
                       away. Resetting at home removes the winding while keeping
                       the physical home reference (we are at home right now). */
                    can_disable_motor(cid, CAN_MASTER_ID);
                    HAL_Delay(2u);
                    can_set_mech_zero(cid, CAN_MASTER_ID);
                    HAL_Delay(2u);
                    can_change_motor_mode(cid, CAN_MASTER_ID, MIT_MODE);
                    HAL_Delay(2u);
                    can_enable_motor(cid, CAN_MASTER_ID);
                    motors_rt[i].pos         = 0.0f;
                    motors_rt[i].pos_offset  = 0.0f;
                    motors_rt[i].hold_pos    = 0.0f;
                    motors_rt[i].hold_vel    = 0.0f;
                    motors_rt[i].watchdog_ms = now_ms;
                    motors_rt[i].state       = MOTOR_ARMED_HOLD;
                } else {
                    /* Step waypoint toward zero at MOTOR_ZERO_RATE (rad/s),
                       scaled by the real loop period so the creep speed is
                       independent of loop frequency. */
                    float sign = (pos > 0.0f) ? -1.0f : 1.0f;
                    motors_rt[i].hold_pos += sign * MOTOR_ZERO_RATE * MOTOR_LOOP_DT_S;
                    /* Clamp — don't overshoot zero */
                    if (sign < 0.0f && motors_rt[i].hold_pos < 0.0f) motors_rt[i].hold_pos = 0.0f;
                    if (sign > 0.0f && motors_rt[i].hold_pos > 0.0f) motors_rt[i].hold_pos = 0.0f;
                    send_mit(i, 0.0f,
                             motors_rt[i].hold_pos,
                             sign * MOTOR_ZERO_RATE,
                             MOTOR_ZERO_KP, MOTOR_ZERO_KD);
                }
                if ((now_ms - motors_rt[i].watchdog_ms) > MOTOR_WATCHDOG_MS) {
                    can_disable_motor(cid, CAN_MASTER_ID);
                    motors_rt[i].state = MOTOR_IDLE;
                }
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
    }
}

HAL_StatusTypeDef motor_runtime_arm(uint8_t idx, uint16_t cmd_seq)
{
    if (idx >= N_MOTORS)                          return HAL_ERROR;
    if (motors_rt[idx].state != MOTOR_IDLE)       return HAL_ERROR;
    if (!motors_rt[idx].alive)                    return HAL_ERROR;

    uint8_t cid = motor_configs[idx].can_id;
    const motor_t *m = get_motor_by_id(cid);
    if (m == NULL) return HAL_ERROR;

    float wrapped = wrap_pi(m->pos);
    motors_rt[idx].pos        = wrapped;
    motors_rt[idx].pos_offset = m->pos - wrapped;
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

    motors_rt[idx].state       = MOTOR_ARMED_HOLD;
    motors_rt[idx].last_cmd_seq = cmd_seq;
    motors_rt[idx].watchdog_ms = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef motor_runtime_disable(uint8_t idx, uint16_t cmd_seq)
{
    if (idx >= N_MOTORS) return HAL_ERROR;
    idle_motor(idx);
    motors_rt[idx].last_cmd_seq = cmd_seq;
    return HAL_OK;
}

HAL_StatusTypeDef motor_runtime_goto_zero(uint8_t idx, uint16_t cmd_seq)
{
    if (idx >= N_MOTORS)                    return HAL_ERROR;
    if (motors_rt[idx].state != MOTOR_IDLE) return HAL_ERROR;
    if (!motors_rt[idx].alive)              return HAL_ERROR;

    uint8_t cid = motor_configs[idx].can_id;

    /* Unwind a motor pinned at the ±4π position-report limit BEFORE trying to
       creep it home. Such a motor can't be controlled — commands saturate at
       the edge and the 16-bit feedback wraps. A motor wound by whole turns sits
       at the same physical shaft angle as home, so resetting its mechanical zero
       in place loses nothing and brings the frame back near 0. (|offset| ≳ 3π
       targets the ±4π case only; a ±2π winding is still safely controllable.) */
    {
        const motor_t *mw = get_motor_by_id(cid);
        if (mw != NULL) {
            float off = mw->pos - wrap_pi(mw->pos);
            if (off > 9.0f || off < -9.0f) {
                can_set_mech_zero(cid, CAN_MASTER_ID);
                HAL_Delay(5u);
            }
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
       correctly. Wrap to the home frame so zeroing creeps the short way toward 0
       instead of unwinding a full turn from a wrapped power-up reading. */
    const motor_t *mz = get_motor_by_id(cid);
    if (mz != NULL) {
        float wrapped = wrap_pi(mz->pos);
        motors_rt[idx].pos        = wrapped;
        motors_rt[idx].pos_offset = mz->pos - wrapped;
    }
    motors_rt[idx].hold_pos = motors_rt[idx].pos;
    motors_rt[idx].hold_vel = 0.0f;

    /* First command: hold current position before the update loop starts stepping */
    send_mit(idx, 0.0f, motors_rt[idx].hold_pos, 0.0f,
             MOTOR_ZERO_KP, MOTOR_ZERO_KD);

    motors_rt[idx].state        = MOTOR_ZEROING;
    motors_rt[idx].last_cmd_seq = cmd_seq;
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
    if (pos > hi) {
        pos = hi;
        if (vel > 0.0f) vel = 0.0f;
    } else if (pos < lo) {
        pos = lo;
        if (vel < 0.0f) vel = 0.0f;
    }

    r->hold_pos = pos;
    r->hold_vel = vel;
    r->state    = MOTOR_ARMED_MIT;
}

void motor_runtime_pack_tele(SpiMotorTele *out, uint8_t idx)
{
    if (idx >= N_MOTORS || out == NULL) return;
    const MotorRuntime *r = &motors_rt[idx];
    out->motor_idx    = idx;
    out->state        = (uint8_t)r->state;
    out->fault_flags  = r->fault_flags;
    out->pos_raw      = f_to_u16(r->pos,  MOTOR_P_MIN, MOTOR_P_MAX);
    out->vel_raw      = f_to_u16(r->vel,  MOTOR_V_MIN, MOTOR_V_MAX);
    out->tau_raw      = f_to_u16(r->tau,  MOTOR_T_MIN, MOTOR_T_MAX);
    out->last_cmd_seq = r->last_cmd_seq;
    out->temp_c       = (uint8_t)(r->temp < 0.0f ? 0u : (uint8_t)r->temp);
}

uint8_t motor_runtime_motors_alive(void)
{
    uint8_t mask = 0u;
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        if (motors_rt[i].alive) mask |= (uint8_t)(1u << i);
    }
    return mask;
}
