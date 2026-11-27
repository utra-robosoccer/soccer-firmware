#include "motor_runtime.h"
#include "can_rx.h"
#include "robstride.h"
#include "enable_monitor.h"
#include "../../../../common/include/drive_sm.h"           /* pure drive machine */
#include "../../../../common/include/master_watchdog.h"   /* master-loss grace phase */
#include "../../../../common/include/motor_soft_limit.h"  /* one-sided soft-limit clamp */
#include "../../../../common/include/arm_sm.h"            /* ENABLING CAN sequencing */
#include "../../../../common/include/motor_rs_can.h"      /* RsIdSweepResult */
#include <math.h>
#include <string.h>

MotorRuntime motors_rt[N_MOTORS];

/* ── helpers ─────────────────────────────────────────────────────────────── */

/* Wrap/stale-safe elapsed-time test: signed difference so a timestamp stamped
   after `now` was captured reads negative ("not yet"), not huge-unsigned. */
static inline int32_t ms_since(uint32_t now, uint32_t t) { return (int32_t)(now - t); }

/* Send an MIT command. Positions are the motor's own single-turn frame: every
   motor is zero_sta-verified at boot (power-on reading in [-pi, pi]), so the
   raw encoder position IS the joint angle — no wrap, no offset bookkeeping.
   Opens the cmd_seq reply window for this motor. */
static void send_mit(uint8_t idx, float torque, float pos, float vel,
                     float kp, float kd)
{
    rs_mit_control_set(motor_user_configs[idx].id.can_id, torque,
                        pos, vel, kp, kd);
    cmd_seq_on_mit_frame(&motors_rt[idx].cmd_track, motors_rt[idx].target_cmd_seq);
}

/* Poll a motor that must stay off (DISABLED / FAULT): a zero-gain zero-torque
   Type-1 — the only frame every model answers while disabled (bench: RS00s are
   silent to a plain Type-4) — elicits the Type-2 that feeds the
   mode/fault/position mirrors. The poll also makes a runaway limp (zero
   gains); the re-assert-stop Type-4 for a motor seen off-Reset is sent at the
   HEAD of the burst by motor_runtime_update, never here: queued mid-burst it
   loses one-shot arbitration to polls/replies at the same phase every tick. */
static void off_poll(uint8_t idx)
{
    rs_mit_control_set(motor_user_configs[idx].id.can_id,
                       0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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
   bounded to cfg->user.v_max, and |tau_ff| to cfg->user.t_max (the overtorque trip threshold).
   Returns MOTOR_FLAG_* bits. */
static uint8_t apply_soft_clamp(const MotorRuntime *r, float *pos, float *vel, float *tau_ff)
{
    uint8_t c = soft_clamp_mit(pos, vel, tau_ff, r->pos,
                               r->cfg->user.p_min, r->cfg->user.p_max,
                               r->cfg->user.v_max, r->cfg->user.t_max);
    uint8_t flags = 0u;
    if (c & SOFT_CLAMPED_POS) flags |= MOTOR_FLAG_CLAMPED_POS;
    if (c & SOFT_CLAMPED_FF)  flags |= MOTOR_FLAG_CLAMPED_TAU;
    return flags;
}

/* Clamp wire-sourced gains to this motor's per-model CAN encode range. The CAN encoder
   (rs_mit_control_set) would saturate them silently; clamping here keeps the runtime
   state consistent with what is actually sent and reports it (MOTOR_FLAG_CLAMPED_GAIN). */
static uint8_t clamp_gains(uint8_t can_id, float *kp, float *kd)
{
    const MotorRsConfig *rg = motor_rs_config_by_id(can_id);
    if (rg == NULL) rg = rs_default_ranges();
    uint8_t c = 0u;
    c |= soft_clamp_range(kp, rg->kp_min, rg->kp_max);
    c |= soft_clamp_range(kd, rg->kd_min, rg->kd_max);
    return c ? (uint8_t)MOTOR_FLAG_CLAMPED_GAIN : 0u;
}

/* Enter a new drive state, stamping the transient-state clock. */
static void enter_state(uint8_t idx, uint8_t state, uint32_t now_ms)
{
    motors_rt[idx].state    = state;
    motors_rt[idx].state_ms = now_ms;
}

/* ENABLING→ENABLED edge: apply the mit latched during ENABLING this cycle
   (all-zero = limp — zero gains and zero torque apply no effort but keep the
   Type-2 feedback, enable monitor and fb timeout alive). */
static void enter_enabled(uint8_t idx, uint32_t now_ms)
{
    MotorRuntime *r = &motors_rt[idx];
    arm_sm_cancel(&r->arm_phase, &r->arm_wait);  /* Run confirmed — sends are done */
    enable_monitor_reset(idx);
    r->watchdog_ms = now_ms;
    r->armed_ms    = now_ms;   /* fb-timeout grace: a just-disabled motor has a
                                  stale last_fb; don't trip until it can reply. */
    enter_state(idx, MOTOR_ENABLED, now_ms);
    send_mit(idx, r->cmd_tau_ff, r->hold_pos, r->hold_vel, r->cmd_kp, r->cmd_kd);
}

/* A fault was DETECTED: let the pure machine pick the reaction (damp vs emergency),
   latch first-wins, and apply the actions. */
static void fault_detected(uint8_t idx, uint8_t fault, uint32_t now_ms)
{
    MotorRuntime *r = &motors_rt[idx];
    DriveFaultDecision f = drive_sm_on_fault(r->state, fault,
                                             (uint8_t)(r->fault != MOTOR_FAULT_NONE));
    if (f.latch_fault != MOTOR_FAULT_NONE) r->fault = f.latch_fault;
    if (f.abort_enable) arm_sm_cancel(&r->arm_phase, &r->arm_wait);
    if (f.send_stop) rs_disable_motor(motor_user_configs[idx].id.can_id, CAN_MASTER_ID);
    if (f.begin_stop) {
        r->hold_vel   = 0.0f;
        r->cmd_tau_ff = 0.0f;
    }
    if (f.next_state != r->state) enter_state(idx, f.next_state, now_ms);
}

/* ── boot CAN-id sweep ───────────────────────────────────────────────────────
   Probe every possible id with a Type-0 get-id — ONE AT A TIME. An unpaced
   blast saturates the bus and some units drop their probe under that load
   (bench: RS00 ids 3,4 never answered the burst but always answer a paced
   probe; replies land ~0.85 ms after the probe). The RX ISR marks replies in
   can_rx's bitmap; the bus is then judged against the configured roster.
   Configured ids still missing are probed ONCE more before latching — CAN TX
   is one-shot, and a single lost frame must not latch the block. The verdict
   latches until the next SLAVE_REQ_RESET re-sweep (or reboot). */
static RsIdSweepResult g_sweep;

static void can_id_sweep(void)
{
    uint8_t seen[32];
    uint8_t cfg_ids[N_MOTORS];
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        cfg_ids[i] = motor_user_configs[i].id.can_id;
    }

    /* 0 is not a valid motor id; 0xFD is the master; 0xFE/0xFF stay reserved. */
    for (uint16_t id = 1u; id <= 0xFCu; id++) {
        rs_get_motor_id((uint8_t)id, CAN_MASTER_ID);
        HAL_Delay(CAN_ID_SWEEP_PROBE_GAP_MS);
    }
    can_rx_sweep_snapshot(seen);
    g_sweep = rs_id_sweep_check(seen, cfg_ids, N_MOTORS);

    uint8_t all = (uint8_t)((1u << N_MOTORS) - 1u);
    if (g_sweep.found_mask != all) {
        for (uint8_t i = 0; i < N_MOTORS; i++) {
            if (!((g_sweep.found_mask >> i) & 1u)) {
                rs_get_motor_id(cfg_ids[i], CAN_MASTER_ID);
                HAL_Delay(CAN_ID_SWEEP_PROBE_GAP_MS);
            }
        }
        can_rx_sweep_snapshot(seen);
        g_sweep = rs_id_sweep_check(seen, cfg_ids, N_MOTORS);
    }
}

/* Boot zero_sta check: every motor must report zero_sta (0x7029) == 1 — the
   power-on-position-in-[-pi,pi] convention, written once via the host tools —
   before the slave may arm. CAN TX is one-shot, so motors that have not
   replied are re-asked, up to 3 rounds: one dropped frame must not
   false-latch the block. */
static uint8_t g_zero_sta_mask;   /* bit i = motor i replied with zero_sta == 1 */
static uint8_t g_fw_ver_mask;     /* bit i = motor i's fw version matched tier 1 */
static uint8_t g_boot_ok;         /* sweep AND zero_sta AND fw version, all motors */
static boot_result_t g_boot_result;

static void zero_sta_check(void)
{
    uint32_t raw;
    for (uint8_t round = 0; round < 3u; round++) {
        uint8_t sent = 0u;
        for (uint8_t i = 0; i < N_MOTORS; i++) {
            if (!can_rx_zero_sta(i, &raw)) {
                rs_read_single_param(motor_user_configs[i].id.can_id,
                                     CAN_MASTER_ID, 0x7029u);
                sent = 1u;
            }
        }
        if (!sent) break;
        HAL_Delay(BOOT_ZERO_STA_WINDOW_MS);
    }
    g_zero_sta_mask = 0u;
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        if (can_rx_zero_sta(i, &raw) && raw == 1u) {
            g_zero_sta_mask |= (uint8_t)(1u << i);
        }
    }
}

/* Boot fw-version check: every motor's reply to the Type-4 version-read
   (Byte0=0, Byte1=0xC4 — safe: sent only here, while the motor is in Reset)
   must echo 00,C4 and match its model's expected bytes in tier 1
   (motor_rs_configs[].fw_version) EXACTLY. A mismatch — wrong build, wrong
   model on this id, or a model whose bytes were never recorded — latches the
   slave BLOCKED, same as zero_sta. One motor at a time with its own reply
   window (the version reply is comm-type-2 and only the window tells it from
   feedback), up to 3 attempts; raw replies stay gdb-visible in can_rx. */
static void fw_version_check(void)
{
    uint8_t raw[8];
    g_fw_ver_mask = 0u;
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        for (uint8_t attempt = 0;
             attempt < 3u && !can_rx_fw_ver(i, raw); attempt++) {
            can_rx_fw_ver_arm(i);
            rs_read_fw_version(motor_user_configs[i].id.can_id, CAN_MASTER_ID);
            HAL_Delay(BOOT_FW_VER_WINDOW_MS);
        }
        if (can_rx_fw_ver(i, raw)) {
            const uint8_t *exp =
                motor_rs_configs[motor_user_configs[i].id.model].fw_version;
            if (raw[0] == 0x00u && raw[1] == 0xC4u &&
                memcmp(&raw[2], exp, RS_FW_VERSION_LEN) == 0) {
                g_fw_ver_mask |= (uint8_t)(1u << i);
            }
        }
    }
    can_rx_fw_ver_disarm_all();   /* close every window before polling starts */
}

/* Run the full blocking boot checks and re-enter every motor's resting state.
   Shared by boot and the SLAVE_REQ_RESET re-sweep. */
static void run_boot_checks(void)
{
    can_id_sweep();
    g_zero_sta_mask = 0u;
    g_fw_ver_mask   = 0u;
    if (g_sweep.ok) {
        zero_sta_check();     /* roster wrong → skip; the slave is blocked anyway */
        fw_version_check();
    }
    uint8_t all = (uint8_t)((1u << N_MOTORS) - 1u);
    g_boot_ok = (uint8_t)(g_sweep.ok &&
                          g_zero_sta_mask == all &&
                          g_fw_ver_mask == all);
    g_boot_result.found_mask         = g_sweep.found_mask;
    g_boot_result.unexpected_id      = g_sweep.unexpected_id;
    /* The verify masks are only meaningful when the checks ran (sweep ok). */
    g_boot_result.zero_sta_fail_mask = g_sweep.ok ? (uint8_t)(all & ~g_zero_sta_mask) : 0u;
    g_boot_result.version_fail_mask  = g_sweep.ok ? (uint8_t)(all & ~g_fw_ver_mask)   : 0u;

    uint32_t now_ms = HAL_GetTick();
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        if ((g_sweep.found_mask >> i) & 1u) {
            /* Seed the feedback clock: the sweep reply proved the motor alive,
               but no Type-2 exists yet and DISABLED's first ticks run before
               the first ready poll is answered — an unseeded last_fb_ms would
               read as a 100 ms timeout and drop the motor to terminal ABSENT.
               (can_rx seeds its motors[] copy on the same Type-0, so the
               snapshot mirrored by on_feedback agrees until real Type-2s.) */
            motors_rt[i].last_fb_ms = now_ms;
            enter_state(i, MOTOR_DISABLED, now_ms);
        } else {
            enter_state(i, MOTOR_ABSENT, now_ms);
        }
    }
}

/* ENABLING CAN-send pacing (cycles): settle after the run-mode write and after the
   enable. Completion is NOT wait-based — the ENABLING→ENABLED edge fires on the
   first feedback showing rs_mode == Run after the enable went out (or times out
   into MOTOR_FAULT_ENABLE_FAILED), so a fast ACK advances early. */
#define ARM_SETTLE_WAIT_CYCLES ((uint8_t)((20u * SLAVE_TICK_HZ + 999u) / 1000u))

/* The verified enable gate: the boot checks must have passed, and THIS motor
   must be DISABLED with a fresh Type-2 from the ready poll showing Reset mode,
   no fault bits, and a shaft position inside the soft limits plus the ready
   margin (a joint parked outside its envelope is a rigging problem: refuse,
   don't fight it in). fb_seen guards the mirrors: motor_mode defaults to
   0 == Reset at boot, so without at least one real Type-2 the checks would
   pass vacuously. Also telemetry's MOTOR_FLAG_READY. */
static uint8_t enable_ready(const MotorRuntime *r, uint32_t now_ms)
{
    return (uint8_t)(g_boot_ok &&
                     r->state == MOTOR_DISABLED &&
                     r->fb_seen &&
                     r->motor_mode == RS_MODE_RESET &&
                     r->motor_fault == 0u &&
                     ms_since(now_ms, r->last_fb_ms) <
                         (int32_t)MOTOR_CAN_FB_TIMEOUT_MS &&
                     r->pos >= r->cfg->user.p_min - MOTOR_READY_POS_MARGIN_RAD &&
                     r->pos <= r->cfg->user.p_max + MOTOR_READY_POS_MARGIN_RAD);
}

/* ── public API ──────────────────────────────────────────────────────────── */

void motor_runtime_init(void)
{
    memset(motors_rt, 0, sizeof(motors_rt));
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        motors_rt[i].cfg   = &motor_user_configs[i];
        motors_rt[i].state = MOTOR_ABSENT;
        motors_rt[i].fault = MOTOR_FAULT_NONE;
    }
    run_boot_checks();
}

void motor_runtime_boot_result(boot_result_t *out)
{
    if (out != NULL) *out = g_boot_result;
}

uint8_t motor_runtime_boot_ok(void)
{
    return g_boot_ok;
}

uint8_t motor_runtime_any_armed(void)
{
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        uint8_t st = motors_rt[i].state;
        if (st == MOTOR_ENABLING || st == MOTOR_ENABLED || st == MOTOR_STOPPING)
            return 1u;
    }
    return 0u;
}

uint8_t motor_runtime_reset_clean(void)
{
    if (!g_boot_ok) return 0u;
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        const MotorRuntime *r = &motors_rt[i];
        if (r->state != MOTOR_DISABLED || r->fault != MOTOR_FAULT_NONE ||
            r->motor_fault != 0u || r->motor_mode == RS_MODE_CALI)
            return 0u;
    }
    return 1u;
}

/* One blocking SLAVE_REQ_RESET round. The caller guarantees nothing is armed
   and has pre-staged the RESETTING telemetry frame (SPI exchanges keep running
   from the ISRs while this blocks). */
void motor_runtime_reset_round(void)
{
    /* Stop + clear each motor first, paced on the quiet bus (the control loop
       is blocked here, so no polls/replies compete for arbitration — the
       clears actually reach the wire, which is what head-of-burst placement
       buys during normal servicing). */
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        rs_clear_fault(motor_user_configs[i].id.can_id, CAN_MASTER_ID);
        motor_set_fault_word(i, 0u);
        HAL_Delay(CAN_ID_SWEEP_PROBE_GAP_MS);
    }

    /* Re-arm the captures and re-run the full boot checks (sweep + verify). */
    can_rx_boot_rearm();
    run_boot_checks();

    /* Provoke a fresh Type-2 per found motor so the dirty/clean judgement runs
       against post-clear state, not boot leftovers. */
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        if ((g_boot_result.found_mask >> i) & 1u) {
            off_poll(i);
            HAL_Delay(CAN_ID_SWEEP_PROBE_GAP_MS);
        }
    }
    HAL_Delay(CAN_ID_SWEEP_PROBE_GAP_MS);

    uint32_t now_ms = HAL_GetTick();
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        MotorRuntime *r = &motors_rt[i];

        /* Rebaseline the per-motor runtime: tuples, trackers, flags. */
        arm_sm_cancel(&r->arm_phase, &r->arm_wait);
        cmd_seq_reset(&r->cmd_track);
        r->target_cmd_seq    = CMD_SEQ_NONE;
        r->hold_pos = r->hold_vel = r->cmd_kp = r->cmd_kd = r->cmd_tau_ff = 0.0f;
        r->last_apply_clamp  = 0u;
        r->cmd_flags_latched = 0u;
        r->got_fresh_cmd     = 0u;
        r->req_rejected      = 0u;
        r->fault             = MOTOR_FAULT_NONE;
        r->watchdog_ms       = now_ms;

        if (!((g_boot_result.found_mask >> i) & 1u)) continue;   /* ABSENT */

        motor_t snap;
        motor_get_snapshot(i, &snap);
        r->pos         = snap.pos;
        r->vel         = snap.rpm;
        r->tau         = snap.torq;
        r->temp        = snap.temperature;
        r->motor_mode  = (uint8_t)snap.status;
        r->motor_fault = pack_faults(&snap);
        r->fault_word  = snap.fault_word;
        r->last_fb_ms  = snap.last_fb_ms;
        r->last_fb_us  = snap.last_fb_us;
        r->fb_seen     = (uint8_t)(snap.fb_count != 0u);
        if (r->motor_fault != 0u || r->motor_mode == RS_MODE_CALI ||
            r->motor_mode == RS_MODE_RUN) {
            /* Still dirty after the clear (or refusing to leave Run): keep it
               latched; the host sees it and can hold RESET for another round. */
            r->fault = MOTOR_FAULT_RS_REPORTED;
            enter_state(i, MOTOR_FAULT, now_ms);
        }
        /* else: run_boot_checks already left it DISABLED. */
    }
}

void motor_runtime_apply_cmd(uint8_t idx, const cmd_motor_t *m, uint16_t cmd_seq)
{
    if (idx >= N_MOTORS || m == NULL) return;

    MotorRuntime *r = &motors_rt[idx];
    uint8_t cid = motor_user_configs[idx].id.can_id;
    uint32_t now_ms = HAL_GetTick();

    /* Enable gate: rejected unless the motor is verified ready (see
       enable_ready). A rejected enable surfaces as MOTOR_FLAG_REQ_REJECTED and
       the motor stays DISABLED — no fault; STOP flows normally. */
    uint8_t gate_reject = (uint8_t)(m->req == MOTOR_REQ_ENABLE &&
                                    r->state == MOTOR_DISABLED &&
                                    !enable_ready(r, now_ms));
    DriveReqDecision d;
    if (gate_reject) {
        memset(&d, 0, sizeof d);
        d.rejected   = 1u;
        d.next_state = r->state;
    } else {
        d = drive_sm_on_request(r->state, m->req,
                                (uint8_t)(r->fault != MOTOR_FAULT_NONE));
    }
    r->req_rejected = d.rejected;

    if (d.abort_enable) arm_sm_cancel(&r->arm_phase, &r->arm_wait);
    if (d.send_stop) rs_disable_motor(cid, CAN_MASTER_ID);
    if (d.begin_enable) {
        /* Handshake: run-mode write, then enable. */
        arm_sm_begin(&r->arm_phase, &r->arm_wait);
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

    /* The mit applies EVERY cycle (all-zero = limp): decoded + clamped while
       ENABLED, or latched during ENABLING and applied on the edge. */
    if (r->state == MOTOR_ENABLED || r->state == MOTOR_ENABLING) {
        r->hold_pos   = proto_i16_to_f(m->mit.pos, PROTO_POS_SCALE);
        r->hold_vel   = proto_i16_to_f(m->mit.vel, PROTO_VEL_SCALE);
        r->cmd_tau_ff = proto_i16_to_f(m->mit.tau_ff, PROTO_TAU_SCALE);
        r->last_apply_clamp = apply_soft_clamp(r, &r->hold_pos, &r->hold_vel, &r->cmd_tau_ff);
        r->cmd_kp = proto_u16_to_f(m->mit.kp, PROTO_KP_SCALE);
        r->cmd_kd = proto_u16_to_f(m->mit.kd, PROTO_KD_SCALE);
        r->last_apply_clamp |= clamp_gains(cid, &r->cmd_kp, &r->cmd_kd);
        r->target_cmd_seq = cmd_seq;
        r->got_fresh_cmd  = 1u;
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

        motors_rt[i].pos         = snap.pos;  /* single-turn frame, zero_sta-verified */
        motors_rt[i].vel         = snap.rpm;
        motors_rt[i].tau         = snap.torq;
        motors_rt[i].temp        = snap.temperature;
        motors_rt[i].motor_mode  = (uint8_t)snap.status;
        motors_rt[i].motor_fault = pack_faults(&snap);
        motors_rt[i].fault_word  = snap.fault_word;
        motors_rt[i].last_fb_ms  = snap.last_fb_ms;
        motors_rt[i].last_fb_us  = snap.last_fb_us;
        motors_rt[i].fb_seen     = (uint8_t)(snap.fb_count != 0u);

        uint8_t driving = (motors_rt[i].state == MOTOR_ENABLED ||
                           motors_rt[i].state == MOTOR_STOPPING);

        uint8_t mon_fresh = (snap.fb_count != motors_rt[i].mon_prev_fb_count);
        motors_rt[i].mon_verdict = (uint8_t)enable_monitor_step(
            driving, mon_fresh, (uint8_t)(snap.status == RS_MODE_RUN),
            MOTOR_ENABLE_MON_K, &motors_rt[i].mon_not_enabled);
        if (mon_fresh) {
            motors_rt[i].mon_prev_fb_count = snap.fb_count;
            cmd_seq_on_reply(&motors_rt[i].cmd_track);   /* pairs this reply with its MIT */
        }
    }
}

void motor_runtime_update(uint32_t now_ms, uint8_t cmd_fresh)
{
    /* Service ENABLING motors FIRST. CAN TX is one-shot (no retransmission), and a
       handshake frame sent mid-burst can lose arbitration to a Type-2 reply elicited
       by an earlier frame of the same burst (replies have a lower ext-ID than Type-3
       enables) — the loser is silently dropped, at the same phase every cycle, which
       starved one bench motor's enable indefinitely. At the head of the burst the bus
       is idle, so the handshake frame always wins. */
    uint8_t order[N_MOTORS];
    uint8_t n = 0;
    for (uint8_t i = 0; i < N_MOTORS; i++)
        if (motors_rt[i].state == MOTOR_ENABLING) order[n++] = i;
    for (uint8_t i = 0; i < N_MOTORS; i++)
        if (motors_rt[i].state != MOTOR_ENABLING) order[n++] = i;

    /* Repair pre-pass, FIRST frames of the burst: a motor that must be off but
       whose Type-2 shows Run/Cali gets a Type-4 stop here, while the bus is
       idle. A Type-4 ext-id (0x04…) loses one-shot arbitration to Type-1 polls
       (0x01…) and Type-2 replies (0x02…), so queued mid-burst the stop was
       dropped at the same phase every tick — bench: a stuck-Run motor
       swallowed 200 re-stops/s with none reaching the wire. This is also the
       EMERGENCY re-send rule: a MOTOR_FAULT motor is re-stopped until its
       feedback confirms RS_MODE_RESET. */
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        MotorRuntime *r = &motors_rt[i];
        if ((r->state == MOTOR_DISABLED || r->state == MOTOR_FAULT) &&
            r->motor_mode != RS_MODE_RESET) {
            rs_disable_motor(motor_user_configs[i].id.can_id, CAN_MASTER_ID);
        }
    }

    for (uint8_t k = 0; k < N_MOTORS; k++) {
        uint8_t i = order[k];
        MotorRuntime *r = &motors_rt[i];
        uint8_t cid = motor_user_configs[i].id.can_id;

        /* ── fault detection on the mirrored feedback state (first trip latches).
           Feedback-timeout is time-based (feedback absence) so it belongs here in
           the per-cycle service; the armed_ms grace covers the first cycles after
           an enable (a just-disabled motor's last_fb is stale). ─────────────── */
        uint8_t live = (uint8_t)(r->state == MOTOR_ENABLED || r->state == MOTOR_STOPPING);
        if (live &&
            ms_since(now_ms, r->last_fb_ms) >= (int32_t)MOTOR_CAN_FB_TIMEOUT_MS &&
            ms_since(now_ms, r->armed_ms)   >= (int32_t)MOTOR_CAN_FB_TIMEOUT_MS) {
            fault_detected(i, MOTOR_FAULT_FEEDBACK_TIMEOUT, now_ms);
        } else if ((live || r->state == MOTOR_ENABLING) && r->motor_fault != 0u) {
            fault_detected(i, MOTOR_FAULT_RS_REPORTED, now_ms);
            motor_set_fault_word(i, 0xFFFFFFFFu);
            r->fault_word = 0xFFFFFFFFu;
            rs_read_single_param(cid, CAN_MASTER_ID, 0x3022u);
        } else if (r->state == MOTOR_ENABLED &&
                   r->mon_verdict == ENABLE_MON_FAULT_NOT_ENABLED) {
            fault_detected(i, MOTOR_FAULT_LOST_ENABLE, now_ms);
        } else if (r->state == MOTOR_ENABLED &&
                   fabsf(r->tau) > r->cfg->user.t_max) {
            fault_detected(i, MOTOR_FAULT_OVERTORQUE, now_ms);
        } else if (r->state == MOTOR_DISABLED && MOTOR_FAULT_CHECK_DISABLED &&
                   (r->motor_fault != 0u || r->motor_mode == RS_MODE_CALI)) {
            /* Vendor bits / Cali at rest — masked off by default until bench
               boot-rest data is clean. */
            fault_detected(i, MOTOR_FAULT_RS_REPORTED, now_ms);
        }

        /* ── master loss: no fresh command this cycle. Both ENABLING and ENABLED
           get the hold grace (a brief SPI hiccup or missed exchange recovers
           transparently); past the grace, ENABLED enters the autonomous damp and
           ENABLING aborts, both → MOTOR_FAULT(MASTER_LOST). STOPPING runs to
           completion on its own clock. ─────────────────────────────────────── */
        if (!cmd_fresh &&
            (r->state == MOTOR_ENABLING || r->state == MOTOR_ENABLED)) {
            uint32_t since = (uint32_t)ms_since(now_ms, r->watchdog_ms);
            if (master_loss_phase(since, MASTER_LOST_GRACE_MS, MASTER_LOST_DAMP_MS)
                    != MASTER_HOLD_GRACE) {
                fault_detected(i, MOTOR_FAULT_MASTER_LOST, now_ms);
            } else if (r->state == MOTOR_ENABLED) {
                /* Grace: hold the last setpoint with v_des/tau_ff zeroed (a limp
                   command — zero gains — just stays limp). */
                send_mit(i, 0.0f, r->hold_pos, 0.0f, r->cmd_kp, r->cmd_kd);
                continue;       /* grace frame sent; skip the normal service */
            }
            /* ENABLING in grace: the handshake is autonomous — keep servicing. */
        }

        /* ── per-state service: one CAN frame per motor per cycle + drive_sm tick. */
        DriveTickInputs in;
        memset(&in, 0, sizeof(in));
        in.fault_latched = (uint8_t)(r->fault != MOTOR_FAULT_NONE);

        switch (r->state) {
        case MOTOR_ENABLING:
            if (arm_sm_active(r->arm_phase)) {
                uint8_t act = arm_sm_step(&r->arm_phase, &r->arm_wait,
                                          ARM_SETTLE_WAIT_CYCLES);
                if (act == ARM_ACT_SEND_MODE) {
                    rs_change_motor_mode(cid, CAN_MASTER_ID, MIT_MODE);
                } else if (act == ARM_ACT_SEND_ENABLE) {
                    rs_enable_motor(cid, CAN_MASTER_ID);
                    r->enable_sent_ms = now_ms;
                }
                /* ARM_ACT_COMPLETE: sends exhausted; Run confirmation decides. */
            } else {
                /* Waiting on the Run confirmation: RE-SEND the handshake, don't
                   just poll. Slave CAN TX is one-shot (no retransmission), so a
                   single lost mode-write or enable would otherwise sink the whole
                   handshake on a marginal drop — the sends are level-triggered
                   and idempotent, and both elicit the confirming Type-2 reply.
                   Alternate so a lost mode write is also repaired. */
                /* arm_wait is free once arm_sm is idle — reuse it as the toggle. */
                r->arm_wait ^= 1u;
                if (r->arm_wait & 1u) {
                    rs_enable_motor(cid, CAN_MASTER_ID);
                    r->enable_sent_ms = now_ms;
                } else {
                    rs_change_motor_mode(cid, CAN_MASTER_ID, MIT_MODE);
                }
            }
            in.run_seen = (uint8_t)(r->enable_sent_ms != 0u &&
                                    r->motor_mode == RS_MODE_RUN &&
                                    (int32_t)(r->last_fb_ms - r->enable_sent_ms) >= 0);
            in.enable_timeout =
                (uint8_t)(ms_since(now_ms, r->state_ms) >= (int32_t)MOTOR_ENABLE_TIMEOUT_MS);
            break;

        case MOTOR_ENABLED:
            /* Apply the current mit every cycle (all-zero = limp: zero gains
               and zero torque apply no effort but keep the Type-2 feedback,
               enable monitor and fb timeout alive). */
            send_mit(i, r->cmd_tau_ff, r->hold_pos, r->hold_vel,
                     r->cmd_kp, r->cmd_kd);
            break;

        case MOTOR_STOPPING:
            /* Controlled stop: the per-motor fallback MIT constants (tier 3 of
               the config — normally kp=0 for a pure backdrive-safe damp) for the
               damp dwell, then the ramp owns its stop; drive_sm picks DISABLED
               vs FAULT. */
            if (ms_since(now_ms, r->state_ms) >= (int32_t)MASTER_LOST_DAMP_MS) {
                rs_disable_motor(cid, CAN_MASTER_ID);
                in.stop_done = 1u;
            } else {
                send_mit(i, r->cfg->fallback.tau_ff, r->pos, 0.0f,
                         r->cfg->fallback.kp, r->cfg->fallback.kd);
            }
            break;

        case MOTOR_DISABLED:
            /* Ready-mode poll: provokes the Type-2 whose Reset mode / fault
               bits / position the enable gate verifies; the repair pre-pass
               re-asserts the stop whenever that Type-2 shows not-Reset. */
            off_poll(i);
            in.fb_timeout =
                (uint8_t)(ms_since(now_ms, r->last_fb_ms) >= (int32_t)MOTOR_CAN_FB_TIMEOUT_MS);
            break;

        case MOTOR_FAULT:
            /* Same poll: keeps telemetry fresh in FAULT and feeds the repair
               pre-pass's Reset confirmation (EMERGENCY re-send rule). */
            off_poll(i);
            break;

        case MOTOR_ABSENT:
            /* Terminal: presence was judged by the id sweep. No re-ping —
               fix the bus, then SLAVE_REQ_RESET (or power-cycle). */
            break;

        default:
            break;
        }

        /* Step the pure machine on the summarised inputs. */
        {
            DriveTickDecision t = drive_sm_on_tick(r->state, &in);
            if (t.latch_fault != MOTOR_FAULT_NONE && r->fault == MOTOR_FAULT_NONE)
                r->fault = t.latch_fault;
            if (t.send_stop) rs_disable_motor(cid, CAN_MASTER_ID);
            if (t.enter_enabled) {
                enter_enabled(i, now_ms);
            } else if (t.next_state != r->state) {
                enter_state(i, t.next_state, now_ms);
            }
        }

        /* Assemble the per-tick telemetry cmd flags (clamp while fresh, else stale). */
        {
            uint8_t cf = 0u;
            if (r->state == MOTOR_ENABLED) {
                cf = r->got_fresh_cmd ? r->last_apply_clamp
                                      : (uint8_t)MOTOR_FLAG_CMD_STALE;
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
    out->rs.pos  = proto_f_to_i16(r->pos,  PROTO_POS_SCALE,  &sat);
    out->rs.vel  = proto_f_to_i16(r->vel,  PROTO_VEL_SCALE,  &sat);
    out->rs.tau  = proto_f_to_i16(r->tau,  PROTO_TAU_SCALE,  &sat);
    out->rs.temp = proto_f_to_i16(r->temp, PROTO_TEMP_SCALE, &sat);
    out->rs.mode          = r->motor_mode;
    out->rs.fault_bits    = r->motor_fault;
    out->rs.fault_word    = r->fault_word;
    out->rs.time_slave_us = r->last_fb_us;

    out->state = r->state;
    out->fault = r->fault;

    uint8_t flags = r->cmd_flags_latched;
    if (enable_ready(r, HAL_GetTick())) flags |= MOTOR_FLAG_READY;
    if (r->req_rejected)                flags |= MOTOR_FLAG_REQ_REJECTED;
    if (sat)                            flags |= MOTOR_FLAG_SATURATED;
    out->flags = flags;

    out->last_applied_seq = r->cmd_track.last_applied;
}

uint8_t motor_runtime_motors_alive(void)
{
    uint8_t mask = 0u;
    for (uint8_t i = 0; i < N_MOTORS; i++) {
        if (motors_rt[i].state > MOTOR_ABSENT) mask |= (uint8_t)(1u << i);
    }
    return mask;
}
