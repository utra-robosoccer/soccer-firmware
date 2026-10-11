#!/usr/bin/env python3
"""Generate motor configuration files from a robot's firmware config.

Config source
-------------
Each robot is described by ONE artifact exported from the soccer-software repo:
``configs/<setup>/firmware_config.yaml`` (imported + hash-verified by
scripts/import_firmware_config.py). It covers every slave in one file —
``slaves: [{slave, motors: [...]}, ...]`` plus shared ``rates``, ``shared_ranges``
and ``models``. There are NO per-slave intermediate files: ``--slave`` selects a
slave's section and ``--system`` reads the whole file.

Which setup is live is selected in ONE place: ``configs/active`` (a single line
naming a subdirectory of configs/), overridable per-invocation by ``SOCCER_SETUP``.

A setup that has not yet been migrated to the artifact (no firmware_config.yaml)
falls back to the LEGACY per-slave ``configs/<setup>/slaveN.yaml`` files.

Two modes:

  --slave  <slaveN | path.yaml>   (default for a bare positional argument)
      Emits firmware/common/include/motor_config.h for THAT ONE slave
      (N_MOTORS, transport bounds, MotorModel enum, motor_can_ranges[],
      motor_configs[], motor_can_range_by_id()). A bare name selects the section
      of the active setup's firmware_config.yaml; an explicit .yaml path is a
      legacy slaveN.yaml. See scripts/build.sh --config.

  --system [<a.yaml> ...]
      With no args, reads the active setup's firmware_config.yaml (all slaves),
      or every legacy slave*.yaml. Emits, covering ALL slaves:
        - firmware/common/include/system_config.h  (master: NUM_SLAVES,
          per-slave motor counts + CAN-id LUTs, global transport bounds)
        - host/master_link/motor_config_gen.py      (host: per-slave SLAVES[]
          plus a flattened view for the dashboard / test_client)

Usage:
    python3 scripts/gen_motor_config.py                    # regen everything for active setup
    python3 scripts/gen_motor_config.py --slave  slave0    # active setup's slave0 section
    python3 scripts/gen_motor_config.py --system           # active setup, all slaves
    SOCCER_SETUP=2s_10m python3 scripts/gen_motor_config.py --system   # one-off setup override

The firmware build does not run Python, so generated files are committed
alongside the source. Re-run after importing a new artifact, then rebuild.
"""

import glob
import hashlib
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# TO_ZERO motion parameters (motion tuning, not per-motor identity). Tick-based
# constants are DERIVED from the configured rates below, not hard-coded here.
GOTO_ZERO = {
    "MOTOR_ZERO_TOL": "0.05f",   # rad (~3 deg) — arrival position threshold
    "MOTOR_ZERO_RATE": "0.3f",   # rad/s        — constant approach speed
    "MOTOR_ZERO_KP": "4.0f",     # position gain while zeroing
    "MOTOR_ZERO_KD": "1.0f",
    "MOTOR_ZERO_LEASH": "0.15f",       # rad   — max waypoint lead over pos
    "MOTOR_ZERO_DAMP_KD": "3.0f",      # Kd of the damping command on zero timeout
    "MOTOR_ZERO_PROGRESS_EPS": "0.01f",# rad   — |pos| improvement counted as progress
    "MOTOR_WOUND_OFFSET_MAX": "9.0f",  # rad — |pos_offset| above this at HOLD-arm → refuse + CAUSE_WOUND
}

# ── Rates & timeouts (system-wide; override per-setup in configs/<setup>/system.yaml) ──
# Rates in Hz; timeouts/debounces in ms. Tick-based firmware constants (loop
# periods, settle-tick count, enable-monitor frame count) are DERIVED from these,
# so changing a rate keeps the real-world durations fixed.
RATE_DEFAULTS = {
    "master_poll_hz":  200,   # SPI poll / command delivery
    "telemetry_hz":    200,   # MSG_ROBOT_TELE emission
    "slave_tick_hz":   200,   # slave control tick (motor_runtime_update)
    "host_cmd_hz":      50,   # expected host command rate (run_policy --rate default)
    "master_spi_prescaler": 16,  # APB2(72MHz)/div SPI clock: 64→1.125MHz ... 8→9MHz.
    # All of 64/32/16/8 are 2-min clean on the bench (short wiring); 16 = 4.5MHz is two
    # steps of margin below the fastest tested (div 8), and the slave SPI resync recovers
    # isolated glitches — re-validate on the robot harness via the CRC/resync counters.
}
TIMING_MS_DEFAULTS = {
    "motor_watchdog_ms":   200,   # master-link (SPI) watchdog → IDLE
    "can_fb_timeout_ms":   100,   # per-motor Type-2 staleness → CAUSE_CAN_TIMEOUT
    "zero_stall_ms":      1500,   # no TO_ZERO progress → CAUSE_ZERO_TIMEOUT
    "zero_settle_ms":       50,   # in-tolerance dwell before TO_ZERO arrival
    "enable_mon_ms":        15,   # not-NORMAL dwell while armed → CAUSE_NOT_ENABLED
    # Dead-man watchdogs (task 6).
    "master_lost_grace_ms":  50,  # slave: hold position (v/tau=0) this long after exchanges stop
    "master_lost_damp_ms":  300,  # DAMPED dwell before IDLE (slave master-loss AND master host-death)
}


def load_system(setup_dir):
    """Load configs/<setup>/system.yaml if present, else defaults. Returns
    (rates, timeouts_ms) with every key filled from the defaults."""
    rates = dict(RATE_DEFAULTS)
    timeouts = dict(TIMING_MS_DEFAULTS)
    path = os.path.join(setup_dir, "system.yaml")
    if os.path.isfile(path):
        doc = load(path) or {}
        rates.update(doc.get("rates", {}) or {})
        timeouts.update(doc.get("timeouts_ms", {}) or {})
    return rates, timeouts


def timing_block(rates, timeouts):
    """Render the generated timing/rate #define block (shared via motor_config.h).
    Derives tick periods and tick/frame counts from the rates so the real-world
    durations stay fixed when a rate changes."""
    poll_ms = max(1, round(1000.0 / rates["master_poll_hz"]))
    tele_ms = max(1, round(1000.0 / rates["telemetry_hz"]))
    tick_ms = max(1, round(1000.0 / rates["slave_tick_hz"]))
    settle_ticks = max(1, round(timeouts["zero_settle_ms"] / tick_ms))
    mon_k        = max(1, round(timeouts["enable_mon_ms"] / tick_ms))
    # Host-death (master side): 3 host-command periods, floored at 25 ms, in master cycles.
    host_lost_ms     = max(25, round(3.0 * 1000.0 / rates["host_cmd_hz"]))
    host_lost_cyc    = max(1, round(host_lost_ms * rates["master_poll_hz"] / 1000.0))
    host_damp_cyc    = max(1, round(timeouts["master_lost_damp_ms"] * rates["master_poll_hz"] / 1000.0))
    # Slave TX-arm deadline: how long after an exchange the slave waits for the motor replies
    # before arming the DMA for the next exchange (so the fresh reply rides it). 0.6 cycle —
    # after the ≤2 ms reply window, comfortably before the next NSS.
    tx_arm_deadline_us = max(1, round(0.6 * 1000000.0 / rates["master_poll_hz"]))
    return f"""\
/* Configured rates (Hz) — reported in MasterStatus and the .bin header. */
#define MASTER_POLL_HZ  {rates['master_poll_hz']}u
#define TELEMETRY_HZ    {rates['telemetry_hz']}u
#define SLAVE_TICK_HZ   {rates['slave_tick_hz']}u
#define HOST_CMD_HZ     {rates['host_cmd_hz']}u

/* One master poll cycle (µs). The slave derives its forward/fallback service
   thresholds from this, so they scale with the poll rate (incl. 400 Hz). */
#define MASTER_CYCLE_US  {round(1000000.0 / rates['master_poll_hz'])}u

/* Master SPI1 clock divider off APB2 (72 MHz): MX_SPI1_Init maps it to the
   SPI_BAUDRATEPRESCALER_* enum. Valid: 2,4,8,16,32,64,128,256. */
#define MASTER_SPI_PRESCALER_DIV  {rates['master_spi_prescaler']}u

/* Derived loop periods (ms) — do not hand-edit; change the rate instead. */
#define MASTER_POLL_PERIOD_MS  {poll_ms}u
#define MASTER_TELE_PERIOD_MS  {tele_ms}u
#define MOTOR_LOOP_PERIOD_MS   {tick_ms}u
#define MOTOR_LOOP_DT_S        ((float)MOTOR_LOOP_PERIOD_MS * 0.001f)

/* Timeouts/debounces (ms) and tick/frame counts derived from the slave tick. */
#define MOTOR_WATCHDOG_MS        {timeouts['motor_watchdog_ms']}u
#define MOTOR_CAN_FB_TIMEOUT_MS  {timeouts['can_fb_timeout_ms']}u
#define MOTOR_ZERO_STALL_MS      {timeouts['zero_stall_ms']}u
#define MOTOR_ZERO_SETTLE_TICKS  {settle_ticks}u   /* {timeouts['zero_settle_ms']} ms / {tick_ms} ms tick */
#define MOTOR_ENABLE_MON_K       {mon_k}u   /* {timeouts['enable_mon_ms']} ms / {tick_ms} ms tick */

/* Dead-man watchdogs (task 6). Master host-death is counted in master cycles; the slave
   master-loss ramp is in ms (HAL_GetTick). */
#define HOST_CMD_TIMEOUT_MS      {host_lost_ms}u   /* 3 host periods, 25 ms floor */
#define HOST_LOST_CYCLES         {host_lost_cyc}u   /* host-death trigger (master cycles) */
#define HOST_LOST_DAMP_CYCLES    {host_damp_cyc}u   /* master DAMPED dwell before IDLE (cycles) */
#define MASTER_LOST_GRACE_MS     {timeouts['master_lost_grace_ms']}u   /* slave hold (v/tau=0) after exchanges stop */
#define MASTER_LOST_DAMP_MS      {timeouts['master_lost_damp_ms']}u   /* slave DAMPED dwell before IDLE */

/* Slave SPI TX-arm deadline (µs after an exchange): wait for motor replies, then arm the
   next exchange's DMA so the fresh reply rides it (removes the one-exchange telemetry lag). */
#define TX_ARM_DEADLINE_US       {tx_arm_deadline_us}u"""


def _f(x):
    """Format a value as a valid C float literal (always has a decimal point)."""
    s = repr(float(x))
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s + "f"


def load(path):
    with open(path) as fh:
        return yaml.safe_load(fh)


def validate(cfg):
    motors = cfg["motors"]
    models = cfg["models"]
    for i, m in enumerate(motors):
        if m["idx"] != i:
            raise ValueError(f"motor #{i} has idx={m['idx']}; must be sequential 0..N-1")
        if m["model"] not in models:
            raise ValueError(f"motor idx {i} references unknown model {m['model']!r}")
    can_ids = [m["can_id"] for m in motors]
    if len(set(can_ids)) != len(can_ids):
        raise ValueError(f"duplicate can_id in motors: {can_ids}")


# Per-model Kp/Kd CAN ranges. RS00/01/02/05 use Kp 0–500 / Kd 0–5; the larger
# RS03/04/06 use Kp 0–5000 / Kd 0–100 (RobStride Type-1 tables). A model in the yaml
# `models:` block may give kp_min/kp_max/kd_min/kd_max explicitly; otherwise this
# per-model default applies — NOT a single global, which would mis-scale a mixed chain.
_LARGE_GAIN_MODELS = ("RS03", "RS04", "RS06")

def _gain(model_name, model_dict, key):
    if key in model_dict:
        return model_dict[key]
    large = model_name in _LARGE_GAIN_MODELS
    return {"kp_min": 0.0, "kp_max": 5000.0 if large else 500.0,
            "kd_min": 0.0, "kd_max": 100.0 if large else 5.0}[key]


def _transport_bounds(cfgs):
    """Global SPI transport encoding bounds across all given configs: widest
    model velocity/torque so every motor's value fits losslessly, plus the
    shared position range (identical across models)."""
    v_max = max(r["v_max"] for cfg in cfgs for r in cfg["models"].values())
    t_max = max(r["t_max"] for cfg in cfgs for r in cfg["models"].values())
    p_min = min(cfg["shared_ranges"]["p_min"] for cfg in cfgs)
    p_max = max(cfg["shared_ranges"]["p_max"] for cfg in cfgs)
    return p_min, p_max, v_max, t_max


# ─────────────────────────────────────────────────────────────────────────────
#  --slave : per-slave C header (motor_config.h)
# ─────────────────────────────────────────────────────────────────────────────

def gen_header(cfg, banner_src, banner_regen, rates, timeouts):
    motors = cfg["motors"]
    models = cfg["models"]
    shared = cfg["shared_ranges"]
    n = len(motors)

    # Transport (SPI u16) encoding bounds: widest model so RS00 fits losslessly.
    v_max = max(models[k]["v_max"] for k in models)
    t_max = max(models[k]["t_max"] for k in models)
    p_min, p_max = shared["p_min"], shared["p_max"]

    model_names = sorted(models.keys())  # stable enum order
    enum_entries = ",\n".join(f"    MOTOR_MODEL_{name} = {i}u"
                              for i, name in enumerate(model_names))

    range_entries = ",\n".join(
        (lambda md: (
            f"    [MOTOR_MODEL_{name}] = {{ {_f(md['v_min'])}, {_f(md['v_max'])}, "
            f"{_f(md['t_min'])}, {_f(md['t_max'])}, "
            f"{_f(_gain(name, md, 'kp_min'))}, {_f(_gain(name, md, 'kp_max'))}, "
            f"{_f(_gain(name, md, 'kd_min'))}, {_f(_gain(name, md, 'kd_max'))} }}"
        ))(models[name])
        for name in model_names
    )

    cfg_entries = []
    for m in motors:
        cfg_entries.append(
            "    {\n"
            f"        .can_id     = {m['can_id']}u,\n"
            f"        .model      = MOTOR_MODEL_{m['model']},\n"
            f"        .joint_name = \"{m['joint_name']}\",\n"
            f"        .soft_min   = {_f(m['soft_min'])},\n"
            f"        .soft_max   = {_f(m['soft_max'])},\n"
            f"        .max_vel    = {_f(m['max_vel'])},\n"
            f"        .max_tau    = {_f(m['max_tau'])},\n"
            f"        .default_kp = {_f(m['default_kp'])},\n"
            f"        .default_kd = {_f(m['default_kd'])},\n"
            "    }"
        )
    cfg_block = ",\n".join(cfg_entries)

    gz = "\n".join(f"#define {k:<14} {v}" for k, v in GOTO_ZERO.items())
    timing = timing_block(rates, timeouts)

    return f"""\
/* AUTO-GENERATED — DO NOT EDIT.
 * Source: {banner_src}
 * Regenerate: {banner_regen}
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {{
#endif

#define N_MOTORS {n}u

/* SPI transport encoding bounds shared by slave (pack) and master (unpack).
 * Set to the widest model so every motor's value fits losslessly. */
#define MOTOR_P_MIN  {_f(p_min)}
#define MOTOR_P_MAX  {_f(p_max)}
#define MOTOR_V_MIN  {_f(-v_max)}
#define MOTOR_V_MAX  {_f(v_max)}
#define MOTOR_T_MIN  {_f(-t_max)}
#define MOTOR_T_MAX  {_f(t_max)}

/* TO_ZERO motion parameters */
{gz}

/* Timing, rates & derived tick counts (system-wide) */
{timing}

/* Motor models present on the bus */
typedef enum {{
{enum_entries}
}} MotorModel;

/* Per-model CAN "operation control mode" (Type 1) ranges — velocity, torque AND
 * Kp/Kd (RS00/02 use Kp 0–500, Kd 0–5; RS03/04/06 use Kp 0–5000, Kd 0–100).
 * From slaveN.yaml `models:`. Only position is shared (MOTOR_P_MIN/MAX). */
typedef struct {{
    float v_min;
    float v_max;
    float t_min;
    float t_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
}} MotorCanRange;

static const MotorCanRange motor_can_ranges[] = {{
{range_entries}
}};

typedef struct {{
    uint8_t     can_id;
    MotorModel  model;
    const char *joint_name;
    float       soft_min;    /* rad */
    float       soft_max;    /* rad */
    float       max_vel;     /* rad/s */
    float       max_tau;     /* Nm — torque trip: |tau| above this idles motor */
    float       default_kp;
    float       default_kd;
}} MotorConfig;

static const MotorConfig motor_configs[N_MOTORS] = {{
{cfg_block}
}};

/* Look up a motor's per-model CAN range by its bus id. Returns NULL if the id
 * is not part of this slave's chain. */
static inline const MotorCanRange *motor_can_range_by_id(uint8_t can_id)
{{
    for (uint8_t i = 0u; i < N_MOTORS; i++) {{
        if (motor_configs[i].can_id == can_id) {{
            return &motor_can_ranges[motor_configs[i].model];
        }}
    }}
    return (const MotorCanRange *)0;
}}

#ifdef __cplusplus
}}
#endif
#endif /* MOTOR_CONFIG_H */
"""


# ─────────────────────────────────────────────────────────────────────────────
#  --system : master C header (system_config.h)
# ─────────────────────────────────────────────────────────────────────────────

def gen_system_header(cfgs, banner_src, banner_regen, rates, timeouts):
    n_slaves = len(cfgs)
    counts = [len(cfg["motors"]) for cfg in cfgs]
    max_per = max(counts)
    total = sum(counts)
    p_min, p_max, v_max, t_max = _transport_bounds(cfgs)
    timing = timing_block(rates, timeouts)

    counts_c = ", ".join(f"{c}u" for c in counts)

    id_rows = []
    for cfg in cfgs:
        ids = [m["can_id"] for m in cfg["motors"]]
        ids += [0] * (max_per - len(ids))            # pad unused slots with 0
        id_rows.append("    { " + ", ".join(f"{i}u" for i in ids) + " }")
    ids_c = ",\n".join(id_rows)

    return f"""\
/* AUTO-GENERATED — DO NOT EDIT.
 * Sources: {banner_src}
 * Regenerate: {banner_regen}
 */
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {{
#endif

#define NUM_SLAVES            {n_slaves}u
#define MAX_MOTORS_PER_SLAVE  {max_per}u
#define TOTAL_MOTORS          {total}u

/* SPI transport encoding bounds (global, widest model across all slaves). */
#define MOTOR_P_MIN  {_f(p_min)}
#define MOTOR_P_MAX  {_f(p_max)}
#define MOTOR_V_MIN  {_f(-v_max)}
#define MOTOR_V_MAX  {_f(v_max)}
#define MOTOR_T_MIN  {_f(-t_max)}
#define MOTOR_T_MAX  {_f(t_max)}

/* Timing, rates & derived tick counts (system-wide; mirror of motor_config.h). */
{timing}

/* Number of active motors on each slave (chain order). */
static const uint8_t slave_motor_counts[NUM_SLAVES] = {{ {counts_c} }};

/* Per-slave RobStride CAN node ids (chain order). Unused slots are 0. */
static const uint8_t slave_motor_ids[NUM_SLAVES][MAX_MOTORS_PER_SLAVE] = {{
{ids_c}
}};

#ifdef __cplusplus
}}
#endif
#endif /* SYSTEM_CONFIG_H */
"""


# ─────────────────────────────────────────────────────────────────────────────
#  --system : host Python constants (motor_config_gen.py)
# ─────────────────────────────────────────────────────────────────────────────

def gen_python(cfgs, banner_src, banner_regen, cfg_name="", cfg_hash="", rates=None):
    n_slaves = len(cfgs)
    rates = rates or RATE_DEFAULTS

    # Per-slave grouped structure.
    slave_blocks = []
    for s, cfg in enumerate(cfgs):
        slave_name = cfg.get("slave") or f"slave{s}"
        motor_lines = []
        for m in cfg["motors"]:
            motor_lines.append(
                "        dict("
                f"idx={m['idx']}, can_id={m['can_id']}, model={m['model']!r}, "
                f"joint_name={m['joint_name']!r}, "
                f"soft_min={float(m['soft_min'])!r}, soft_max={float(m['soft_max'])!r}, "
                f"max_vel={float(m['max_vel'])!r}, max_tau={float(m['max_tau'])!r}, "
                f"default_kp={float(m['default_kp'])!r}, default_kd={float(m['default_kd'])!r})"
            )
        motors_block = ",\n".join(motor_lines)
        slave_blocks.append(
            f"    dict(slave={s}, name={slave_name!r}, motors=[\n{motors_block},\n    ])"
        )
    slaves_block = ",\n".join(slave_blocks)

    # Union of model ranges across all slaves.
    model_union = {}
    for cfg in cfgs:
        for k, r in cfg["models"].items():
            model_union.setdefault(k, r)
    model_lines = []
    for k in sorted(model_union):
        r = model_union[k]
        model_lines.append(
            f"    {k!r}: dict(v_min={float(r['v_min'])!r}, v_max={float(r['v_max'])!r}, "
            f"t_min={float(r['t_min'])!r}, t_max={float(r['t_max'])!r})"
        )
    model_block = ",\n".join(model_lines)

    # Global SPI transport bounds — must equal the C MOTOR_*_MIN/MAX the slave
    # encodes telemetry with; the host decodes raw pos/vel/tau with these.
    p_min, p_max, v_max, t_max = _transport_bounds(cfgs)

    return f'''\
# AUTO-GENERATED — DO NOT EDIT.
# Sources: {banner_src}
# Regenerate: {banner_regen}
"""Motor configuration constants for host tools (generated, multi-slave)."""

# Active setup name + sha256 over its slave YAMLs (sorted by filename). The runner
# recomputes the hash from configs/<CONFIG_NAME>/ and warns if this file is stale.
CONFIG_NAME = {cfg_name!r}
CONFIG_HASH = {cfg_hash!r}

N_SLAVES = {n_slaves}

# Per-slave structure, in slave order. Each motor dict's "idx" is LOCAL to its
# slave (0-based chain index). The host addresses a motor by (slave, idx).
SLAVES = [
{slaves_block},
]

# Per-model CAN velocity/torque ranges (union across slaves).
MODEL_RANGES = {{
{model_block},
}}

# ── Global SPI transport encoding bounds (widest model across slaves) ──────────
# The slave encodes pos/vel/tau_raw over THESE bounds (not per-model); decode raw
# telemetry with them. pos_raw is home-frame wrapped [-pi, pi] over the +-4pi bound.
MOTOR_P_MIN = {float(p_min)!r}
MOTOR_P_MAX = {float(p_max)!r}
MOTOR_V_MIN = {float(-v_max)!r}
MOTOR_V_MAX = {float(v_max)!r}
MOTOR_T_MIN = {float(-t_max)!r}
MOTOR_T_MAX = {float(t_max)!r}

# ── Flattened view (global index = position in this list) ──────────────────────
# Each motor dict gains "slave" (slave index) and keeps its local "idx".
MOTORS = [
    dict(m, slave=s["slave"]) for s in SLAVES for m in s["motors"]
]

N_MOTORS = len(MOTORS)

MOTOR_DEFAULT_KP = [m["default_kp"] for m in MOTORS]
MOTOR_DEFAULT_KD = [m["default_kd"] for m in MOTORS]

# Per-motor soft angle limits (rad). Commands are clamped to [soft_min, soft_max].
MOTOR_SOFT_MIN = [m["soft_min"] for m in MOTORS]
MOTOR_SOFT_MAX = [m["soft_max"] for m in MOTORS]

# Motor counts per slave, in slave order.
SLAVE_MOTOR_COUNTS = [len(s["motors"]) for s in SLAVES]

# ── Configured rates (Hz) — mirror the firmware motor_config.h; recorded in the
# .bin header and used as the run_policy --rate default (HOST_CMD_HZ). ──────────
MASTER_POLL_HZ = {rates['master_poll_hz']}
TELEMETRY_HZ   = {rates['telemetry_hz']}
SLAVE_TICK_HZ  = {rates['slave_tick_hz']}
HOST_CMD_HZ    = {rates['host_cmd_hz']}

RATES = dict(master_poll_hz=MASTER_POLL_HZ, telemetry_hz=TELEMETRY_HZ,
             slave_tick_hz=SLAVE_TICK_HZ, host_cmd_hz=HOST_CMD_HZ)
'''


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

def _resolve(path):
    if not os.path.isabs(path):
        path = os.path.join(REPO_ROOT, path)
    return path


CONFIGS_DIR = os.path.join(REPO_ROOT, "configs")


def _configs_rel(src):
    """Path relative to configs/, e.g. 'bench/slave0.yaml' — used in banners."""
    return os.path.relpath(src, CONFIGS_DIR)


def active_setup():
    """Name of the active physical setup (a subdir under configs/).

    Precedence: $SOCCER_SETUP overrides the configs/active pointer file.
    Returns None if neither is set."""
    env = os.environ.get("SOCCER_SETUP")
    if env and env.strip():
        return env.strip()
    ptr = os.path.join(CONFIGS_DIR, "active")
    if os.path.isfile(ptr):
        with open(ptr) as fh:
            name = fh.read().strip()
        return name or None
    return None


def _require_setup():
    s = active_setup()
    if not s:
        sys.exit("no active setup selected: write one to configs/active "
                 "(e.g. `echo bench > configs/active`) or set SOCCER_SETUP")
    setup_dir = os.path.join(CONFIGS_DIR, s)
    if not os.path.isdir(setup_dir):
        sys.exit(f"active setup {s!r} has no directory (expected configs/{s}/)")
    return s


def resolve_config(token):
    """Resolve a slave-config token to an absolute path.

    An existing file (absolute or repo-relative) is used as-is; otherwise a bare
    name like 'slave0' resolves to configs/<active setup>/slave0.yaml."""
    cand = token if os.path.isabs(token) else os.path.join(REPO_ROOT, token)
    if os.path.isfile(cand):
        return cand
    name = token if token.endswith(".yaml") else token + ".yaml"
    setup = _require_setup()
    p = os.path.join(CONFIGS_DIR, setup, name)
    if os.path.isfile(p):
        return p
    sys.exit(f"config {token!r} not found (looked for {cand} and {p})")


def active_slaves():
    """Every slave*.yaml in the active setup dir, sorted (slave0, slave1, ...)."""
    setup = _require_setup()
    files = sorted(glob.glob(os.path.join(CONFIGS_DIR, setup, "slave*.yaml")))
    if not files:
        sys.exit(f"no slave*.yaml found in configs/{setup}/")
    return files


def config_hash(paths):
    """LEGACY: sha256 over the given slave*.yaml files' contents, sorted by filename.
    (The firmware_config.yaml path uses the artifact's own content_sha256 instead.)"""
    h = hashlib.sha256()
    for p in sorted(paths, key=os.path.basename):
        with open(p, "rb") as fh:
            h.update(fh.read())
    return h.hexdigest()


# ── firmware_config.yaml — the single-file artifact exported per robot (preferred) ──
def firmware_config_path(setup):
    return os.path.join(CONFIGS_DIR, setup, "firmware_config.yaml")


def load_firmware_config(setup):
    """Parsed configs/<setup>/firmware_config.yaml, or None if that setup still uses
    legacy per-slave slaveN.yaml files."""
    p = firmware_config_path(setup)
    if not os.path.isfile(p):
        return None
    doc = load(p)
    if not isinstance(doc, dict) or "slaves" not in doc:
        sys.exit(f"{os.path.relpath(p, REPO_ROOT)}: not a firmware_config.yaml (no 'slaves' list)")
    return doc


def _fw_rates_timeouts(doc):
    """Rates come from the artifact's `rates` block (the 4 Hz); the SPI prescaler and the
    ms timeouts/debounces are firmware-side defaults the artifact does not carry."""
    rates = dict(RATE_DEFAULTS)
    rates.update(doc.get("rates") or {})
    return rates, dict(TIMING_MS_DEFAULTS)


def _fw_cfg(doc, section):
    """A per-slave cfg dict (shared models/ranges + that slave's motors) for the emitters."""
    return {"slave": section.get("slave"),
            "shared_ranges": doc["shared_ranges"],
            "models": doc["models"],
            "motors": section["motors"]}


def _fw_section(doc, slave_name):
    for s in doc["slaves"]:
        if s.get("slave") == slave_name:
            return s
    have = [s.get("slave") for s in doc["slaves"]]
    sys.exit(f"slave {slave_name!r} not in firmware_config.yaml (have {have})")


# ── emitters ──
def _write_slave_header(cfg, banner_src, banner_regen, rates, timeouts):
    validate(cfg)
    header_path = os.path.join(REPO_ROOT, "firmware/common/include/motor_config.h")
    with open(header_path, "w") as fh:
        fh.write(gen_header(cfg, banner_src, banner_regen, rates, timeouts))
    print(f"Generated {os.path.relpath(header_path, REPO_ROOT)}  (slave: {cfg.get('slave', '?')}, "
          f"N_MOTORS={len(cfg['motors'])}, tick={rates['slave_tick_hz']}Hz)")


def _write_system(cfgs, banner_src, banner_regen, cfg_name, cfg_hash, rates, timeouts):
    for c in cfgs:
        validate(c)
    sys_path = os.path.join(REPO_ROOT, "firmware/common/include/system_config.h")
    py_path = os.path.join(REPO_ROOT, "host/master_link/motor_config_gen.py")
    with open(sys_path, "w") as fh:
        fh.write(gen_system_header(cfgs, banner_src, banner_regen, rates, timeouts))
    with open(py_path, "w") as fh:
        fh.write(gen_python(cfgs, banner_src, banner_regen, cfg_name, cfg_hash, rates))
    counts = [len(c["motors"]) for c in cfgs]
    print(f"Generated {os.path.relpath(sys_path, REPO_ROOT)}  (NUM_SLAVES={len(cfgs)}, counts={counts})")
    print(f"Generated {os.path.relpath(py_path, REPO_ROOT)}")


# ── slave header ──
def do_slave_name(setup, slave_name):
    """Emit motor_config.h for one slave of `setup` — firmware_config.yaml section if present,
    else the legacy configs/<setup>/<slave_name>.yaml."""
    doc = load_firmware_config(setup)
    if doc is not None:
        cfg = _fw_cfg(doc, _fw_section(doc, slave_name))
        rates, timeouts = _fw_rates_timeouts(doc)
        _write_slave_header(
            cfg,
            f"configs/{setup}/firmware_config.yaml (section {slave_name})",
            f"SOCCER_SETUP={setup} python3 scripts/gen_motor_config.py --slave {slave_name}",
            rates, timeouts)
        return
    do_slave_path(resolve_config(slave_name))


def do_slave_path(path):
    """LEGACY: emit motor_config.h for an explicit slaveN.yaml path."""
    path = _resolve(path)
    rel = _configs_rel(path)
    cfg = load(path)
    rates, timeouts = load_system(os.path.dirname(path))
    _write_slave_header(cfg, f"configs/{rel}",
                        f"python3 scripts/gen_motor_config.py --slave configs/{rel}",
                        rates, timeouts)


# ── system headers ──
def do_system_setup(setup):
    """Emit system_config.h + motor_config_gen.py for `setup` — firmware_config.yaml if present,
    else all legacy slaveN.yaml."""
    doc = load_firmware_config(setup)
    if doc is not None:
        cfgs = [_fw_cfg(doc, s) for s in doc["slaves"]]
        rates, timeouts = _fw_rates_timeouts(doc)
        _write_system(
            cfgs,
            f"configs/{setup}/firmware_config.yaml",
            f"SOCCER_SETUP={setup} python3 scripts/gen_motor_config.py --system",
            setup, doc.get("content_sha256") or "", rates, timeouts)
        return
    do_system_paths(active_slaves())


def do_system_paths(paths):
    """LEGACY: emit system headers from explicit slaveN.yaml paths."""
    paths = [_resolve(p) for p in paths]
    names = [_configs_rel(p) for p in paths]
    cfgs = [load(p) for p in paths]
    cfg_name = os.path.dirname(names[0]) or os.path.splitext(os.path.basename(names[0]))[0]
    cfg_hash = config_hash(paths)
    rates, timeouts = load_system(os.path.dirname(paths[0]))
    src_list = " ".join(f"configs/{n}" for n in names)
    _write_system(cfgs, ", ".join(names),
                  f"python3 scripts/gen_motor_config.py --system {src_list}",
                  cfg_name, cfg_hash, rates, timeouts)


def _is_path_token(tok):
    cand = tok if os.path.isabs(tok) else os.path.join(REPO_ROOT, tok)
    return os.path.isfile(cand) or tok.endswith(".yaml") or os.sep in tok


def main():
    args = sys.argv[1:]
    if not args:
        # Regenerate everything for the active setup: system files + the default
        # per-slave header (first slave).
        setup = _require_setup()
        doc = load_firmware_config(setup)
        if doc is not None:
            print(f"Active setup: {setup}  ({len(doc['slaves'])} slave(s), firmware_config.yaml)")
            do_system_setup(setup)
            do_slave_name(setup, doc["slaves"][0]["slave"])
        else:
            slaves = active_slaves()
            print(f"Active setup: {setup}  ({len(slaves)} slave(s), legacy slaveN.yaml)")
            do_system_paths(slaves)
            do_slave_path(slaves[0])
        return
    if args[0] == "--slave":
        if len(args) != 2:
            sys.exit("usage: gen_motor_config.py --slave <slaveN | path.yaml>")
        tok = args[1]
        if _is_path_token(tok):
            do_slave_path(tok)           # explicit legacy slaveN.yaml path
        else:
            do_slave_name(_require_setup(), tok)   # bare name → firmware_config.yaml section
    elif args[0] == "--system":
        srcs = args[1:]
        if srcs:
            do_system_paths(srcs)        # explicit legacy paths
        else:
            do_system_setup(_require_setup())
    elif args[0].startswith("--"):
        sys.exit(f"unknown option {args[0]!r}; use --slave or --system")
    else:
        tok = args[0]
        if _is_path_token(tok):
            do_slave_path(tok)
        else:
            do_slave_name(_require_setup(), tok)


if __name__ == "__main__":
    main()
