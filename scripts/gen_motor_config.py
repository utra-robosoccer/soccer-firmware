#!/usr/bin/env python3
"""Generate motor configuration files from slave YAML descriptions.

Active setup
------------
Which physical setup is in use (e.g. bench vs robot) is selected in ONE place:
the file ``configs/active`` (a single line naming a subdirectory of configs/),
overridable per-invocation by the ``SOCCER_SETUP`` env var. Slave configs live
in ``configs/<setup>/slaveN.yaml``. A bare slave name like ``slave0`` resolves
against the active setup, so the whole toolchain follows one switch.

Two modes:

  --slave  <slaveN | path.yaml>   (default for a bare positional argument)
      Emits firmware/common/include/motor_config.h for THAT ONE slave
      (N_MOTORS, transport bounds, MotorModel enum, motor_can_ranges[],
      motor_configs[], motor_can_range_by_id()). The slave firmware build
      compiles its own slave's config — see scripts/build.sh --config.

  --system [<a.yaml> <b.yaml> ...]
      With no args, uses every slave*.yaml in the active setup. Emits, covering
      ALL slaves in the system:
        - firmware/common/include/system_config.h  (master: NUM_SLAVES,
          per-slave motor counts + CAN-id LUTs, global transport bounds)
        - host/master_link/motor_config_gen.py      (host: per-slave SLAVES[]
          plus a flattened view for the dashboard / test_client)

Usage:
    python3 scripts/gen_motor_config.py                    # regen everything for active setup
    python3 scripts/gen_motor_config.py --slave  slave0    # active setup's slave0
    python3 scripts/gen_motor_config.py --system           # active setup, all slaves
    python3 scripts/gen_motor_config.py --slave  configs/robot/slave0.yaml   # explicit path
    SOCCER_SETUP=robot python3 scripts/gen_motor_config.py # one-off setup override

The firmware build does not run Python, so generated files are committed
alongside the source. Re-run after editing a YAML, then rebuild.
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
}
TIMING_MS_DEFAULTS = {
    "motor_watchdog_ms":   200,   # master-link (SPI) watchdog → IDLE
    "can_fb_timeout_ms":   100,   # per-motor Type-2 staleness → CAUSE_CAN_TIMEOUT
    "zero_stall_ms":      1500,   # no TO_ZERO progress → CAUSE_ZERO_TIMEOUT
    "zero_settle_ms":       50,   # in-tolerance dwell before TO_ZERO arrival
    "enable_mon_ms":        15,   # not-NORMAL dwell while armed → CAUSE_NOT_ENABLED
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
    return f"""\
/* Configured rates (Hz) — reported in MasterStatus and the .bin header. */
#define MASTER_POLL_HZ  {rates['master_poll_hz']}u
#define TELEMETRY_HZ    {rates['telemetry_hz']}u
#define SLAVE_TICK_HZ   {rates['slave_tick_hz']}u
#define HOST_CMD_HZ     {rates['host_cmd_hz']}u

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
#define MOTOR_ENABLE_MON_K       {mon_k}u   /* {timeouts['enable_mon_ms']} ms / {tick_ms} ms tick */"""


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

def gen_header(cfg, src_name, rates, timeouts):
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
        f"    [MOTOR_MODEL_{name}] = {{ {_f(models[name]['v_min'])}, {_f(models[name]['v_max'])}, "
        f"{_f(models[name]['t_min'])}, {_f(models[name]['t_max'])} }}"
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
 * Source: configs/{src_name}
 * Regenerate: python3 scripts/gen_motor_config.py --slave configs/{src_name}
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

/* Per-model CAN "operation control mode" (Type 1) velocity/torque ranges.
 * Position, Kp and Kd are identical across models and stay global. */
typedef struct {{
    float v_min;
    float v_max;
    float t_min;
    float t_max;
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

def gen_system_header(cfgs, names, rates, timeouts):
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

    src_list = " ".join(f"configs/{n}" for n in names)

    return f"""\
/* AUTO-GENERATED — DO NOT EDIT.
 * Sources: {", ".join(names)}
 * Regenerate: python3 scripts/gen_motor_config.py --system {src_list}
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

def gen_python(cfgs, names, cfg_name="", cfg_hash="", rates=None):
    n_slaves = len(cfgs)
    rates = rates or RATE_DEFAULTS

    # Per-slave grouped structure.
    slave_blocks = []
    for s, (cfg, name) in enumerate(zip(cfgs, names)):
        slave_name = cfg.get("slave", os.path.splitext(os.path.basename(name))[0])
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

    src_list = " ".join(f"configs/{n}" for n in names)

    return f'''\
# AUTO-GENERATED — DO NOT EDIT.
# Sources: {", ".join(names)}
# Regenerate: python3 scripts/gen_motor_config.py --system {src_list}
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


def do_slave(src):
    src = _resolve(src)
    src_name = _configs_rel(src)
    cfg = load(src)
    validate(cfg)
    rates, timeouts = load_system(os.path.dirname(src))
    header_path = os.path.join(REPO_ROOT, "firmware/common/include/motor_config.h")
    with open(header_path, "w") as fh:
        fh.write(gen_header(cfg, src_name, rates, timeouts))
    print(f"Generated {os.path.relpath(header_path, REPO_ROOT)}  (slave: {src_name}, "
          f"N_MOTORS={len(cfg['motors'])}, tick={rates['slave_tick_hz']}Hz)")


def config_hash(paths):
    """sha256 over the given YAML files' contents, sorted by filename. The runtime
    staleness check (host/master_link/config_meta.py) recomputes this identically."""
    h = hashlib.sha256()
    for p in sorted(paths, key=os.path.basename):
        with open(p, "rb") as fh:
            h.update(fh.read())
    return h.hexdigest()


def do_system(srcs):
    srcs = [_resolve(s) for s in srcs]
    names = [_configs_rel(s) for s in srcs]
    cfgs = []
    for s in srcs:
        cfg = load(s)
        validate(cfg)
        cfgs.append(cfg)

    cfg_name = os.path.dirname(names[0]) or os.path.splitext(os.path.basename(names[0]))[0]
    cfg_hash = config_hash(srcs)
    rates, _timeouts = load_system(os.path.dirname(srcs[0]))

    sys_path = os.path.join(REPO_ROOT, "firmware/common/include/system_config.h")
    py_path = os.path.join(REPO_ROOT, "host/master_link/motor_config_gen.py")
    with open(sys_path, "w") as fh:
        fh.write(gen_system_header(cfgs, names, rates, _timeouts))
    with open(py_path, "w") as fh:
        fh.write(gen_python(cfgs, names, cfg_name, cfg_hash, rates))
    counts = [len(c["motors"]) for c in cfgs]
    print(f"Generated {os.path.relpath(sys_path, REPO_ROOT)}  (NUM_SLAVES={len(cfgs)}, counts={counts})")
    print(f"Generated {os.path.relpath(py_path, REPO_ROOT)}")


def main():
    args = sys.argv[1:]
    if not args:
        # Regenerate everything for the active setup: system files from all its
        # slaves, plus the default per-slave header (slave0).
        setup = _require_setup()
        slaves = active_slaves()
        print(f"Active setup: {setup}  ({len(slaves)} slave(s))")
        do_system(slaves)
        do_slave(slaves[0])
        return
    if args[0] == "--slave":
        if len(args) != 2:
            sys.exit("usage: gen_motor_config.py --slave <slaveN | path.yaml>")
        do_slave(resolve_config(args[1]))
    elif args[0] == "--system":
        srcs = args[1:]
        do_system([resolve_config(s) for s in srcs] if srcs else active_slaves())
    elif args[0].startswith("--"):
        sys.exit(f"unknown option {args[0]!r}; use --slave or --system")
    else:
        # Backward-compatible positional: single slave header (bare name or path).
        do_slave(resolve_config(args[0]))


if __name__ == "__main__":
    main()
