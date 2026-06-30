#!/usr/bin/env python3
"""Generate motor configuration files from slave YAML descriptions.

Two modes:

  --slave  <one.yaml>         (default for a bare positional argument)
      Emits firmware/common/include/motor_config.h for THAT ONE slave
      (N_MOTORS, transport bounds, MotorModel enum, motor_can_ranges[],
      motor_configs[], motor_can_range_by_id()). The slave firmware build
      compiles its own slave's config — see scripts/build.sh --config.

  --system <a.yaml> <b.yaml> ...
      Emits, covering ALL slaves in the system:
        - firmware/common/include/system_config.h  (master: NUM_SLAVES,
          per-slave motor counts + CAN-id LUTs, global transport bounds)
        - tools/motor_config_gen.py                (host: per-slave SLAVES[]
          plus a flattened view for the dashboard / test_client)

Usage:
    python3 scripts/gen_motor_config.py --slave  configs/slave0.yaml
    python3 scripts/gen_motor_config.py --system configs/slave0.yaml configs/slave1.yaml
    python3 scripts/gen_motor_config.py configs/slave0.yaml        # == --slave

The firmware build does not run Python, so generated files are committed
alongside the source. Re-run after editing a YAML, then rebuild.
"""

import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Goto-zero motion parameters (motion tuning, not per-motor identity).
GOTO_ZERO = {
    "MOTOR_ZERO_TOL": "0.05f",   # rad (~3 deg) — arrival threshold
    "MOTOR_ZERO_RATE": "0.3f",   # rad/s        — constant approach speed
    "MOTOR_ZERO_KP": "4.0f",     # position gain while zeroing
    "MOTOR_ZERO_KD": "1.0f",
}


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

def gen_header(cfg, src_name):
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

/* Goto-zero motion parameters */
{gz}

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

def gen_system_header(cfgs, names):
    n_slaves = len(cfgs)
    counts = [len(cfg["motors"]) for cfg in cfgs]
    max_per = max(counts)
    total = sum(counts)
    p_min, p_max, v_max, t_max = _transport_bounds(cfgs)

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

def gen_python(cfgs, names):
    n_slaves = len(cfgs)

    # Per-slave grouped structure.
    slave_blocks = []
    for s, (cfg, name) in enumerate(zip(cfgs, names)):
        slave_name = cfg.get("slave", os.path.splitext(name)[0])
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

    src_list = " ".join(f"configs/{n}" for n in names)

    return f'''\
# AUTO-GENERATED — DO NOT EDIT.
# Sources: {", ".join(names)}
# Regenerate: python3 scripts/gen_motor_config.py --system {src_list}
"""Motor configuration constants for host tools (generated, multi-slave)."""

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
'''


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

def _resolve(path):
    if not os.path.isabs(path):
        path = os.path.join(REPO_ROOT, path)
    return path


def do_slave(src):
    src = _resolve(src)
    src_name = os.path.basename(src)
    cfg = load(src)
    validate(cfg)
    header_path = os.path.join(REPO_ROOT, "firmware/common/include/motor_config.h")
    with open(header_path, "w") as fh:
        fh.write(gen_header(cfg, src_name))
    print(f"Generated {os.path.relpath(header_path, REPO_ROOT)}  (slave: {src_name}, N_MOTORS={len(cfg['motors'])})")


def do_system(srcs):
    srcs = [_resolve(s) for s in srcs]
    names = [os.path.basename(s) for s in srcs]
    cfgs = []
    for s in srcs:
        cfg = load(s)
        validate(cfg)
        cfgs.append(cfg)

    sys_path = os.path.join(REPO_ROOT, "firmware/common/include/system_config.h")
    py_path = os.path.join(REPO_ROOT, "tools/motor_config_gen.py")
    with open(sys_path, "w") as fh:
        fh.write(gen_system_header(cfgs, names))
    with open(py_path, "w") as fh:
        fh.write(gen_python(cfgs, names))
    counts = [len(c["motors"]) for c in cfgs]
    print(f"Generated {os.path.relpath(sys_path, REPO_ROOT)}  (NUM_SLAVES={len(cfgs)}, counts={counts})")
    print(f"Generated {os.path.relpath(py_path, REPO_ROOT)}")


def main():
    args = sys.argv[1:]
    if not args:
        do_slave("configs/slave0.yaml")
        return
    if args[0] == "--slave":
        if len(args) != 2:
            sys.exit("usage: gen_motor_config.py --slave <one.yaml>")
        do_slave(args[1])
    elif args[0] == "--system":
        if len(args) < 2:
            sys.exit("usage: gen_motor_config.py --system <a.yaml> [b.yaml ...]")
        do_system(args[1:])
    elif args[0].startswith("--"):
        sys.exit(f"unknown option {args[0]!r}; use --slave or --system")
    else:
        # Backward-compatible positional: single slave header.
        do_slave(args[0])


if __name__ == "__main__":
    main()
