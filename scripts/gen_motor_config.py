#!/usr/bin/env python3
"""Generate motor configuration files from a slave YAML description.

Reads a config like configs/slave0.yaml (the single source of truth) and emits:
  - firmware/common/include/motor_config.h   (C header for slave + master)
  - tools/motor_config_gen.py                (Python constants for host tools)

Usage:
    python3 scripts/gen_motor_config.py [configs/slave0.yaml]

Re-run after editing the YAML, then rebuild the firmware. The generated files
carry an AUTO-GENERATED banner and should not be hand-edited.
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
 * Regenerate: python3 scripts/gen_motor_config.py configs/{src_name}
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
    float       max_tau;     /* Nm */
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


def gen_python(cfg, src_name):
    motors = cfg["motors"]
    models = cfg["models"]
    n = len(motors)

    motor_lines = []
    for m in motors:
        motor_lines.append(
            "    dict("
            f"idx={m['idx']}, can_id={m['can_id']}, model={m['model']!r}, "
            f"joint_name={m['joint_name']!r}, "
            f"default_kp={float(m['default_kp'])!r}, default_kd={float(m['default_kd'])!r})"
        )
    motor_block = ",\n".join(motor_lines)

    model_lines = []
    for name in sorted(models.keys()):
        r = models[name]
        model_lines.append(
            f"    {name!r}: dict(v_min={float(r['v_min'])!r}, v_max={float(r['v_max'])!r}, "
            f"t_min={float(r['t_min'])!r}, t_max={float(r['t_max'])!r})"
        )
    model_block = ",\n".join(model_lines)

    return f'''\
# AUTO-GENERATED — DO NOT EDIT.
# Source: configs/{src_name}
# Regenerate: python3 scripts/gen_motor_config.py configs/{src_name}
"""Motor configuration constants for host tools (generated)."""

N_MOTORS = {n}

# One dict per motor, in chain order (index == motor_idx).
MOTORS = [
{motor_block},
]

# Per-model CAN velocity/torque ranges.
MODEL_RANGES = {{
{model_block},
}}

MOTOR_DEFAULT_KP = [m["default_kp"] for m in MOTORS]
MOTOR_DEFAULT_KD = [m["default_kd"] for m in MOTORS]
'''


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO_ROOT, "configs/slave0.yaml")
    if not os.path.isabs(src):
        src = os.path.join(REPO_ROOT, src)
    src_name = os.path.basename(src)

    cfg = load(src)
    validate(cfg)

    header_path = os.path.join(REPO_ROOT, "firmware/common/include/motor_config.h")
    py_path = os.path.join(REPO_ROOT, "tools/motor_config_gen.py")

    with open(header_path, "w") as fh:
        fh.write(gen_header(cfg, src_name))
    with open(py_path, "w") as fh:
        fh.write(gen_python(cfg, src_name))

    print(f"Generated {os.path.relpath(header_path, REPO_ROOT)}")
    print(f"Generated {os.path.relpath(py_path, REPO_ROOT)}")
    print(f"  N_MOTORS = {len(cfg['motors'])}")


if __name__ == "__main__":
    main()
