#!/usr/bin/env python3
"""Generate motor configuration files from a robot's firmware config.

Config source
-------------
Each robot is described by ONE file: ``configs/<setup>/firmware_config.yaml``
(schema v2, currently hand-maintained in this repo; the soccer-software export
will adopt this schema before imports resume). It carries ONLY what varies per
robot — ``slaves: [{slave, motors: [...]}, ...]`` where each motor is three
blocks: ``identity`` (can_id/model/joint_name), ``user`` (tier-2 limits) and
``fallback`` (tier-3 dead-man damp constants). A motor's slave membership and
index are given by position: the slave block that contains it, in list order.

NOT in the yaml (fixed, firmware-side):
  - RS vendor ranges (tier 1)  → motor_rs_config.h + the RS_CONFIG mirror here.
  - rates / timeouts           → RATES / TIMING_MS below. Coupled to firmware
    behavior; changing one IS a firmware change (edit here, regen, rebuild).

Which setup is live is selected in ONE place: ``configs/active`` (a single line
naming a subdirectory of configs/), overridable per-invocation by ``SOCCER_SETUP``.

Two modes:

  --slave  <slaveN | path.yaml>   (default for a bare positional argument)
      Emits firmware/common/include/motor_user_config.h for THAT ONE slave
      (N_MOTORS, motor_user_configs[] = {{id, user, fallback}},
      motor_rs_config_by_id(); vendor ranges come from the hand-written
      motor_rs_config.h, tier 1). The name selects the section of the active
      setup's firmware_config.yaml. See scripts/build.sh --config.

  --system
      Reads the active setup's firmware_config.yaml (all slaves). Emits:
        - firmware/common/include/master_config.h  (master: NUM_SLAVES,
          per-slave motor counts)
        - host/master_link/motor_config_gen.py      (host: per-slave SLAVES[]
          plus a flattened view for the dashboard / test_client)

Usage:
    python3 scripts/gen_motor_config.py                    # regen everything for active setup
    python3 scripts/gen_motor_config.py --slave  slave0    # active setup's slave0 section
    python3 scripts/gen_motor_config.py --system           # active setup, all slaves
    SOCCER_SETUP=2s_10m python3 scripts/gen_motor_config.py --system   # one-off setup override

The firmware build does not run Python: regenerate, then rebuild. The generated
files are gitignored — the yaml (and this script's constants) are the source.
"""

import hashlib
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ── Rates & timing: HAND-WRITTEN in firmware/common/include/timing_config.h ──
# The generator does not own them; it only copies the rates the host needs
# (run_policy rate-verify, .bin log header) out of that header at regen time,
# so there is a single C-side source of truth.
TIMING_CONFIG_H = os.path.join(
    REPO_ROOT, "firmware/common/include/timing_config.h")
_HOST_RATE_DEFINES = ("MASTER_POLL_HZ", "TELEMETRY_HZ", "SLAVE_TICK_HZ",
                      "HOST_CMD_HZ")


def read_rates():
    """The host-facing rate #defines parsed out of timing_config.h."""
    import re
    text = open(TIMING_CONFIG_H).read()
    rates = {}
    for name in _HOST_RATE_DEFINES:
        m = re.search(rf"^#define\s+{name}\s+(\d+)u?\b", text, re.MULTILINE)
        if not m:
            sys.exit(f"{os.path.relpath(TIMING_CONFIG_H, REPO_ROOT)}: "
                     f"could not parse #define {name} (needed by the host)")
        rates[name] = int(m.group(1))
    return rates


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
    """cfg['motors'] holds _norm_motor()-flattened entries. Tier 2 (user) and
    tier 3 (fallback) must fit inside tier 1 (RS_CONFIG)."""
    motors = cfg["motors"]
    for i, m in enumerate(motors):
        if m["model"] not in RS_CONFIG:
            raise ValueError(f"motor #{i} references unknown model {m['model']!r} "
                             f"(known: {sorted(RS_CONFIG)})")
        rs = RS_CONFIG[m["model"]]
        name = f"motor #{i} ({m['joint_name']}, {m['model']})"
        if not (RS_P_MIN <= float(m["u_p_min"]) < float(m["u_p_max"]) <= RS_P_MAX):
            raise ValueError(f"{name}: user.p [{m['u_p_min']}, {m['u_p_max']}] "
                             f"invalid or outside RS position range ±{RS_P_MAX}")
        for key, lim in (("u_v_max", rs["v_max"]), ("u_t_max", rs["t_max"]),
                         ("u_default_kp", rs["kp_max"]), ("u_default_kd", rs["kd_max"]),
                         ("fb_kp", rs["kp_max"]), ("fb_kd", rs["kd_max"])):
            if not (0.0 <= float(m[key]) <= lim):
                raise ValueError(f"{name}: {key}={m[key]} outside RS limit [0, {lim}]")
        if abs(float(m["fb_tau"])) > rs["t_max"]:
            raise ValueError(f"{name}: fb_tau={m['fb_tau']} outside RS limit "
                             f"±{rs['t_max']}")
    can_ids = [m["can_id"] for m in motors]
    if len(set(can_ids)) != len(can_ids):
        raise ValueError(f"duplicate can_id in motors: {can_ids}")


# ── Tier 1: RS (vendor) limits — per-model Type-1 encode ranges ──────────────
# MIRROR of firmware/common/include/motor_rs_config.h (hand-written, fixed —
# sourced from each model's user manual reference code). Used here for transport
# bounds and to VALIDATE robot configs; the firmware reads the C header. The two
# tables must stay identical — change both or neither.
RS_CONFIG = {
    "RS00": dict(v_max=33.0, t_max=14.0, kp_max=500.0,  kd_max=5.0),
    "RS02": dict(v_max=44.0, t_max=17.0, kp_max=500.0,  kd_max=5.0),
    "RS03": dict(v_max=20.0, t_max=60.0, kp_max=5000.0, kd_max=100.0),
    "RS05": dict(v_max=50.0, t_max=5.5,  kp_max=500.0,  kd_max=5.0),
    "RS06": dict(v_max=50.0, t_max=36.0, kp_max=5000.0, kd_max=100.0),
}
RS_P_MIN, RS_P_MAX = -12.57, 12.57

# Enable-gate position margin (rad): an ENABLE is accepted only with the shaft
# inside [p_min - margin, p_max + margin]. Per-slave override via an optional
# top-level `ready_margin_rad` next to `motors` in firmware_config.yaml.
READY_MARGIN_RAD_DEFAULT = 0.2

# ── Schema v2 per-motor shape ─────────────────────────────────────────────────
# identity: {can_id, model, joint_name}        — three names for one physical motor
# user:     {p_min, p_max, v_max, t_max,       — tier 2: our operating envelope,
#            default_kp, default_kd}             inside tier 1 (names mirror
#                                                MotorRsConfig; tier is the scope)
# fallback: {kp, kd, tau}                      — tier 3: dead-man damp MIT
#                                                constants (pos=current, vel=0
#                                                implied; kp normally 0). REQUIRED
#                                                and explicit — these run with no
#                                                host, so no silent defaults.
_IDENTITY_KEYS = ("can_id", "model", "joint_name")
_USER_KEYS = ("p_min", "p_max", "v_max", "t_max", "default_kp", "default_kd")
_FALLBACK_KEYS = ("kp", "kd", "tau")

_LEGACY_MARKERS = ("soft_min", "soft_max", "max_vel", "max_tau", "idx")


def _norm_motor(m, where):
    """Validate one schema-v2 motor entry and flatten it for the emitters."""
    if any(k in m for k in _LEGACY_MARKERS) or "identity" not in m:
        raise ValueError(
            f"{where}: legacy/flat motor entry — schema v2 requires the three "
            f"blocks identity{_IDENTITY_KEYS} / user{_USER_KEYS} / "
            f"fallback{_FALLBACK_KEYS}")
    out = {}
    for block, keys in (("identity", _IDENTITY_KEYS), ("user", _USER_KEYS),
                        ("fallback", _FALLBACK_KEYS)):
        got = m.get(block)
        if not isinstance(got, dict):
            raise ValueError(f"{where}: missing block {block!r} (schema v2)")
        missing = [k for k in keys if k not in got]
        extra = [k for k in got if k not in keys]
        if missing or extra:
            raise ValueError(f"{where}: block {block!r} missing {missing}, "
                             f"unexpected {extra}")
        prefix = {"identity": "", "user": "u_", "fallback": "fb_"}[block]
        for k in keys:
            out[prefix + k] = got[k]
    return out


# ─────────────────────────────────────────────────────────────────────────────
#  --slave : per-slave C header (motor_user_config.h)
# ─────────────────────────────────────────────────────────────────────────────

def gen_header(cfg, banner_src, banner_regen):
    motors = cfg["motors"]
    n = len(motors)
    margin = _f(cfg.get("ready_margin_rad", READY_MARGIN_RAD_DEFAULT))

    cfg_entries = []
    for m in motors:
        cfg_entries.append(
            "    {\n"
            f"        .id       = {{ {m['can_id']}u, MOTOR_MODEL_{m['model']}, "
            f"\"{m['joint_name']}\" }},\n"
            f"        .user     = {{ {_f(m['u_p_min'])}, {_f(m['u_p_max'])}, "
            f"{_f(m['u_v_max'])}, {_f(m['u_t_max'])}, "
            f"{_f(m['u_default_kp'])}, {_f(m['u_default_kd'])} }},\n"
            f"        .fallback = {{ {_f(m['fb_kp'])}, {_f(m['fb_kd'])}, "
            f"{_f(m['fb_tau'])} }},\n"
            "    }"
        )
    cfg_block = ",\n".join(cfg_entries)

    return f"""\
/* AUTO-GENERATED — DO NOT EDIT.
 * Source: {banner_src}
 * Regenerate: {banner_regen}
 */
#ifndef MOTOR_USER_CONFIG_H
#define MOTOR_USER_CONFIG_H

#include <stdint.h>

#include "motor_rs_config.h"   /* tier 1: fixed vendor ranges (MotorModel, motor_rs_configs[]) */
#include "timing_config.h"     /* rates, timeouts, guards — hand-written firmware constants */

#ifdef __cplusplus
extern "C" {{
#endif

#define N_MOTORS {n}u

/* Enable-gate position margin (rad): an ENABLE is accepted only with the shaft
 * inside [p_min - margin, p_max + margin]; outside it is rejected (no fault,
 * motor stays DISABLED). Optional per-slave yaml key: ready_margin_rad. */
#define MOTOR_READY_POS_MARGIN_RAD {margin}

/* ── Motor-config tiers ───────────────────────────────────────────────────────
 * 1. RS (vendor) limits — motor_rs_config.h (included above): fixed per-MODEL
 *    Type-1 encode ranges from the RobStride manuals. Never configurable.
 * 2. USER limits — MotorUserLimits, per MOTOR: the operating envelope we
 *    choose. Field names mirror MotorRsConfig; the tier is carried by the
 *    access path (cfg->user.p_min vs motor_rs_configs[model].v_max). Validated
 *    at generation to fit inside tier 1.
 * 3. Fallback — MotorFallback, per MOTOR: MIT constants the slave applies
 *    AUTONOMOUSLY when the host or master dies (stop-ramp damp dwell), so a
 *    falling robot is damped with known-safe values. pos=current, vel=0 are
 *    implied; kp is normally 0 for a pure backdrive-safe damp.
 * A motor's slave membership is by containment: this header IS one slave's
 * chain, and the array index is the chain index.
 * ──────────────────────────────────────────────────────────────────────────── */

typedef struct {{
    uint8_t     can_id;      /* RobStride bus id            */
    MotorModel  model;       /* index into motor_rs_limits  */
    const char *joint_name;  /* robot joint this motor IS   */
}} MotorIdentity;

typedef struct {{
    float p_min;        /* rad — soft angle range (asymmetric per joint)        */
    float p_max;
    float v_max;        /* rad/s — |vel| command bound                          */
    float t_max;        /* Nm — |tau| command bound AND overtorque fault trip   */
    float default_kp;   /* gains used when the host sends USE_CONFIG_GAINS      */
    float default_kd;
}} MotorUserLimits;

typedef struct {{
    float kp;        /* damp-phase position gain (normally 0)        */
    float kd;        /* damp-phase velocity gain — the damping knob  */
    float tau_ff;    /* damp-phase feedforward torque (normally 0)   */
}} MotorFallback;

typedef struct {{
    MotorIdentity   id;        /* who this motor is                  */
    MotorUserLimits user;      /* tier 2: our operating envelope     */
    MotorFallback   fallback;  /* tier 3: dead-man damp constants    */
}} MotorUserConfig;

static const MotorUserConfig motor_user_configs[N_MOTORS] = {{
{cfg_block}
}};

/* Look up a motor's RS (vendor) Type-1 encode ranges by its bus id. Returns
 * NULL if the id is not part of this slave's chain. */
static inline const MotorRsConfig *motor_rs_config_by_id(uint8_t can_id)
{{
    for (uint8_t i = 0u; i < N_MOTORS; i++) {{
        if (motor_user_configs[i].id.can_id == can_id) {{
            return &motor_rs_configs[motor_user_configs[i].id.model];
        }}
    }}
    return (const MotorRsConfig *)0;
}}

#ifdef __cplusplus
}}
#endif
#endif /* MOTOR_USER_CONFIG_H */
"""


# ─────────────────────────────────────────────────────────────────────────────
#  --system : master C header (master_config.h)
# ─────────────────────────────────────────────────────────────────────────────

def gen_system_header(cfgs, banner_src, banner_regen):
    n_slaves = len(cfgs)
    counts = [len(cfg["motors"]) for cfg in cfgs]
    max_per = max(counts)

    counts_c = ", ".join(f"{c}u" for c in counts)

    return f"""\
/* AUTO-GENERATED — DO NOT EDIT.
 * Sources: {banner_src}
 * Regenerate: {banner_regen}
 */
#ifndef MASTER_CONFIG_H
#define MASTER_CONFIG_H

#include <stdint.h>

#include "timing_config.h"   /* rates, timeouts — hand-written firmware constants */

#ifdef __cplusplus
extern "C" {{
#endif

#define NUM_SLAVES            {n_slaves}u
#define MAX_MOTORS_PER_SLAVE  {max_per}u

/* Number of active motors on each slave (chain order). CAN ids are the
 * slave's business (motor_user_config.h) — the master only counts motors. */
static const uint8_t slave_motor_counts[NUM_SLAVES] = {{ {counts_c} }};

#ifdef __cplusplus
}}
#endif
#endif /* MASTER_CONFIG_H */
"""


# ─────────────────────────────────────────────────────────────────────────────
#  --system : host Python constants (motor_config_gen.py)
# ─────────────────────────────────────────────────────────────────────────────

def gen_python(cfgs, banner_src, banner_regen, cfg_name="", cfg_hash="", rates=None):
    n_slaves = len(cfgs)
    rates = rates or read_rates()

    # Per-slave grouped structure.
    slave_blocks = []
    for s, cfg in enumerate(cfgs):
        slave_name = cfg.get("slave") or f"slave{s}"
        motor_lines = []
        for idx, m in enumerate(cfg["motors"]):
            motor_lines.append(
                "        dict("
                f"idx={idx}, can_id={m['can_id']}, model={m['model']!r}, "
                f"joint_name={m['joint_name']!r}, "
                f"p_min={float(m['u_p_min'])!r}, p_max={float(m['u_p_max'])!r}, "
                f"v_max={float(m['u_v_max'])!r}, t_max={float(m['u_t_max'])!r}, "
                f"default_kp={float(m['u_default_kp'])!r}, default_kd={float(m['u_default_kd'])!r}, "
                f"fallback_kp={float(m['fb_kp'])!r}, fallback_kd={float(m['fb_kd'])!r}, "
                f"fallback_tau={float(m['fb_tau'])!r})"
            )
        motors_block = ",\n".join(motor_lines)
        slave_blocks.append(
            f"    dict(slave={s}, name={slave_name!r}, motors=[\n{motors_block},\n    ])"
        )
    slaves_block = ",\n".join(slave_blocks)

    return f'''\
# AUTO-GENERATED — DO NOT EDIT.
# Sources: {banner_src}
# Regenerate: {banner_regen}
"""Motor configuration constants for host tools (generated, multi-slave)."""

# Active setup name + sha256 of its firmware_config.yaml. The runner recomputes
# the hash and warns if this file is stale.
CONFIG_NAME = {cfg_name!r}
CONFIG_HASH = {cfg_hash!r}

N_SLAVES = {n_slaves}

# Per-slave structure, in slave order. Each motor dict's "idx" is LOCAL to its
# slave (0-based chain index). The host addresses a motor by (slave, idx).
SLAVES = [
{slaves_block},
]

# ── Flattened view (global index = position in this list) ──────────────────────
# Each motor dict gains "slave" (slave index) and keeps its local "idx".
MOTORS = [
    dict(m, slave=s["slave"]) for s in SLAVES for m in s["motors"]
]

N_MOTORS = len(MOTORS)

MOTOR_DEFAULT_KP = [m["default_kp"] for m in MOTORS]
MOTOR_DEFAULT_KD = [m["default_kd"] for m in MOTORS]

# Per-motor USER (tier-2) soft angle limits (rad). Commands are clamped to
# [p_min, p_max]. Named USER to distinguish from the RS vendor ranges (tier 1).
MOTOR_USER_P_MIN = [m["p_min"] for m in MOTORS]
MOTOR_USER_P_MAX = [m["p_max"] for m in MOTORS]

# Motor counts per slave, in slave order.
SLAVE_MOTOR_COUNTS = [len(s["motors"]) for s in SLAVES]

# ── Configured rates (Hz) — parsed from firmware/common/include/timing_config.h
# (the single source); recorded in the .bin header and used as the run_policy
# --rate default (HOST_CMD_HZ). ─────────────────────────────────────────────────
MASTER_POLL_HZ = {rates['MASTER_POLL_HZ']}
TELEMETRY_HZ   = {rates['TELEMETRY_HZ']}
SLAVE_TICK_HZ  = {rates['SLAVE_TICK_HZ']}
HOST_CMD_HZ    = {rates['HOST_CMD_HZ']}

RATES = dict(master_poll_hz=MASTER_POLL_HZ, telemetry_hz=TELEMETRY_HZ,
             slave_tick_hz=SLAVE_TICK_HZ, host_cmd_hz=HOST_CMD_HZ)
'''


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

CONFIGS_DIR = os.path.join(REPO_ROOT, "configs")


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


def config_hash(path):
    """sha256 of the firmware_config.yaml file as generated from."""
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        h.update(fh.read())
    return h.hexdigest()


# ── firmware_config.yaml (schema v2) — the single per-robot config file ──────
def firmware_config_path(setup):
    return os.path.join(CONFIGS_DIR, setup, "firmware_config.yaml")


def load_firmware_config(setup):
    """Parsed, schema-checked configs/<setup>/firmware_config.yaml."""
    p = firmware_config_path(setup)
    rel = os.path.relpath(p, REPO_ROOT)
    if not os.path.isfile(p):
        sys.exit(f"{rel}: missing — every setup needs a firmware_config.yaml "
                 f"(schema v2)")
    doc = load(p)
    if not isinstance(doc, dict) or "slaves" not in doc:
        sys.exit(f"{rel}: not a firmware_config.yaml (no 'slaves' list)")
    stale = [k for k in ("rates", "shared_ranges", "models") if k in doc]
    if stale:
        sys.exit(f"{rel}: legacy schema — {stale} moved out of the config "
                 f"(rates/timeouts are firmware constants in gen_motor_config.py; "
                 f"vendor ranges live in motor_rs_config.h). Migrate to schema v2 "
                 f"(see configs/1s_5m/firmware_config.yaml).")
    return doc


def _fw_cfg(doc, section):
    """A per-slave cfg dict with schema-v2-normalized motors for the emitters."""
    slave = section.get("slave")
    motors = [_norm_motor(m, f"{slave} motor #{i}")
              for i, m in enumerate(section["motors"])]
    margin = section.get("ready_margin_rad", READY_MARGIN_RAD_DEFAULT)
    if not isinstance(margin, (int, float)) or margin < 0:
        sys.exit(f"{slave}: ready_margin_rad must be a non-negative number "
                 f"(got {margin!r})")
    return {"slave": slave, "motors": motors, "ready_margin_rad": float(margin)}


def _fw_section(doc, slave_name):
    for s in doc["slaves"]:
        if s.get("slave") == slave_name:
            return s
    have = [s.get("slave") for s in doc["slaves"]]
    sys.exit(f"slave {slave_name!r} not in firmware_config.yaml (have {have})")


# ── emitters ──
def _write_slave_header(cfg, banner_src, banner_regen):
    validate(cfg)
    header_path = os.path.join(REPO_ROOT, "firmware/common/include/motor_user_config.h")
    with open(header_path, "w") as fh:
        fh.write(gen_header(cfg, banner_src, banner_regen))
    print(f"Generated {os.path.relpath(header_path, REPO_ROOT)}  (slave: {cfg.get('slave', '?')}, "
          f"N_MOTORS={len(cfg['motors'])})")


def _write_system(cfgs, banner_src, banner_regen, cfg_name, cfg_hash):
    for c in cfgs:
        validate(c)
    sys_path = os.path.join(REPO_ROOT, "firmware/common/include/master_config.h")
    py_path = os.path.join(REPO_ROOT, "host/master_link/motor_config_gen.py")
    with open(sys_path, "w") as fh:
        fh.write(gen_system_header(cfgs, banner_src, banner_regen))
    with open(py_path, "w") as fh:
        fh.write(gen_python(cfgs, banner_src, banner_regen, cfg_name, cfg_hash))
    counts = [len(c["motors"]) for c in cfgs]
    print(f"Generated {os.path.relpath(sys_path, REPO_ROOT)}  (NUM_SLAVES={len(cfgs)}, counts={counts})")
    print(f"Generated {os.path.relpath(py_path, REPO_ROOT)}")


# ── slave header ──
def do_slave_name(setup, slave_name):
    """Emit motor_user_config.h for one slave section of `setup`'s firmware_config.yaml."""
    doc = load_firmware_config(setup)
    cfg = _fw_cfg(doc, _fw_section(doc, slave_name))
    _write_slave_header(
        cfg,
        f"configs/{setup}/firmware_config.yaml (section {slave_name})",
        f"SOCCER_SETUP={setup} python3 scripts/gen_motor_config.py --slave {slave_name}")


# ── system headers ──
def do_system_setup(setup):
    """Emit master_config.h + motor_config_gen.py for `setup` (all slaves)."""
    doc = load_firmware_config(setup)
    cfgs = [_fw_cfg(doc, s) for s in doc["slaves"]]
    _write_system(
        cfgs,
        f"configs/{setup}/firmware_config.yaml",
        f"SOCCER_SETUP={setup} python3 scripts/gen_motor_config.py --system",
        setup, config_hash(firmware_config_path(setup)))


def main():
    args = sys.argv[1:]
    setup = _require_setup()
    if not args:
        # Regenerate everything for the active setup: system files + the default
        # per-slave header (first slave).
        doc = load_firmware_config(setup)
        print(f"Active setup: {setup}  ({len(doc['slaves'])} slave(s))")
        do_system_setup(setup)
        do_slave_name(setup, doc["slaves"][0]["slave"])
        return
    if args[0] == "--slave":
        if len(args) != 2:
            sys.exit("usage: gen_motor_config.py --slave <slaveN>")
        do_slave_name(setup, args[1])
    elif args[0] == "--system":
        if len(args) != 1:
            sys.exit("usage: gen_motor_config.py --system   (reads the active "
                     "setup's firmware_config.yaml; explicit paths are gone "
                     "with the legacy slaveN.yaml format)")
        do_system_setup(setup)
    elif args[0].startswith("--"):
        sys.exit(f"unknown option {args[0]!r}; use --slave or --system")
    else:
        do_slave_name(setup, args[0])


if __name__ == "__main__":
    main()
