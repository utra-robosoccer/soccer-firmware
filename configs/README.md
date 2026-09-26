# Configs

Robot and motor parameter files live here. Keep values that both host tools and
firmware need in machine-readable files, then generate C headers and Python
constants from them instead of copying limits by hand.

## Active setup (single switch)

We run in two physical places — the **bench** and the **robot** — each with its
own motor chains. Which one is live is selected in ONE place:

```
configs/
  active            <- one line: the active setup name (e.g. `bench-1-chain`)
  bench-1-chain/
    slave0.yaml
  bench-2-chain/
    slave0.yaml
    slave1.yaml
  robot/
    slave0.yaml
    slave1.yaml
```

`configs/active` names a subdirectory. Switch setups by editing that one line:

```sh
echo robot > configs/active     # then regenerate + rebuild
```

For a one-off without touching the file, set `SOCCER_SETUP` (it overrides
`configs/active`):

```sh
SOCCER_SETUP=robot python3 scripts/gen_motor_config.py
```

The generator and `scripts/build.sh` both resolve slave configs through this
switch, so a bare name like `slave0` always means "slave0 of the active setup"
(`configs/<active>/slave0.yaml`). Add a new setup by creating a folder and
dropping `slaveN.yaml` files in it — no code changes.

## Single source of truth

Each `slaveN.yaml` describes one slave's motor chain: motor count, per-motor
identity (`idx`, `can_id`, `model`, `joint_name`), soft limits / gains, and the
per-model CAN "operation control mode" (Communication Type 1) velocity and
torque ranges. Position (±4π), Kp and Kd are shared within a model size class,
so only velocity and torque are model-specific. See
`docs/robostride-motor-reference.md` for the full per-model capability tables.

## Generating

```sh
python3 scripts/gen_motor_config.py                 # regen everything for the active setup
python3 scripts/gen_motor_config.py --slave slave0  # just slave0's header (active setup)
python3 scripts/gen_motor_config.py --system        # just the master + host files (active setup)
```

An explicit path still works and ignores the active switch
(`--slave configs/robot/slave0.yaml`). Generated files (each carries an
`AUTO-GENERATED — DO NOT EDIT` banner — edit the YAML, not these):

- `firmware/common/include/motor_config.h` — per-slave header for the slave and
  master firmware (`N_MOTORS`, transport encoding bounds, the `MotorModel` enum,
  the per-model `motor_can_ranges[]` table, `motor_configs[]`, and the
  `motor_can_range_by_id()` lookup). The slave firmware build regenerates this
  for the slave it targets via `scripts/build.sh --config slaveN`.
- `firmware/common/include/system_config.h` — master's system view across all
  slaves (`NUM_SLAVES`, per-slave motor counts + CAN-id LUTs, global transport
  bounds).
- `tools/motor_config_gen.py` — Python constants imported by `test_client.py`
  (`SLAVES`, `MOTORS`, `MODEL_RANGES`, default gains, soft limits).

After editing a YAML, regenerate and rebuild the firmware. The firmware build
does not run Python, so the generated files are committed alongside the source.
The generated banners record which setup/slave they came from.
