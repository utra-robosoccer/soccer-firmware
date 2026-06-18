# Configs

Robot and motor parameter files live here. Keep values that both host tools and
firmware need in machine-readable files, then generate C headers and Python
constants from them instead of copying limits by hand.

## Single source of truth

`slave0.yaml` describes the motor chain on SLAVE-0: motor count, per-motor
identity (`idx`, `can_id`, `model`, `joint_name`), soft limits / gains, and the
per-model CAN "operation control mode" (Communication Type 1) velocity and
torque ranges. Position (±4π), Kp (0–500) and Kd (0–5) are identical across the
RS00 and RS02 models, so only velocity and torque are model-specific.

## Generating

```sh
python3 scripts/gen_motor_config.py configs/slave0.yaml
```

This regenerates two files (each carries an `AUTO-GENERATED — DO NOT EDIT`
banner — edit the YAML, not these):

- `firmware/common/include/motor_config.h` — consumed by both the slave and
  master firmware (`N_MOTORS`, transport encoding bounds, the `MotorModel` enum,
  the per-model `motor_can_ranges[]` table, `motor_configs[]`, and the
  `motor_can_range_by_id()` lookup used for per-model CAN scaling).
- `tools/motor_config_gen.py` — Python constants imported by `test_client.py`
  (`N_MOTORS`, `MOTORS`, `MODEL_RANGES`, default gains).

After editing the YAML, regenerate and rebuild the firmware. The firmware build
does not run Python, so the generated header is committed alongside the source.
