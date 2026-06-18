# AUTO-GENERATED — DO NOT EDIT.
# Source: configs/slave0.yaml
# Regenerate: python3 scripts/gen_motor_config.py configs/slave0.yaml
"""Motor configuration constants for host tools (generated)."""

N_MOTORS = 5

# One dict per motor, in chain order (index == motor_idx).
MOTORS = [
    dict(idx=0, can_id=1, model='RS02', joint_name='motor0', default_kp=15.0, default_kd=1.0),
    dict(idx=1, can_id=2, model='RS02', joint_name='motor1', default_kp=15.0, default_kd=1.0),
    dict(idx=2, can_id=3, model='RS00', joint_name='motor2', default_kp=15.0, default_kd=1.0),
    dict(idx=3, can_id=4, model='RS00', joint_name='motor3', default_kp=15.0, default_kd=1.0),
    dict(idx=4, can_id=5, model='RS00', joint_name='motor4', default_kp=15.0, default_kd=1.0),
]

# Per-model CAN velocity/torque ranges.
MODEL_RANGES = {
    'RS00': dict(v_min=-33.0, v_max=33.0, t_min=-14.0, t_max=14.0),
    'RS02': dict(v_min=-44.0, v_max=44.0, t_min=-17.0, t_max=17.0),
}

MOTOR_DEFAULT_KP = [m["default_kp"] for m in MOTORS]
MOTOR_DEFAULT_KD = [m["default_kd"] for m in MOTORS]
