# AUTO-GENERATED — DO NOT EDIT.
# Sources: slave0.yaml, slave1.yaml
# Regenerate: python3 scripts/gen_motor_config.py --system configs/slave0.yaml configs/slave1.yaml
"""Motor configuration constants for host tools (generated, multi-slave)."""

N_SLAVES = 2

# Per-slave structure, in slave order. Each motor dict's "idx" is LOCAL to its
# slave (0-based chain index). The host addresses a motor by (slave, idx).
SLAVES = [
    dict(slave=0, name='slave0', motors=[
        dict(idx=0, can_id=1, model='RS02', joint_name='motor0', soft_min=-0.79, soft_max=0.79, max_vel=10.0, max_tau=0.8, default_kp=15.0, default_kd=1.0),
        dict(idx=1, can_id=2, model='RS02', joint_name='motor1', soft_min=-1.05, soft_max=1.05, max_vel=10.0, max_tau=0.8, default_kp=15.0, default_kd=1.0),
        dict(idx=2, can_id=3, model='RS00', joint_name='motor2', soft_min=-1.31, soft_max=1.31, max_vel=10.0, max_tau=0.8, default_kp=15.0, default_kd=1.0),
        dict(idx=3, can_id=4, model='RS00', joint_name='motor3', soft_min=-1.05, soft_max=1.05, max_vel=10.0, max_tau=0.8, default_kp=15.0, default_kd=1.0),
        dict(idx=4, can_id=5, model='RS00', joint_name='motor4', soft_min=-0.52, soft_max=0.52, max_vel=10.0, max_tau=0.8, default_kp=15.0, default_kd=1.0),
    ]),
    dict(slave=1, name='slave1', motors=[
        dict(idx=0, can_id=6, model='RS02', joint_name='motor6', soft_min=-0.79, soft_max=0.79, max_vel=10.0, max_tau=0.8, default_kp=15.0, default_kd=1.0),
        dict(idx=1, can_id=7, model='RS00', joint_name='motor7', soft_min=-0.79, soft_max=0.79, max_vel=10.0, max_tau=0.8, default_kp=15.0, default_kd=1.0),
    ]),
]

# Per-model CAN velocity/torque ranges (union across slaves).
MODEL_RANGES = {
    'RS00': dict(v_min=-33.0, v_max=33.0, t_min=-14.0, t_max=14.0),
    'RS02': dict(v_min=-44.0, v_max=44.0, t_min=-17.0, t_max=17.0),
}

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
