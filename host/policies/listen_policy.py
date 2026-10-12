"""ListenPolicy — requests DISABLE for every motor each tick.

Under the level-triggered protocol a policy sends a drive request every tick;
"listen" simply keeps every motor DISABLED (no torque), so the link still logs
telemetry while nothing is driven. Use for passive capture (moving a joint by
hand) and as the reference for the Policy interface.
"""
from master_link.motor_config_gen import MOTORS

from .base import Policy, Action, MotorCommand, REQ_DISABLE, LinkState


class ListenPolicy(Policy):
    name = "listen"

    def __init__(self):
        self._motors = [(m["slave"], m["idx"]) for m in MOTORS]

    def step(self, state: LinkState, t_ns: int) -> Action:
        return Action(motors=[MotorCommand(s, l, req=REQ_DISABLE)
                              for (s, l) in self._motors])
