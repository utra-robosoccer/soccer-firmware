"""ListenPolicy — requests IDLE for every motor each tick.

Under the level-triggered protocol a policy sends a mode request every tick;
"listen" simply holds every motor in IDLE (no motion), so the link still logs
telemetry while nothing is driven. Use for passive capture (moving a joint by
hand) and as the reference for the Policy interface.
"""
from master_link.motor_config_gen import MOTORS

from .base import Policy, Action, MotorCommand, MODE_IDLE, LinkState


class ListenPolicy(Policy):
    name = "listen"

    def __init__(self):
        self._motors = [(m["slave"], m["idx"]) for m in MOTORS]

    def step(self, state: LinkState, t_ns: int) -> Action:
        return Action(motors=[MotorCommand(s, l, mode=MODE_IDLE)
                              for (s, l) in self._motors])
