"""Policy interface and the Action it returns.

Under the robot/chain/motor wire protocol an Action carries a per-motor
MotorCommand list (level-triggered mode + targets); the runner sends them as one
cmd_robot_t per tick. The command/state types are owned by master_link.

Designed so a manual_policy (its own keyboard thread queuing intents) drops in
without changing the runner: step() returns whatever the policy decided this tick.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

# Re-exported for policy authors: build these in step().
from master_link.link import (  # noqa: F401
    MotorCommand, LinkState,
    MODE_IDLE, MODE_HOLD, MODE_MIT, MODE_DAMPED, MODE_TO_ZERO,
)


@dataclass
class Action:
    motors: list = field(default_factory=list)   # list[MotorCommand]

    def is_empty(self) -> bool:
        return not self.motors


class Policy(ABC):
    """Base class. name is used for the log filename and header producer field."""
    name: str = "policy"

    def setup(self, state: LinkState, t_ns: int) -> None:
        """Optional one-time init once the link is up (state may be sparse).
        t_ns is the MASTER clock in nanoseconds (the telemetry frame's time)."""

    @abstractmethod
    def step(self, state: LinkState, t_ns: int) -> Action:
        """Given the latest link state, return the Action for this tick.

        t_ns is MASTER time in nanoseconds (the stepped telemetry frame's master clock),
        not the host clock — so step cadence is perfectly regular and a logged run replays
        identically. Policies MUST derive all timing from t_ns and never read a clock
        themselves — this keeps step() a pure function of (state, t_ns)."""

    def teardown(self) -> None:
        """Optional cleanup on shutdown (before motors are disabled)."""
