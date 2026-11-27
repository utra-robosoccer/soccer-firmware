"""Policy interface and the Action it returns.

An Action carries a per-motor MotorCommand list (level-triggered request +
mit) plus the set of slaves whose SLAVE_REQ_RESET to assert this tick; the
runner sends them as one cmd_master_t per tick. The command/state types are
owned by master_link.

Designed so a manual_policy (its own keyboard thread queuing intents) drops in
without changing the runner: step() returns whatever the policy decided this tick.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

# Re-exported for policy authors: build these in step().
from master_link.link import (  # noqa: F401
    MotorCommand, LinkState,
    REQ_STOP, REQ_ENABLE,
)
from master_link import motor_config_gen as _mc

# Activation window default (seconds into the arm window): a STOP baseline,
# then ENABLE + hold. There is deliberately NO automatic fault clear here —
# a latched MOTOR_FAULT / MOTOR_ABSENT needs the slave-level SLAVE_REQ_RESET
# (a blocking re-sweep), requested explicitly via run_policy --reset-slaves:
# stop and investigate, don't auto-recover.
ARM_ENABLE_S = 0.30

# Per-motor default gains from the generated config, keyed (slave, local).
# The host sends gains EXPLICITLY in every mit — there is no config-gains flag.
_DEFAULT_GAINS = {(m["slave"], m["idx"]): (m["default_kp"], m["default_kd"])
                  for m in _mc.MOTORS}


def default_gains(slave, local):
    """(kp, kd) for one motor from the generated config."""
    return _DEFAULT_GAINS[(slave, local)]


def arm_command(slave, local, el, pos):
    """One motor's command during the activation window: STOP baseline, then
    ENABLE while streaming a hold mit at `pos` with the config-default gains
    sent explicitly. Streaming the mit through ENABLING closes the limp
    window — the slave latches it and applies it on the ENABLING→ENABLED
    edge."""
    if el < ARM_ENABLE_S:
        return MotorCommand(slave, local, req=REQ_STOP)
    kp, kd = default_gains(slave, local)
    return MotorCommand(slave, local, req=REQ_ENABLE, pos=pos, kp=kp, kd=kd)


@dataclass
class Action:
    motors: list = field(default_factory=list)         # list[MotorCommand]
    reset_slaves: set = field(default_factory=set)     # slave indices → SLAVE_REQ_RESET

    def is_empty(self) -> bool:
        return not self.motors and not self.reset_slaves


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
