"""Policy interface and the Action it returns.

An Action can carry per-motor MIT commands, control requests (arm/zero/disable),
both, or nothing. The command/state types are owned by master_link (they're
wire-adjacent); policies just compose them.

Designed so a future manual_policy (its own keyboard thread queuing intents) drops
in without changing the runner: step() simply returns whatever the policy has
decided this tick.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

# Re-exported for policy authors: build these in step().
from master_link.link import (  # noqa: F401
    MitCommand, ControlRequest, ControlKind, LinkState,
)


@dataclass
class Action:
    mit: list = field(default_factory=list)      # list[MitCommand]
    control: list = field(default_factory=list)  # list[ControlRequest]

    def is_empty(self) -> bool:
        return not self.mit and not self.control


class Policy(ABC):
    """Base class. name is used for the log filename and header producer field."""
    name: str = "policy"

    def setup(self, state: LinkState) -> None:
        """Optional one-time init once the link is up (state may be sparse)."""

    @abstractmethod
    def step(self, state: LinkState) -> Action:
        """Given the latest link state, return the Action for this tick."""

    def teardown(self) -> None:
        """Optional cleanup on shutdown (before motors are disabled)."""
