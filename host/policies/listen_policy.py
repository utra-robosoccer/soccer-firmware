"""ListenPolicy — sends nothing; just lets the link log telemetry.

Use for passive capture (e.g. moving a joint by hand) and as the reference for the
Policy interface.
"""
from .base import Policy, Action, LinkState


class ListenPolicy(Policy):
    name = "listen"

    def step(self, state: LinkState, t_ns: int) -> Action:
        return Action()
