"""Abstract controller base class.

All controllers in the project subclass this. The interface is intentionally
small: one method, one return type. Anything fancier (gain scheduling, mode
switching, anti windup) lives inside a specific controller subclass.
"""

from __future__ import annotations

from .BalanceState import BalanceState
from .ControlInterfaces import ControlOutput


class ControllerBase:
    """Base class for any closed loop controller.

    Subclasses receive the :class:`RobotConfig` at construction time so that
    physical parameters and limits are available without global state.
    """

    def __init__(self, config: object) -> None:
        self.config = config

    def Compute(self, state: BalanceState) -> ControlOutput:
        """Map the current state estimate to a motor command.

        Should be pure: same state in, same command out. Side effects (logging,
        writing to motors) belong elsewhere.
        """
        raise NotImplementedError

    def Reset(self) -> None:  # noqa: B027  intentional optional hook with no op default
        """Optional hook called before a new run. Default is a no op.

        Override this in stateful controllers to clear integrators, hold
        registers, mode flags, and so on.
        """
