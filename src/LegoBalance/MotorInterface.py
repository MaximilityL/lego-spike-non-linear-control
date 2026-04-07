"""Abstract MotorInterface.

Single motor abstraction. The robot uses two of these (left and right). All
units returned and accepted by this interface are SI: radians and rad/s.
Conversions to and from Pybricks degrees are the adapter's responsibility.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class MotorInterface(ABC):
    """One drive motor."""

    @abstractmethod
    def Angle(self) -> float:
        """Encoder angle in radians."""

    @abstractmethod
    def Velocity(self) -> float:
        """Encoder velocity in radians per second."""

    @abstractmethod
    def RunVelocity(self, radPerSec: float) -> None:
        """Command an angular velocity in radians per second.

        Implementations should clip to the configured maximum and refuse
        any non finite input.
        """

    @abstractmethod
    def RunDuty(self, dutyPercent: float) -> None:
        """Command an open loop duty cycle in the range ``[-100, 100]``.

        Useful for low level characterization. Higher level code should
        prefer :meth:`RunVelocity`.
        """

    @abstractmethod
    def Stop(self) -> None:
        """Release the motor (no holding torque)."""

    @abstractmethod
    def Brake(self) -> None:
        """Brake the motor (short the windings)."""
