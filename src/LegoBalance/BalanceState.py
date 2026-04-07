"""BalanceState dataclass.

The single canonical state vector that the rest of the project speaks. Every
estimator returns one. Every controller consumes one. Adding a new field here
is the only way to extend the state.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class BalanceState:
    """Estimated state of the two wheel inverted pendulum.

    Attributes:
        tilt: Body tilt angle in radians. ``0`` means upright. Positive means
            the body is leaning forward.
        tiltRate: Body tilt rate in radians per second. Positive when the
            tilt is increasing.
        wheelPosition: Forward wheel base position along the floor in meters.
            Positive when rolling forward.
        wheelVelocity: Forward wheel base velocity along the floor in meters
            per second.
        timestamp: Time of estimation in seconds. Source is up to the caller
            (monotonic clock on the desktop, ``StopWatch`` on the hub).
        valid: ``False`` until the estimator has produced its first
            consistent estimate. Controllers should not act on an invalid
            state.
    """

    tilt: float = 0.0
    tiltRate: float = 0.0
    wheelPosition: float = 0.0
    wheelVelocity: float = 0.0
    timestamp: float = 0.0
    valid: bool = False

    def AsList(self) -> list[float]:
        """Return ``[tilt, tiltRate, wheelPosition, wheelVelocity]``.

        Convenience for tests and for any code that wants to treat the state
        as a flat numerical vector. This is the ``[theta, thetaDot, p, pDot]``
        state used by the balancing controller.
        """
        return [self.tilt, self.tiltRate, self.wheelPosition, self.wheelVelocity]

    def Copy(self) -> BalanceState:
        """Return a deep copy. Used by the logger so log records are stable."""
        return BalanceState(
            tilt=self.tilt,
            tiltRate=self.tiltRate,
            wheelPosition=self.wheelPosition,
            wheelVelocity=self.wheelVelocity,
            timestamp=self.timestamp,
            valid=self.valid,
        )


@dataclass
class StateBounds:
    """Optional per field bounds used by the safety monitor.

    Kept here next to the state itself so that the bounds and the state
    cannot drift apart.
    """

    tiltMax: float = 0.6
    tiltRateMax: float = 10.0
    wheelVelocityMax: float = 10.0
    additional: list[str] = field(default_factory=list)
