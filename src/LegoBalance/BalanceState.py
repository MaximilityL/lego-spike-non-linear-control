"""BalanceState dataclass.

The single canonical state vector that the rest of the project speaks. Every
estimator returns one. Every controller consumes one. Adding a new field here
is the only way to extend the state.

Naming convention used in this project (pre balancing phase):

* ``tilt`` is the body tilt angle ``theta`` around the balancing axis. Zero
  means upright, positive means leaning forward.
* ``tiltRate`` is ``thetaDot``, the body tilt rate.
* ``phi`` is the mean wheel rotation angle in radians, taken as the average
  of the two sign corrected wheel encoder angles. ``phi`` is the primary
  implemented wheel motion state at this stage of the project because it
  comes most directly from the hardware encoders and does not depend on the
  exact wheel radius.
* ``phiDot`` is the mean wheel rotation rate in radians per second.

The pure translational state ``p`` (linear distance in meters) and ``pDot``
(linear velocity in meters per second) is *not* stored on the dataclass.
``p`` and ``pDot`` can always be derived from ``phi`` and ``phiDot`` if a
reliable wheel radius is available, via the helper methods
:meth:`LinearPosition` and :meth:`LinearVelocity`. They are intentionally a
secondary, derived view, because we do not want the core state estimate to
silently depend on a calibration value (the wheel radius) that may not yet
be finalized.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class BalanceState:
    """Estimated state of the two wheel inverted pendulum.

    Attributes:
        tilt: Body tilt angle ``theta`` in radians. ``0`` means upright.
            Positive means the body is leaning forward.
        tiltRate: Body tilt rate ``thetaDot`` in radians per second.
            Positive when the tilt is increasing.
        phi: Mean wheel rotation angle in radians. Mean of the two sign
            corrected wheel encoder angles. Positive when the wheel base
            has rolled forward.
        phiDot: Mean wheel rotation rate in radians per second. Positive
            when the wheel base is rolling forward.
        timestamp: Time of estimation in seconds. Source is up to the caller
            (monotonic clock on the desktop, ``StopWatch`` on the hub).
        valid: ``False`` until the estimator has produced its first
            consistent estimate. Controllers should not act on an invalid
            state.
    """

    tilt: float = 0.0
    tiltRate: float = 0.0
    phi: float = 0.0
    phiDot: float = 0.0
    timestamp: float = 0.0
    valid: bool = False

    # Convenience aliases so callers that prefer the math notation can use
    # ``state.theta`` / ``state.thetaDot`` instead of ``state.tilt`` /
    # ``state.tiltRate``. They are read only views; the dataclass fields
    # remain the single source of truth.
    @property
    def theta(self) -> float:
        """Alias for :attr:`tilt`. Body tilt angle in radians."""
        return self.tilt

    @property
    def thetaDot(self) -> float:
        """Alias for :attr:`tiltRate`. Body tilt rate in radians per second."""
        return self.tiltRate

    def AsList(self) -> list[float]:
        """Return ``[tilt, tiltRate, phi, phiDot]``.

        Convenience for tests and for any code that wants to treat the state
        as a flat numerical vector. This is the ``[theta, thetaDot, phi, phiDot]``
        state used by the future balancing controller.
        """
        return [self.tilt, self.tiltRate, self.phi, self.phiDot]

    def Copy(self) -> BalanceState:
        """Return a deep copy. Used by the logger so log records are stable."""
        return BalanceState(
            tilt=self.tilt,
            tiltRate=self.tiltRate,
            phi=self.phi,
            phiDot=self.phiDot,
            timestamp=self.timestamp,
            valid=self.valid,
        )

    # ----- Optional translation conversion. -----
    #
    # ``phi`` and ``phiDot`` are the implemented core state. The two helpers
    # below let callers compute the linear translation ``p`` and ``pDot``
    # whenever a wheel radius is available, without baking the wheel radius
    # into the estimator itself.

    def LinearPosition(self, wheelRadius: float) -> float:
        """Return the linear wheel base position ``p = r * phi`` in meters.

        Args:
            wheelRadius: Wheel rolling radius in meters. Must be positive.

        Returns:
            Linear distance the wheel base has rolled forward, in meters.
        """
        if wheelRadius <= 0.0:
            raise ValueError("wheelRadius must be positive to compute LinearPosition")
        return wheelRadius * self.phi

    def LinearVelocity(self, wheelRadius: float) -> float:
        """Return the linear wheel base velocity ``pDot = r * phiDot`` in m/s.

        Args:
            wheelRadius: Wheel rolling radius in meters. Must be positive.

        Returns:
            Forward velocity of the wheel base, in meters per second.
        """
        if wheelRadius <= 0.0:
            raise ValueError("wheelRadius must be positive to compute LinearVelocity")
        return wheelRadius * self.phiDot


@dataclass
class StateBounds:
    """Optional per field bounds used by the safety monitor.

    Kept here next to the state itself so that the bounds and the state
    cannot drift apart.
    """

    tiltMax: float = 1.0
    tiltRateMax: float = 10.0
    phiDotMax: float = 17.453292519943295
    additional: list[str] = field(default_factory=list)
