"""Abstract HubInterface.

Defines the surface that the rest of the desktop side code uses to talk to a
"hub". The real implementation lives in the hub side scripts under ``hub/``
and uses ``pybricks.hubs.PrimeHub``. Tests use ``MockAdapters.MockHub``.

The point of this interface is that no desktop side module ever imports from
``pybricks``. That keeps the desktop side testable on any laptop.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class HubInterface(ABC):
    """Minimal hub level capabilities required by the project."""

    @abstractmethod
    def IsConnected(self) -> bool:
        """Return ``True`` when the hub is reachable and ready."""

    @abstractmethod
    def BatteryVoltage(self) -> float:
        """Return battery voltage in volts.

        Pybricks exposes ``hub.battery.voltage()`` which returns millivolts.
        Adapters convert to volts before returning here.
        """

    @abstractmethod
    def Now(self) -> float:
        """Return a monotonic time in seconds.

        On the hub this wraps ``StopWatch().time()``. On the desktop the
        mock uses ``time.monotonic()``.
        """

    @abstractmethod
    def TiltDegrees(self) -> tuple[float, float]:
        """Return ``(pitch_deg, roll_deg)`` from the IMU.

        Returned in degrees because that is what Pybricks gives us. The
        :class:`ImuInterface` is the place where conversion to radians
        happens.
        """

    @abstractmethod
    def AngularVelocityDegreesPerSec(self) -> tuple[float, float, float]:
        """Return ``(gx, gy, gz)`` body rates in deg/s from the IMU."""

    @abstractmethod
    def Shutdown(self) -> None:
        """Best effort safe shutdown.

        Implementations should stop motors and release locks. Safe to call
        multiple times.
        """
