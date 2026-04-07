"""Mock adapters for hardware free testing and simulation.

These implement the abstract interfaces with simple, predictable behavior so
that desktop side tests and the simulation example can run without a hub.
The dynamics in :class:`MockHub` are intentionally crude. They are good
enough to exercise the controller API and to catch shape errors. They are
not a substitute for a real plant model.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .HubInterface import HubInterface
from .ImuInterface import ImuInterface
from .MotorInterface import MotorInterface
from .Saturation import SaturateSymmetric
from .Units import RadPerSecToDegPerSec, RadToDeg


@dataclass
class _MotorState:
    angleRad: float = 0.0
    velocityRadPerSec: float = 0.0
    commandedRadPerSec: float = 0.0


class MockMotor(MotorInterface):
    """A first order motor model.

    The commanded velocity is followed with a single time constant ``tau``.
    Encoder angle is integrated from velocity. The model is intentionally
    simple. The point is to give the controller something to push on, not to
    represent a real LEGO motor.
    """

    def __init__(self, maxRadPerSec: float = 10.0, tau: float = 0.05) -> None:
        self._state = _MotorState()
        self._maxRadPerSec = maxRadPerSec
        self._tau = max(tau, 1e-3)

    def Step(self, dt: float) -> None:
        """Advance the internal model by ``dt`` seconds."""
        if dt <= 0.0:
            return
        target = self._state.commandedRadPerSec
        # Exponential approach to the commanded value.
        decay = math.exp(-dt / self._tau)
        self._state.velocityRadPerSec = (
            target + (self._state.velocityRadPerSec - target) * decay
        )
        self._state.angleRad += self._state.velocityRadPerSec * dt

    def Angle(self) -> float:
        return self._state.angleRad

    def Velocity(self) -> float:
        return self._state.velocityRadPerSec

    def RunVelocity(self, radPerSec: float) -> None:
        if not math.isfinite(radPerSec):
            raise ValueError("RunVelocity received a non finite value")
        self._state.commandedRadPerSec = SaturateSymmetric(radPerSec, self._maxRadPerSec)

    def RunDuty(self, dutyPercent: float) -> None:
        # Treat duty as a linear proxy for velocity for the mock.
        clipped = SaturateSymmetric(dutyPercent, 100.0)
        self.RunVelocity(self._maxRadPerSec * clipped / 100.0)

    def Stop(self) -> None:
        self._state.commandedRadPerSec = 0.0

    def Brake(self) -> None:
        self._state.commandedRadPerSec = 0.0
        self._state.velocityRadPerSec = 0.0


class MockImu(ImuInterface):
    """A trivial IMU mock backed by an internal tilt and tilt rate.

    The :class:`MockHub` is responsible for advancing these values to
    represent the body of a (very simplified) inverted pendulum.
    """

    def __init__(self) -> None:
        self._tiltRad = 0.0
        self._tiltRateRadPerSec = 0.0
        self._calibrated = True

    def SetTilt(self, tiltRad: float, tiltRateRadPerSec: float) -> None:
        self._tiltRad = tiltRad
        self._tiltRateRadPerSec = tiltRateRadPerSec

    def TiltAngleRadians(self) -> float:
        return self._tiltRad

    def TiltRateRadiansPerSec(self) -> float:
        return self._tiltRateRadPerSec

    def IsCalibrated(self) -> bool:
        return self._calibrated

    def Calibrate(self, sampleCount: int = 200) -> None:
        self._calibrated = True


class MockHub(HubInterface):
    """A toy plant model for the simulation example and the unit tests.

    The hub owns:

    - one :class:`MockImu` representing the body
    - two :class:`MockMotor` instances representing the wheels

    Each call to :meth:`Step` integrates a small inverted pendulum like
    plant. The dynamics are intentionally simplified: gravity drives tilt
    forward, wheel commands push tilt backward. They are not physically
    correct. They exist only so that the controller API has something to
    react to in tests.

    Use a real model (or real hardware) when you start tuning anything.
    """

    def __init__(
        self,
        gravityCoupling: float = 9.0,
        wheelCoupling: float = 0.5,
        damping: float = 0.05,
    ) -> None:
        self._imu = MockImu()
        self._left = MockMotor()
        self._right = MockMotor()
        self._gravityCoupling = gravityCoupling
        self._wheelCoupling = wheelCoupling
        self._damping = damping
        self._connected = True
        self._batteryVolts = 7.6
        self._tStart = time.monotonic()

    # ----- Convenience accessors used by the simulation example. -----
    @property
    def LeftMotor(self) -> MockMotor:
        return self._left

    @property
    def RightMotor(self) -> MockMotor:
        return self._right

    @property
    def Imu(self) -> MockImu:
        return self._imu

    def SetInitialTilt(self, tiltRad: float) -> None:
        self._imu.SetTilt(tiltRad, 0.0)

    def Step(self, dt: float) -> None:
        """Advance both motors and the toy body model by ``dt`` seconds."""
        if dt <= 0.0:
            return
        self._left.Step(dt)
        self._right.Step(dt)
        # Toy body model: thetaDDot = g * sin(theta) - k * meanWheelRate - c * thetaDot
        meanWheelRate = 0.5 * (self._left.Velocity() + self._right.Velocity())
        tilt = self._imu.TiltAngleRadians()
        tiltRate = self._imu.TiltRateRadiansPerSec()
        accel = (
            self._gravityCoupling * math.sin(tilt)
            - self._wheelCoupling * meanWheelRate
            - self._damping * tiltRate
        )
        newRate = tiltRate + accel * dt
        newTilt = tilt + newRate * dt
        self._imu.SetTilt(newTilt, newRate)

    # ----- HubInterface implementation. -----
    def IsConnected(self) -> bool:
        return self._connected

    def BatteryVoltage(self) -> float:
        return self._batteryVolts

    def Now(self) -> float:
        return time.monotonic() - self._tStart

    def TiltDegrees(self) -> tuple[float, float]:
        return RadToDeg(self._imu.TiltAngleRadians()), 0.0

    def AngularVelocityDegreesPerSec(self) -> tuple[float, float, float]:
        return 0.0, RadPerSecToDegPerSec(self._imu.TiltRateRadiansPerSec()), 0.0

    def Shutdown(self) -> None:
        self._left.Stop()
        self._right.Stop()
        self._connected = False
