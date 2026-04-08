"""Shared data containers and command values used at the controller boundary.

These are deliberately small so they are safe to log, copy, and pass between
threads if you ever need to.
"""

from __future__ import annotations


class _EnumValue(str):
    @property
    def value(self) -> str:
        return str(self)


class ControlMode:
    """Which kind of motor command the controller is producing.

    ``Velocity`` is the easiest to bring up because Pybricks already
    implements an inner velocity loop. ``Torque`` is closer to what a
    Lyapunov design would naturally output but requires either a torque
    constant calibration or accepting that ``Motor.dc`` is only a duty
    proxy.
    """

    Velocity = _EnumValue("velocity")
    Torque = _EnumValue("torque")
    DutyCycle = _EnumValue("duty")

class ControlOutput:
    """One step of controller output.

    Attributes:
        leftCommand: Command for the left wheel in the controller/chassis
            convention and in the units implied by ``mode``. Positive
            velocity means forward wheel-base motion / increasing ``phi``;
            hardware adapters apply motor mounting signs before calling raw
            motor APIs.
        rightCommand: Command for the right wheel in the same convention as
            ``leftCommand``.
        mode: How the commands should be interpreted.
        timestamp: Time at which the controller produced this output.
    """

    def __init__(
        self,
        leftCommand: float,
        rightCommand: float,
        mode: ControlMode,
        timestamp: float = 0.0,
    ) -> None:
        self.leftCommand = leftCommand
        self.rightCommand = rightCommand
        self.mode = mode
        self.timestamp = timestamp

    @staticmethod
    def Stop(mode: ControlMode = ControlMode.Velocity, timestamp: float = 0.0) -> ControlOutput:
        """Return a zero command in the given mode.

        Used by the safety monitor to halt the motors when a fault is
        detected.
        """
        return ControlOutput(leftCommand=0.0, rightCommand=0.0, mode=mode, timestamp=timestamp)


class Measurement:
    """Raw measurement bundle handed to the estimator.

    All units are SI radians and SI radians per second. Conversions from
    Pybricks units happen in the adapter, not here.
    """

    def __init__(
        self,
        tiltAngle: float,
        tiltRate: float,
        leftWheelAngle: float,
        rightWheelAngle: float,
        leftWheelRate: float,
        rightWheelRate: float,
        timestamp: float,
        valid: bool = True,
    ) -> None:
        self.tiltAngle = tiltAngle
        self.tiltRate = tiltRate
        self.leftWheelAngle = leftWheelAngle
        self.rightWheelAngle = rightWheelAngle
        self.leftWheelRate = leftWheelRate
        self.rightWheelRate = rightWheelRate
        self.timestamp = timestamp
        self.valid = valid
