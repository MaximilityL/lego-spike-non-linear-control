"""Shared dataclasses and enums used at the controller boundary.

These are deliberately small and frozen so they are safe to log, copy, and
pass between threads if you ever need to.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ControlMode(str, Enum):
    """Which kind of motor command the controller is producing.

    ``Velocity`` is the easiest to bring up because Pybricks already
    implements an inner velocity loop. ``Torque`` is closer to what a
    Lyapunov design would naturally output but requires either a torque
    constant calibration or accepting that ``Motor.dc`` is only a duty
    proxy.
    """

    Velocity = "velocity"
    Torque = "torque"
    DutyCycle = "duty"


@dataclass(frozen=True)
class ControlOutput:
    """One step of controller output.

    Attributes:
        leftCommand: Command for the left wheel motor in the units implied
            by ``mode``.
        rightCommand: Command for the right wheel motor in the units implied
            by ``mode``.
        mode: How the commands should be interpreted.
        timestamp: Time at which the controller produced this output.
    """

    leftCommand: float
    rightCommand: float
    mode: ControlMode
    timestamp: float = 0.0

    @staticmethod
    def Stop(mode: ControlMode = ControlMode.Velocity, timestamp: float = 0.0) -> ControlOutput:
        """Return a zero command in the given mode.

        Used by the safety monitor to halt the motors when a fault is
        detected.
        """
        return ControlOutput(leftCommand=0.0, rightCommand=0.0, mode=mode, timestamp=timestamp)


@dataclass(frozen=True)
class Measurement:
    """Raw measurement bundle handed to the estimator.

    All units are SI radians and SI radians per second. Conversions from
    Pybricks units happen in the adapter, not here.
    """

    tiltAngle: float
    tiltRate: float
    leftWheelAngle: float
    rightWheelAngle: float
    leftWheelRate: float
    rightWheelRate: float
    timestamp: float
    valid: bool = True
