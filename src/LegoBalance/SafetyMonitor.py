"""SafetyMonitor.

A small but important module. The safety monitor sits between the controller
and the motors. Every command goes through it. If anything looks wrong it
substitutes a zero command and reports a fault.

The intent is that this module should be hard to bypass and easy to test.
Both are true.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .BalanceState import BalanceState
from .ControlInterfaces import ControlMode, ControlOutput
from .RobotConfig import RobotConfig
from .Saturation import SaturateSymmetric


@dataclass
class SafetyStatus:
    """Reports the most recent safety decision.

    Attributes:
        armed: Whether the safety monitor is currently allowing commands.
        tripped: Whether the most recent check tripped a fault.
        reasons: Human readable explanations of why the most recent check
            failed (empty if it passed).
        lastUpdateTime: Time of the most recent update for watchdog tracking.
    """

    armed: bool = False
    tripped: bool = False
    reasons: list[str] = field(default_factory=list)
    lastUpdateTime: float = 0.0


class SafetyMonitor:
    """Pre flight check, runtime guard, and watchdog all in one.

    The monitor is disarmed by default. The application code must call
    :meth:`Arm` after a successful pre flight check before any commands will
    be passed through to the motors.
    """

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self._status = SafetyStatus()
        self._maxTilt = config.control.maxTilt
        self._maxTiltRate = config.control.maxTiltRate
        self._maxWheelRate = config.control.maxWheelRate
        self._watchdogTimeout = config.control.watchdogTimeout

    @property
    def status(self) -> SafetyStatus:
        return self._status

    def Arm(self, currentTime: float = 0.0) -> None:
        """Arm the monitor and start the watchdog clock."""
        self._status.armed = True
        self._status.tripped = False
        self._status.reasons = []
        self._status.lastUpdateTime = currentTime

    def Disarm(self) -> None:
        """Disarm the monitor. Future commands will be replaced with stops."""
        self._status.armed = False

    def Trip(self, reason: str) -> None:
        """Mark the monitor as tripped with a given reason."""
        self._status.tripped = True
        self._status.armed = False
        self._status.reasons.append(reason)

    def Check(
        self,
        state: BalanceState,
        controlOutput: ControlOutput,
        currentTime: float | None = None,
    ) -> ControlOutput:
        """Validate one (state, control) pair.

        Returns the original ``controlOutput`` if everything is fine, or a
        zero command otherwise. Trip reasons accumulate on
        :attr:`status.reasons`.
        """
        if currentTime is None:
            currentTime = controlOutput.timestamp

        # Watchdog. If too much time has passed since the last update we
        # do not trust anything.
        if self._status.armed and self._watchdogTimeout > 0.0:
            elapsed = currentTime - self._status.lastUpdateTime
            if elapsed > self._watchdogTimeout:
                self.Trip(f"watchdog timeout: {elapsed:.3f}s > {self._watchdogTimeout:.3f}s")

        if not state.valid:
            self.Trip("state estimate not valid")
        if abs(state.tilt) > self._maxTilt:
            self.Trip(f"|tilt| {abs(state.tilt):.3f} exceeds max {self._maxTilt:.3f}")
        if abs(state.tiltRate) > self._maxTiltRate:
            self.Trip(
                f"|tiltRate| {abs(state.tiltRate):.3f} exceeds max {self._maxTiltRate:.3f}"
            )

        # Saturate the commands to the wheel velocity ceiling regardless of
        # whether anything tripped above. This is the lowest cost guard.
        leftClipped = SaturateSymmetric(controlOutput.leftCommand, self._maxWheelRate)
        rightClipped = SaturateSymmetric(controlOutput.rightCommand, self._maxWheelRate)

        if not self._status.armed or self._status.tripped:
            return ControlOutput.Stop(mode=controlOutput.mode, timestamp=currentTime)

        self._status.lastUpdateTime = currentTime
        return ControlOutput(
            leftCommand=leftClipped,
            rightCommand=rightClipped,
            mode=controlOutput.mode,
            timestamp=currentTime,
        )

    def StopCommand(self, mode: ControlMode = ControlMode.Velocity, timestamp: float = 0.0) -> ControlOutput:
        """Convenience: build a zero command without going through Check."""
        return ControlOutput.Stop(mode=mode, timestamp=timestamp)
