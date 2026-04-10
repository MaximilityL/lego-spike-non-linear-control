"""SafetyMonitor.

A small but important module. The safety monitor sits between the controller
and the motors. Every command goes through it. If anything looks wrong it
substitutes a zero command and reports a fault.

The intent is that this module should be hard to bypass and easy to test.
Both are true.
"""

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.Saturation import SaturateSymmetric


class SafetyStatus:
    """Reports the most recent safety decision.

    Attributes:
        armed: Whether the safety monitor is currently allowing commands.
        tripped: Whether the most recent check tripped a fault.
        reasons: Human readable explanations of why the most recent check
            failed (empty if it passed).
        lastUpdateTime: Time of the most recent update for watchdog tracking.
    """

    def __init__(
        self,
        armed: bool = False,
        tripped: bool = False,
        reasons=None,
        lastUpdateTime: float = 0.0,
    ) -> None:
        self.armed = armed
        self.tripped = tripped
        self.reasons = [] if reasons is None else list(reasons)
        self.lastUpdateTime = lastUpdateTime


class SafetyMonitor:
    """Pre flight check, runtime guard, and watchdog all in one.

    The monitor is disarmed by default. The application code must call
    :meth:`Arm` after a successful pre flight check before any commands will
    be passed through to the motors.
    """

    def __init__(self, config: object) -> None:
        self.config = config
        self.status = SafetyStatus()
        self._maxTilt = config.control.maxTilt
        self._maxTiltRate = config.control.maxTiltRate
        self._maxWheelRate = config.control.maxWheelRate
        self._watchdogTimeout = config.control.watchdogTimeout
        # Tighter cap used by the pre balancing drive command path. The
        # drive controller is not allowed to push the wheels around once
        # the body tilt has wandered past this. ``maxTilt`` is still the
        # absolute upper bound that trips the monitor for any controller.
        self._maxTiltForMotion = config.drive.maxTiltForMotion

    def Arm(self, currentTime: float = 0.0) -> None:
        """Arm the monitor and start the watchdog clock."""
        self.status.armed = True
        self.status.tripped = False
        self.status.reasons = []
        self.status.lastUpdateTime = currentTime

    def Disarm(self) -> None:
        """Disarm the monitor. Future commands will be replaced with stops."""
        self.status.armed = False

    def Trip(self, reason: str) -> None:
        """Mark the monitor as tripped with a given reason."""
        self.status.tripped = True
        self.status.armed = False
        self.status.reasons.append(reason)

    def _IsFinite(self, value: float) -> bool:
        """Return ``True`` when the scalar is neither NaN nor infinite."""
        return value == value and value not in (float("inf"), float("-inf"))

    def Check(
        self,
        state: BalanceState,
        controlOutput: ControlOutput,
        currentTime=None,
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
        if self.status.armed and self._watchdogTimeout > 0.0:
            elapsed = currentTime - self.status.lastUpdateTime
            if elapsed > self._watchdogTimeout:
                self.Trip(f"watchdog timeout: {elapsed:.3f}s > {self._watchdogTimeout:.3f}s")

        if not state.valid:
            self.Trip("state estimate not valid")
        if not self._IsFinite(state.tilt):
            self.Trip("tilt is not finite")
        if not self._IsFinite(state.tiltRate):
            self.Trip("tiltRate is not finite")
        if not self._IsFinite(controlOutput.leftCommand):
            self.Trip("left command is not finite")
        if not self._IsFinite(controlOutput.rightCommand):
            self.Trip("right command is not finite")
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

        if not self.status.armed or self.status.tripped:
            return ControlOutput.Stop(mode=controlOutput.mode, timestamp=currentTime)

        self.status.lastUpdateTime = currentTime
        return ControlOutput(
            leftCommand=leftClipped,
            rightCommand=rightClipped,
            mode=controlOutput.mode,
            timestamp=currentTime,
        )

    def StopCommand(self, mode: ControlMode = ControlMode.Velocity, timestamp: float = 0.0) -> ControlOutput:
        """Convenience: build a zero command without going through Check."""
        return ControlOutput.Stop(mode=mode, timestamp=timestamp)

    def IsTiltSafeForDriveMotion(self, state: BalanceState) -> bool:
        """Return ``True`` when the body tilt is small enough to allow drive motion.

        Used by the pre balancing drive command path. The drive controller
        is honest about not balancing anything, so we want a tighter limit
        than the absolute :attr:`config.control.maxTilt` ceiling before we
        let it command motion. Returning ``False`` here is meant as a soft
        gate: callers should substitute a stop command rather than tripping
        the safety monitor outright.
        """
        if not state.valid:
            return False
        return abs(state.tilt) <= self._maxTiltForMotion
