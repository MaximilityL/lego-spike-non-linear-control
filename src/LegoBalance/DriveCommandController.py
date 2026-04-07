"""DriveCommandController.

A small, intentionally non balancing controller used in the post sign
verification, pre balancing phase. It exists so that the project has a
controller it can plug into the normal estimator + safety + motor pipeline
without pretending to balance anything.

What it does:

* Accepts a high level :class:`DriveCommand` such as ``Forward``, ``Backward``
  or ``Stop``.
* Maps that command to a symmetric left/right wheel velocity using the
  configured drive test speed.
* Returns a :class:`ControlOutput` in the same shape that the future
  :class:`LyapunovController` will return, so the rest of the pipeline does
  not care which controller is in front.

What it does not do:

* It does not look at ``state.tilt`` or any other state field to decide what
  to send. The :class:`SafetyMonitor` is still the gatekeeper that refuses
  motion when the body tilt is unsafe. Keeping the drive controller stateless
  with respect to the body tilt makes its behavior obvious and easy to test.
* It does not balance, stabilize, or otherwise close a loop on the body. It
  is a pre balancing motion command path.

The controller still consumes the :class:`BalanceState` produced by the
estimator (because :class:`ControllerBase.Compute` takes one), so the loop
that uses it is exactly the loop that the future balancing controller will
plug into. Swapping :class:`DriveCommandController` for
:class:`LyapunovController` is a one line change in application code.
"""

from __future__ import annotations

from enum import Enum

from .BalanceState import BalanceState
from .ControlInterfaces import ControlMode, ControlOutput
from .ControllerBase import ControllerBase
from .RobotConfig import RobotConfig


class DriveCommand(str, Enum):
    """High level drive intent for the pre balancing motion smoke tests.

    These are intentionally coarse. They map to a fixed magnitude wheel
    velocity command on both wheels, with the sign chosen so that
    ``Forward`` rolls the wheel base forward (i.e. produces increasing
    ``phi``).
    """

    Stop = "stop"
    Forward = "forward"
    Backward = "backward"


class DriveCommandController(ControllerBase):
    """Open loop drive command path for the pre balancing phase.

    Attributes:
        config: Reference to the :class:`RobotConfig`. Read only.
    """

    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)
        self._command: DriveCommand = DriveCommand.Stop
        self._driveSpeed = float(config.drive.testSpeed)
        # The wheel velocity ceiling is enforced again by the safety monitor.
        # We also clip here so that nothing downstream of the controller has
        # to worry about an obviously unsafe magnitude leaking through.
        if self._driveSpeed < 0.0:
            raise ValueError("drive.testSpeed must be non negative")
        if self._driveSpeed > config.motors.maxAngularRate:
            self._driveSpeed = float(config.motors.maxAngularRate)

    # ----- Public command surface. -----
    def SetCommand(self, command: DriveCommand) -> None:
        """Set the next drive command. Takes effect on the next :meth:`Compute`."""
        if not isinstance(command, DriveCommand):
            raise TypeError(f"command must be a DriveCommand, got {type(command).__name__}")
        self._command = command

    def Forward(self) -> None:
        """Convenience: set the command to :attr:`DriveCommand.Forward`."""
        self.SetCommand(DriveCommand.Forward)

    def Backward(self) -> None:
        """Convenience: set the command to :attr:`DriveCommand.Backward`."""
        self.SetCommand(DriveCommand.Backward)

    def Stop(self) -> None:
        """Convenience: set the command to :attr:`DriveCommand.Stop`."""
        self.SetCommand(DriveCommand.Stop)

    @property
    def command(self) -> DriveCommand:
        """Currently active drive command."""
        return self._command

    @property
    def driveSpeed(self) -> float:
        """Magnitude of the wheel velocity used for forward/backward, in rad/s."""
        return self._driveSpeed

    # ----- Controller surface. -----
    def Compute(self, state: BalanceState) -> ControlOutput:
        """Map the current state and active command to a wheel velocity output.

        The current state is accepted (and is required by the
        :class:`ControllerBase` interface) so that this controller plugs
        into the same loop the future balancing controller will. It is not
        used to alter the command magnitude. The :class:`SafetyMonitor` is
        responsible for refusing motion when the tilt is unsafe.

        Args:
            state: Latest state estimate from :class:`StateEstimator`.

        Returns:
            A :class:`ControlOutput` in :class:`ControlMode.Velocity` with
            symmetric left and right commands.
        """
        # Refuse to act on a state that the estimator has not validated yet.
        if not state.valid:
            return ControlOutput.Stop(mode=ControlMode.Velocity, timestamp=state.timestamp)

        if self._command is DriveCommand.Stop:
            magnitude = 0.0
        elif self._command is DriveCommand.Forward:
            magnitude = +self._driveSpeed
        elif self._command is DriveCommand.Backward:
            magnitude = -self._driveSpeed
        else:  # pragma: no cover - DriveCommand is closed
            magnitude = 0.0

        return ControlOutput(
            leftCommand=magnitude,
            rightCommand=magnitude,
            mode=ControlMode.Velocity,
            timestamp=state.timestamp,
        )

    def Reset(self) -> None:
        """Reset the active command back to :attr:`DriveCommand.Stop`."""
        self._command = DriveCommand.Stop
