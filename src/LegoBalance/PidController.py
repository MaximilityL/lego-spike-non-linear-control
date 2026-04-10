"""Discrete PID balancing controller.

This controller is intentionally model-light and follows the structure shown
in the referenced LEGO self-balancing example as closely as practical within
this repo's controller boundary:

    error       = tilt_deg - target_tilt_deg
    integral   += error * pidIntegralStep
    derivative  = error - prev_error
    result_dps  = kp*error + ki*integral + kd*derivative + ks*phi_deg

The key adaptation is sign convention. In this project:

* positive tilt means the body leans forward,
* positive command means forward wheel motion.

So the proportional error is defined as ``tilt - target`` rather than
``target - tilt`` to keep "forward lean -> forward correction" true.

The controller outputs wheel velocity in rad/s so it can plug into the same
package-backed hub path and post-run plots as the existing balance runs.
"""

from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.ControllerBase import ControllerBase
from LegoBalance.Saturation import SaturateSymmetric

PI = 3.141592653589793


def _RadToDeg(rad: float) -> float:
    return rad * 180.0 / PI


def _DegPerSecToRadPerSec(degPerSec: float) -> float:
    return degPerSec * PI / 180.0


class PidController(ControllerBase):
    """Discrete PID controller with optional wheel-position correction."""

    def __init__(self, config) -> None:
        ControllerBase.__init__(self, config)
        cc = config.controller
        self._kp = getattr(cc, "pidKp", 11.0)
        self._ki = getattr(cc, "pidKi", 4.2)
        self._kd = getattr(cc, "pidKd", 92.0)
        self._ks = getattr(cc, "pidKs", -0.6)
        self._integralStep = getattr(cc, "pidIntegralStep", 0.25)
        self._integralLimit = getattr(cc, "pidIntegralLimit", 50.0)
        self._positionTargetDeg = getattr(cc, "pidPositionTargetDeg", 0.0)
        self._targetTiltDeg = _RadToDeg(config.control.targetTilt)
        self._thetaDeadbandDeg = _RadToDeg(getattr(cc, "thetaDeadband", 0.0))
        self._maxWheelRate = config.control.maxWheelRate
        self._integral = 0.0
        self._prevErrorDeg = 0.0
        self._hasPrevError = False

    def IsPlaceholder(self) -> bool:
        return False

    def _IsFinite(self, value: float) -> bool:
        return value == value and value not in (float("inf"), float("-inf"))

    def _ApplyDeadband(self, value: float, width: float) -> float:
        if width <= 0.0:
            return value
        if value > width:
            return value - width
        if value < -width:
            return value + width
        return 0.0

    def _ComputeVelocityCommand(self, theta: float, phi: float) -> float:
        tiltDeg = _RadToDeg(theta)
        phiDeg = _RadToDeg(phi)

        # Positive forward lean must create a positive forward wheel command.
        errorDeg = self._ApplyDeadband(
            tiltDeg - self._targetTiltDeg,
            self._thetaDeadbandDeg,
        )
        self._integral += errorDeg * self._integralStep
        if self._integralLimit > 0.0:
            self._integral = SaturateSymmetric(self._integral, self._integralLimit)

        if self._hasPrevError:
            derivativeDeg = errorDeg - self._prevErrorDeg
        else:
            derivativeDeg = 0.0
            self._hasPrevError = True
        self._prevErrorDeg = errorDeg

        # Positive phi means the base has rolled forward. With ks < 0 this
        # term resists walk-off in the same spirit as the referenced example's
        # start-target correction.
        positionErrorDeg = phiDeg - self._positionTargetDeg

        commandDegPerSec = (
            self._kp * errorDeg
            + self._ki * self._integral
            + self._kd * derivativeDeg
            + self._ks * positionErrorDeg
        )
        commandRadPerSec = _DegPerSecToRadPerSec(commandDegPerSec)
        return SaturateSymmetric(commandRadPerSec, self._maxWheelRate)

    def Compute(self, state):
        if not state.valid:
            return ControlOutput.Stop(mode=ControlMode.Velocity, timestamp=state.timestamp)

        theta = state.theta
        phi = state.phi
        if not self._IsFinite(theta) or not self._IsFinite(phi):
            return ControlOutput.Stop(mode=ControlMode.Velocity, timestamp=state.timestamp)

        command = self._ComputeVelocityCommand(theta, phi)
        return ControlOutput(
            leftCommand=command,
            rightCommand=command,
            mode=ControlMode.Velocity,
            timestamp=state.timestamp,
        )

    def Reset(self) -> None:
        self._integral = 0.0
        self._prevErrorDeg = 0.0
        self._hasPrevError = False
