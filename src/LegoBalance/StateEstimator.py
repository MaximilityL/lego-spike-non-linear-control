"""StateEstimator skeleton.

The placeholder estimator applies the configured IMU and wheel encoder
signs/radius to derive the controller state.
A real implementation should run a complementary filter or a small Kalman
filter on the IMU and encoder data. The expected interface is documented
here so that the controller never needs to know which kind of estimator is
behind it.
"""

from __future__ import annotations

from .BalanceState import BalanceState
from .ControlInterfaces import Measurement
from .RobotConfig import RobotConfig
from .Units import WheelAngleToLinearDistance


class StateEstimator:
    """Pass through estimator with the right interface for future upgrades.

    Attributes:
        config: Reference to the robot configuration. The estimator does not
            mutate it.
        alpha: Complementary filter weight, taken from
            ``config.estimator.alpha``. Currently unused, kept here so the
            future implementation has a place to read it from.
    """

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.alpha = config.estimator.alpha
        self._lastTimestamp: float | None = None
        self._tiltEstimate: float = 0.0
        self._tiltRateEstimate: float = 0.0

    def Reset(self) -> None:
        """Drop any internal history. Call before each new run."""
        self._lastTimestamp = None
        self._tiltEstimate = 0.0
        self._tiltRateEstimate = 0.0

    def Update(self, measurement: Measurement) -> BalanceState:
        """Convert one raw measurement bundle into a :class:`BalanceState`.

        TODO: replace the corrected IMU pass through with a real
        complementary filter once the sign convention has been verified on
        hardware. The wheel encoder path applies the configured direction
        signs and wheel radius so the controller sees ``p`` and ``pDot``.

        Args:
            measurement: Raw measurement in SI units.

        Returns:
            A populated :class:`BalanceState`. ``valid`` is ``True`` once at
            least one measurement has been seen.
        """
        # Corrected IMU pass through. The hooks for a complementary filter
        # are named here so the future implementation can drop in.
        self._tiltEstimate = (
            self.config.imu.tiltSign * measurement.tiltAngle + self.config.imu.zeroOffset
        )
        self._tiltRateEstimate = (
            self.config.imu.tiltSign * measurement.tiltRate - self.config.imu.gyroBias
        )

        signedLeftAngle = self.config.motors.leftEncoderSign * measurement.leftWheelAngle
        signedRightAngle = self.config.motors.rightEncoderSign * measurement.rightWheelAngle
        signedLeftRate = self.config.motors.leftEncoderSign * measurement.leftWheelRate
        signedRightRate = self.config.motors.rightEncoderSign * measurement.rightWheelRate
        meanWheelAngle = 0.5 * (signedLeftAngle + signedRightAngle)
        meanWheelRate = 0.5 * (signedLeftRate + signedRightRate)
        forwardSign = self.config.motors.forwardSign
        wheelPosition = forwardSign * WheelAngleToLinearDistance(
            meanWheelAngle, self.config.chassis.wheelRadius
        )
        wheelVelocity = forwardSign * WheelAngleToLinearDistance(
            meanWheelRate, self.config.chassis.wheelRadius
        )

        self._lastTimestamp = measurement.timestamp
        return BalanceState(
            tilt=self._tiltEstimate,
            tiltRate=self._tiltRateEstimate,
            wheelPosition=wheelPosition,
            wheelVelocity=wheelVelocity,
            timestamp=measurement.timestamp,
            valid=True,
        )
