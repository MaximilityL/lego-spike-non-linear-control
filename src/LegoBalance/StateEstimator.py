"""StateEstimator skeleton.

The placeholder estimator passes measurements through with minimal processing.
A real implementation should run a complementary filter or a small Kalman
filter on the IMU and encoder data. The expected interface is documented
here so that the controller never needs to know which kind of estimator is
behind it.
"""

from __future__ import annotations

from .BalanceState import BalanceState
from .ControlInterfaces import Measurement
from .RobotConfig import RobotConfig


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

        TODO: replace this body with a real complementary filter once the
        IMU sign convention has been verified on hardware. The current
        implementation is a pass through. It is shape correct so that the
        controller can be exercised in tests, but it makes no claim about
        accuracy.

        Args:
            measurement: Raw measurement in SI units.

        Returns:
            A populated :class:`BalanceState`. ``valid`` is ``True`` once at
            least one measurement has been seen.
        """
        # Trivial pass through. The hooks for a complementary filter are
        # named here so the future implementation can drop in.
        self._tiltEstimate = measurement.tiltAngle
        self._tiltRateEstimate = measurement.tiltRate

        meanWheelAngle = 0.5 * (measurement.leftWheelAngle + measurement.rightWheelAngle)
        meanWheelRate = 0.5 * (measurement.leftWheelRate + measurement.rightWheelRate)

        self._lastTimestamp = measurement.timestamp
        return BalanceState(
            tilt=self._tiltEstimate,
            tiltRate=self._tiltRateEstimate,
            wheelPosition=meanWheelAngle,
            wheelVelocity=meanWheelRate,
            timestamp=measurement.timestamp,
            valid=True,
        )
