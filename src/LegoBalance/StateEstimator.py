"""StateEstimator.

A deliberately minimal estimator for the post sign verification, pre balancing
phase. The built in LEGO SPIKE Prime sensors (IMU and motor encoders) are
already accurate enough for this stage, so this class does not run a
complementary filter or a Kalman filter. It only:

1. applies the configured sign conventions and calibration offsets that have
   already been verified on hardware (tilt sign, gyro sign, encoder signs,
   forward sign, zero offset, gyro bias),
2. averages the two wheel encoders into the mean wheel rotation angle ``phi``
   and the mean wheel rotation rate ``phiDot``,
3. produces a populated :class:`BalanceState`.

The implemented core state is

    [theta, thetaDot, phi, phiDot]

The translation pair ``p`` and ``pDot`` is *not* part of the implemented
state, on purpose. They can be derived from ``phi`` and ``phiDot`` if a
reliable wheel radius is available, via :meth:`LinearPosition` and
:meth:`LinearVelocity`. This keeps the estimator close to the raw measured
quantities and avoids any silent dependence on the exact wheel radius before
the balancing phase begins.

What this estimator does *not* do (and is not pretending to do):

* It does not fuse IMU and encoder signals together.
* It does not low pass filter anything.
* It does not estimate biases online.

Hooks for those upgrades are documented inline so a future complementary
filter or small Kalman filter can be dropped in without changing the public
interface.
"""

from __future__ import annotations

from .BalanceState import BalanceState
from .ControlInterfaces import Measurement


class StateEstimator:
    """Light, sign aware estimator that produces a :class:`BalanceState`.

    Attributes:
        config: Reference to the robot configuration. The estimator does not
            mutate it.
        alpha: Complementary filter weight, taken from
            ``config.estimator.alpha``. Currently unused at this stage, kept
            here so the future complementary filter has a place to read it
            from.
    """

    def __init__(self, config: object) -> None:
        self.config = config
        self.alpha = config.estimator.alpha
        self._lastTimestamp: float | None = None
        # Most recent estimates, kept so future stateful filters can extend
        # this class without rewriting the constructor.
        self._tiltEstimate: float = 0.0
        self._tiltRateEstimate: float = 0.0
        self._phiEstimate: float = 0.0
        self._phiDotEstimate: float = 0.0

    def Reset(self) -> None:
        """Drop any internal history. Call before each new run."""
        self._lastTimestamp = None
        self._tiltEstimate = 0.0
        self._tiltRateEstimate = 0.0
        self._phiEstimate = 0.0
        self._phiDotEstimate = 0.0

    def Update(self, measurement: Measurement) -> BalanceState:
        """Convert one raw measurement bundle into a :class:`BalanceState`.

        The mapping is intentionally simple:

        * tilt: apply the configured sign and add the calibrated upright
          zero offset to get the body tilt angle in radians, matching
          ``theta = tiltSign * rawTilt + zeroOffset`` on the hub.
        * tiltRate: apply the configured sign and subtract the gyro bias to
          get the body tilt rate in radians per second.
        * phi: apply each wheel's encoder sign, average the two, and apply
          the chassis forward sign so positive ``phi`` corresponds to the
          robot rolling forward.
        * phiDot: same treatment as ``phi`` but for the wheel rates.

        Args:
            measurement: Raw measurement in SI radians and radians per
                second. The adapter is responsible for converting Pybricks
                degrees and deg/s before they reach this method.

        Returns:
            A populated :class:`BalanceState`. ``valid`` is ``True`` once at
            least one measurement has been seen.
        """
        # ----- Body tilt: theta and thetaDot. -----
        # Sign correction first, then add the calibrated zero offset. The hooks for a
        # future complementary filter live here: blend self._tiltEstimate
        # with an integrated tiltRate using self.alpha when the time comes.
        self._tiltEstimate = (
            self.config.imu.tiltSign * measurement.tiltAngle + self.config.imu.zeroOffset
        )
        self._tiltRateEstimate = (
            self.config.imu.tiltSign * measurement.tiltRate - self.config.imu.gyroBias
        )

        # ----- Mean wheel rotation: phi and phiDot. -----
        # Sign each encoder so that "forward roll" maps to a positive
        # number, then average the two. The forwardSign is the final flip
        # used to keep increasing phi == rolling forward.
        signedLeftAngle = self.config.motors.leftEncoderSign * measurement.leftWheelAngle
        signedRightAngle = self.config.motors.rightEncoderSign * measurement.rightWheelAngle
        signedLeftRate = self.config.motors.leftEncoderSign * measurement.leftWheelRate
        signedRightRate = self.config.motors.rightEncoderSign * measurement.rightWheelRate
        meanWheelAngle = 0.5 * (signedLeftAngle + signedRightAngle)
        meanWheelRate = 0.5 * (signedLeftRate + signedRightRate)
        forwardSign = self.config.motors.forwardSign
        self._phiEstimate = forwardSign * meanWheelAngle
        self._phiDotEstimate = forwardSign * meanWheelRate

        self._lastTimestamp = measurement.timestamp
        return BalanceState(
            tilt=self._tiltEstimate,
            tiltRate=self._tiltRateEstimate,
            phi=self._phiEstimate,
            phiDot=self._phiDotEstimate,
            timestamp=measurement.timestamp,
            valid=True,
        )

    # ----- Optional translation helpers. -----
    #
    # These are convenience accessors that turn the implemented ``phi`` /
    # ``phiDot`` state into the translation pair ``p`` / ``pDot`` using the
    # wheel radius from the chassis config. They are intentionally separate
    # methods so that callers must opt in. The core estimator state never
    # silently depends on the wheel radius.

    def LinearPosition(self, state: BalanceState) -> float:
        """Return ``p = r * phi`` for the given state, in meters.

        Args:
            state: A :class:`BalanceState` produced by this estimator (or
                any other source that follows the convention).

        Returns:
            Forward linear position of the wheel base, in meters.
        """
        return state.LinearPosition(self.config.chassis.wheelRadius)

    def LinearVelocity(self, state: BalanceState) -> float:
        """Return ``pDot = r * phiDot`` for the given state, in m/s.

        Args:
            state: A :class:`BalanceState` produced by this estimator.

        Returns:
            Forward linear velocity of the wheel base, in meters per second.
        """
        return state.LinearVelocity(self.config.chassis.wheelRadius)
