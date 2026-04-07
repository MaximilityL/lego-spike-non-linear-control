"""EstimatorReadout.py

Tiny desktop side, mock backed example that prints the implemented
``[theta, thetaDot, phi, phiDot]`` state at every step. The point is to
give a beginner a one screen view of what the estimator is producing
without having to wire up a controller or any motion.

Run with:

    python examples/EstimatorReadout.py
"""

from __future__ import annotations

from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.MockAdapters import MockHub
from LegoBalance.RobotConfig import LoadConfig
from LegoBalance.StateEstimator import StateEstimator

DT_SEC = 0.05
STEP_COUNT = 20


def Main() -> int:
    config = LoadConfig()
    hub = MockHub()
    estimator = StateEstimator(config)
    hub.SetInitialTilt(0.03)

    print("LegoBalance EstimatorReadout (mock)")
    print(
        "Primary implemented state: [theta (rad), thetaDot (rad/s), "
        "phi (rad), phiDot (rad/s)]"
    )
    print(
        f"{'t_s':>6}  {'theta':>8}  {'thetaDot':>9}  {'phi':>8}  {'phiDot':>9}  "
        f"{'p_m':>7}  {'pDot_mps':>9}"
    )

    timeSec = 0.0
    for _ in range(STEP_COUNT):
        rawLeftAngle = (
            config.motors.leftEncoderSign * config.motors.forwardSign * hub.LeftMotor.Angle()
        )
        rawRightAngle = (
            config.motors.rightEncoderSign * config.motors.forwardSign * hub.RightMotor.Angle()
        )
        rawLeftRate = (
            config.motors.leftEncoderSign * config.motors.forwardSign * hub.LeftMotor.Velocity()
        )
        rawRightRate = (
            config.motors.rightEncoderSign * config.motors.forwardSign * hub.RightMotor.Velocity()
        )
        measurement = Measurement(
            tiltAngle=config.imu.tiltSign
            * (hub.Imu.TiltAngleRadians() - config.imu.zeroOffset),
            tiltRate=config.imu.tiltSign
            * (hub.Imu.TiltRateRadiansPerSec() + config.imu.gyroBias),
            leftWheelAngle=rawLeftAngle,
            rightWheelAngle=rawRightAngle,
            leftWheelRate=rawLeftRate,
            rightWheelRate=rawRightRate,
            timestamp=timeSec,
            valid=True,
        )
        state = estimator.Update(measurement)
        # The translation pair is secondary. We print it here only for
        # physical intuition. Removing the wheel radius from config would
        # not affect the implemented state.
        pM = estimator.LinearPosition(state)
        pDotMps = estimator.LinearVelocity(state)
        print(
            f"{timeSec:6.2f}  {state.tilt:+8.3f}  {state.tiltRate:+9.3f}  "
            f"{state.phi:+8.3f}  {state.phiDot:+9.3f}  "
            f"{pM:+7.3f}  {pDotMps:+9.3f}"
        )
        hub.Step(DT_SEC)
        timeSec += DT_SEC

    hub.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
