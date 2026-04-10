"""Tests for LegoBalance.StateEstimator."""

from __future__ import annotations

import pytest

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.RobotConfig import LoadConfig
from LegoBalance.StateEstimator import StateEstimator


def MakeMeasurement(
    tilt: float = 0.0,
    tiltRate: float = 0.0,
    leftAngle: float = 0.0,
    rightAngle: float = 0.0,
    leftRate: float = 0.0,
    rightRate: float = 0.0,
    timestamp: float = 0.0,
) -> Measurement:
    return Measurement(
        tiltAngle=tilt,
        tiltRate=tiltRate,
        leftWheelAngle=leftAngle,
        rightWheelAngle=rightAngle,
        leftWheelRate=leftRate,
        rightWheelRate=rightRate,
        timestamp=timestamp,
        valid=True,
    )


def test_UpdateReturnsBalanceStateWithRightShape():
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    state = estimator.Update(MakeMeasurement(tilt=0.1, tiltRate=0.2))
    assert isinstance(state, BalanceState)
    assert len(state.AsList()) == 4
    assert state.valid is True
    # Body tilt: sign correction then upright zero offset.
    assert state.tilt == pytest.approx(config.imu.tiltSign * 0.1 + config.imu.zeroOffset)
    # Body tilt rate: sign correction then gyro bias.
    assert state.tiltRate == pytest.approx(config.imu.tiltSign * 0.2 - config.imu.gyroBias)


def test_PhiAndPhiDotAreMeanOfSignedWheelSignals():
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    state = estimator.Update(
        MakeMeasurement(leftAngle=2.0, rightAngle=4.0, leftRate=1.0, rightRate=3.0)
    )
    expectedPhi = config.motors.forwardSign * 0.5 * (
        config.motors.leftEncoderSign * 2.0 + config.motors.rightEncoderSign * 4.0
    )
    expectedPhiDot = config.motors.forwardSign * 0.5 * (
        config.motors.leftEncoderSign * 1.0 + config.motors.rightEncoderSign * 3.0
    )
    assert state.phi == pytest.approx(expectedPhi)
    assert state.phiDot == pytest.approx(expectedPhiDot)


def test_PhiDoesNotDependOnWheelRadius():
    """The implemented phi state must not depend on wheel radius at all."""
    config = LoadConfig(applyLocalOverride=False)
    originalRadius = config.chassis.wheelRadius
    estimator = StateEstimator(config)
    stateA = estimator.Update(MakeMeasurement(leftAngle=1.0, rightAngle=-1.0))
    estimator.Reset()
    config.chassis.wheelRadius = originalRadius * 5.0
    stateB = estimator.Update(MakeMeasurement(leftAngle=1.0, rightAngle=-1.0))
    config.chassis.wheelRadius = originalRadius
    assert stateA.phi == pytest.approx(stateB.phi)
    assert stateA.phiDot == pytest.approx(stateB.phiDot)


def test_ForwardWheelRotationProducesPositivePhi():
    """Sign-corrected forward motion of both wheels should give phi > 0."""
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    # Feed raw hardware-frame signals that represent +1 rad of chassis
    # forward wheel rotation after the estimator applies encoder signs and
    # the final hardware-verified forwardSign.
    leftRaw = +1.0 * config.motors.leftEncoderSign * config.motors.forwardSign
    rightRaw = +1.0 * config.motors.rightEncoderSign * config.motors.forwardSign
    state = estimator.Update(
        MakeMeasurement(
            leftAngle=leftRaw,
            rightAngle=rightRaw,
            leftRate=leftRaw,
            rightRate=rightRaw,
        )
    )
    assert state.phi > 0.0
    assert state.phiDot > 0.0


def test_LinearHelpersOnEstimatorMatchWheelRadius():
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    state = BalanceState(phi=2.0, phiDot=3.0, valid=True)
    assert estimator.LinearPosition(state) == pytest.approx(2.0 * config.chassis.wheelRadius)
    assert estimator.LinearVelocity(state) == pytest.approx(3.0 * config.chassis.wheelRadius)


def test_UprightOffsetSubtractionBehaves():
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    # tilt = 0 raw should map to whatever the configured zero offset is.
    state = estimator.Update(MakeMeasurement(tilt=0.0))
    assert state.tilt == pytest.approx(config.imu.zeroOffset)


def test_GyroBiasSubtractionBehaves():
    config = LoadConfig(applyLocalOverride=False)
    config.imu = type(config.imu)(
        tiltAxis=config.imu.tiltAxis,
        tiltSign=config.imu.tiltSign,
        zeroOffset=config.imu.zeroOffset,
        gyroBias=0.05,
    )
    estimator = StateEstimator(config)
    # Raw tiltRate = 0 with a +0.05 rad/s bias should produce -0.05 once
    # subtracted (the sign correction is +1 here so it stays negative).
    state = estimator.Update(MakeMeasurement(tiltRate=0.0))
    assert state.tiltRate == pytest.approx(config.imu.tiltSign * 0.0 - 0.05)


def test_ResetClearsHistory():
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    estimator.Update(MakeMeasurement(tilt=0.5, leftAngle=1.0, rightAngle=1.0))
    estimator.Reset()
    state = estimator.Update(MakeMeasurement(tilt=0.0))
    assert state.tilt == pytest.approx(config.imu.zeroOffset)
    assert state.phi == pytest.approx(0.0)
    assert state.phiDot == pytest.approx(0.0)


def test_UpdatePropagatesMeasurementValidity():
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    measurement = MakeMeasurement(tilt=0.1, tiltRate=0.2, timestamp=0.5)
    measurement.valid = False
    state = estimator.Update(measurement)
    assert state.valid is False
