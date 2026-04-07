"""Tests for LegoBalance.StateEstimator."""

from __future__ import annotations

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
    assert state.tilt == 0.1
    assert state.tiltRate == 0.2


def test_UpdateAveragesWheels():
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    state = estimator.Update(
        MakeMeasurement(leftAngle=2.0, rightAngle=4.0, leftRate=1.0, rightRate=3.0)
    )
    assert state.wheelPosition == 3.0
    assert state.wheelVelocity == 2.0


def test_ResetClearsHistory():
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    estimator.Update(MakeMeasurement(tilt=0.5))
    estimator.Reset()
    state = estimator.Update(MakeMeasurement(tilt=0.0))
    assert state.tilt == 0.0
