"""Tests for LegoBalance.PidController."""

from __future__ import annotations

import pytest

from LegoBalance.BalanceControllerFactory import BuildBalanceController
from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.PidController import PidController
from LegoBalance.RobotConfig import LoadConfig


def _BuildValidState(**overrides) -> BalanceState:
    defaults = {
        "tilt": 0.0,
        "tiltRate": 0.0,
        "phi": 0.0,
        "phiDot": 0.0,
        "timestamp": 1.23,
        "valid": True,
    }
    defaults.update(overrides)
    return BalanceState(**defaults)


def _DegPerSecToRadPerSec(value: float) -> float:
    return value * 3.141592653589793 / 180.0


def test_FactorySelectsPidControllerFromConfig():
    config = LoadConfig(applyLocalOverride=False)
    config.controller.algorithm = "pid"
    controller = BuildBalanceController(config)
    assert isinstance(controller, PidController)


def test_IsPlaceholderFalseForRealController():
    config = LoadConfig(applyLocalOverride=False)
    controller = PidController(config)
    assert controller.IsPlaceholder() is False


def test_ComputeReturnsStopForInvalidState():
    config = LoadConfig(applyLocalOverride=False)
    controller = PidController(config)
    invalid = BalanceState(tilt=1.0, timestamp=4.5, valid=False)
    output = controller.Compute(invalid)
    assert isinstance(output, ControlOutput)
    assert output.leftCommand == 0.0
    assert output.rightCommand == 0.0
    assert output.mode == ControlMode.Velocity
    assert output.timestamp == pytest.approx(4.5)


def test_ComputeReturnsSymmetricVelocityCommand():
    config = LoadConfig(applyLocalOverride=False)
    controller = PidController(config)
    state = _BuildValidState(tilt=0.08, phi=0.05, timestamp=2.0)
    output = controller.Compute(state)
    assert output.mode == ControlMode.Velocity
    assert output.leftCommand == pytest.approx(output.rightCommand)
    assert output.timestamp == pytest.approx(2.0)


def test_ComputeUsesConfiguredPidTerms():
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0
    config.control.maxWheelRate = 100.0
    config.controller.thetaDeadband = 0.0
    config.controller.pidKp = 2.0
    config.controller.pidKi = 3.0
    config.controller.pidKd = 5.0
    config.controller.pidKs = -7.0
    config.controller.pidIntegralStep = 0.25
    config.controller.pidIntegralLimit = 100.0
    config.controller.pidPositionTargetDeg = 0.0
    controller = PidController(config)

    state = _BuildValidState(tilt=0.1, phi=0.2)
    output = controller.Compute(state)

    tilt_deg = 0.1 * 180.0 / 3.141592653589793
    phi_deg = 0.2 * 180.0 / 3.141592653589793
    expected_deg_per_sec = (
        2.0 * tilt_deg
        + 3.0 * (tilt_deg * 0.25)
        + 5.0 * 0.0
        - 7.0 * phi_deg
    )
    assert output.leftCommand == pytest.approx(_DegPerSecToRadPerSec(expected_deg_per_sec))
    assert output.rightCommand == pytest.approx(_DegPerSecToRadPerSec(expected_deg_per_sec))


def test_ComputeUsesPreviousErrorForDerivativeTerm():
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0
    config.control.maxWheelRate = 100.0
    config.controller.thetaDeadband = 0.0
    config.controller.pidKp = 0.0
    config.controller.pidKi = 0.0
    config.controller.pidKd = 10.0
    config.controller.pidKs = 0.0
    controller = PidController(config)

    first = controller.Compute(_BuildValidState(tilt=0.05))
    second = controller.Compute(_BuildValidState(tilt=0.10))

    assert first.leftCommand == pytest.approx(0.0)
    error_step_deg = (0.10 - 0.05) * 180.0 / 3.141592653589793
    assert second.leftCommand == pytest.approx(_DegPerSecToRadPerSec(10.0 * error_step_deg))


def test_IntegralTermAccumulatesAcrossCalls():
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0
    config.control.maxWheelRate = 100.0
    config.controller.thetaDeadband = 0.0
    config.controller.pidKp = 0.0
    config.controller.pidKi = 2.0
    config.controller.pidKd = 0.0
    config.controller.pidKs = 0.0
    config.controller.pidIntegralStep = 0.25
    config.controller.pidIntegralLimit = 100.0
    controller = PidController(config)

    state = _BuildValidState(tilt=0.1)
    first = controller.Compute(state)
    second = controller.Compute(state)

    assert second.leftCommand > first.leftCommand > 0.0


def test_ResetClearsIntegralAndPreviousError():
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0
    config.control.maxWheelRate = 100.0
    config.controller.thetaDeadband = 0.0
    config.controller.pidKp = 0.0
    config.controller.pidKi = 1.0
    config.controller.pidKd = 9.0
    config.controller.pidKs = 0.0
    controller = PidController(config)

    state = _BuildValidState(tilt=0.1)
    controller.Compute(state)
    output_before_reset = controller.Compute(state)
    controller.Reset()
    output_after_reset = controller.Compute(state)

    assert output_before_reset.leftCommand > output_after_reset.leftCommand


def test_SaturationClampsAgainstConfiguredWheelRate():
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0
    config.control.maxWheelRate = 1.0
    config.controller.thetaDeadband = 0.0
    config.controller.pidKp = 500.0
    config.controller.pidKi = 0.0
    config.controller.pidKd = 0.0
    config.controller.pidKs = 0.0
    controller = PidController(config)

    output = controller.Compute(_BuildValidState(tilt=0.5))
    assert output.leftCommand == pytest.approx(1.0)
    assert output.rightCommand == pytest.approx(1.0)


def test_WheelPositionTermOpposesWalkOffWhenKsIsNegative():
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0
    config.control.maxWheelRate = 100.0
    config.controller.pidKp = 0.0
    config.controller.pidKi = 0.0
    config.controller.pidKd = 0.0
    config.controller.pidKs = -1.0
    config.controller.pidPositionTargetDeg = 0.0
    controller = PidController(config)

    output = controller.Compute(_BuildValidState(phi=0.1))
    assert output.leftCommand < 0.0
    assert output.rightCommand < 0.0


def test_ThetaDeadbandSuppressesSmallPOnlyTwitch():
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0
    config.controller.pidKp = 20.0
    config.controller.pidKi = 0.0
    config.controller.pidKd = 0.0
    config.controller.pidKs = 0.0
    config.controller.thetaDeadband = 0.02
    controller = PidController(config)

    inside = controller.Compute(_BuildValidState(tilt=0.01))
    outside = controller.Compute(_BuildValidState(tilt=0.03))

    assert inside.leftCommand == pytest.approx(0.0)
    assert inside.rightCommand == pytest.approx(0.0)
    assert outside.leftCommand > 0.0
    assert outside.rightCommand > 0.0
