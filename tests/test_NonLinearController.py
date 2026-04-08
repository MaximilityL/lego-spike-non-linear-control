"""Tests for LegoBalance.NonLinearController."""

from __future__ import annotations

import pytest

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.LyapunovController import LyapunovController
from LegoBalance.NonLinearController import NonLinearController
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


def test_IsPlaceholderFalseForRealController():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    assert controller.IsPlaceholder() is False


def test_ComputeReturnsStopForInvalidState():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    invalid = BalanceState(tilt=1.0, timestamp=4.5, valid=False)
    output = controller.Compute(invalid)
    assert isinstance(output, ControlOutput)
    assert output.leftCommand == 0.0
    assert output.rightCommand == 0.0
    assert output.mode == ControlMode.Velocity
    assert output.timestamp == pytest.approx(4.5)


def test_ComputeReturnsSymmetricVelocityCommand():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.08, tiltRate=0.25, phi=0.05, phiDot=0.2, timestamp=2.0)
    output = controller.Compute(state)
    assert isinstance(output, ControlOutput)
    assert output.mode == ControlMode.Velocity
    assert output.leftCommand == pytest.approx(output.rightCommand)
    assert output.timestamp == pytest.approx(2.0)


def test_ComputeSaturatesAgainstConfiguredWheelRate():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=5.0, tiltRate=15.0, phi=-3.0, phiDot=-4.0)
    output = controller.Compute(state)
    assert output.leftCommand == pytest.approx(config.control.maxWheelRate)
    assert output.rightCommand == pytest.approx(config.control.maxWheelRate)


def test_ResetClearsInternalBookkeeping():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.05, tiltRate=0.1, phi=0.02, phiDot=0.03, timestamp=3.0)
    controller.Compute(state)
    assert controller._lastTimestamp == pytest.approx(3.0)
    assert controller._lastSlidingVariable != 0.0
    assert controller._lastCommand != 0.0

    controller.Reset()

    assert controller._lastTimestamp == 0.0
    assert controller._lastSlidingVariable == 0.0
    assert controller._lastCommand == 0.0


def test_ForwardLeanCommandsForwardWheelMotion():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.1)
    output = controller.Compute(state)
    assert output.leftCommand > 0.0
    assert output.rightCommand > 0.0


def test_LegacyLyapunovControllerAliasStillWorks():
    config = LoadConfig(applyLocalOverride=False)
    controller = LyapunovController(config)
    assert controller.IsPlaceholder() is False
