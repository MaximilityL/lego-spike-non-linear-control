"""Tests for LegoBalance.NonLinearController."""

from __future__ import annotations

import math

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


def _AlphaEstimate(config) -> float:
    m = config.chassis.bodyMass
    l = config.chassis.bodyHeightCoM
    I = config.chassis.bodyInertia
    return m * 9.81 * l / (I + m * l * l)


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


def test_ComputeUsesConfiguredControllerGains():
    config = LoadConfig(applyLocalOverride=False)
    config.control.maxWheelRate = 100.0
    config.controller.gravityCompGain = 1.0
    config.controller.kTheta = 2.0
    config.controller.kThetaDot = 3.0
    config.controller.kPhi = 4.0
    config.controller.kPhiDot = 5.0
    config.controller.thetaDeadband = 0.0
    config.controller.thetaDotDeadband = 0.0
    controller = NonLinearController(config)

    state = _BuildValidState(tilt=0.3, tiltRate=0.2, phi=0.1, phiDot=0.05)
    output = controller.Compute(state)
    expected = (
        _AlphaEstimate(config) * math.sin(state.tilt)
        + 2.0 * state.tilt
        + 3.0 * state.tiltRate
        - 4.0 * state.phi
        - 5.0 * state.phiDot
    )

    assert output.leftCommand == pytest.approx(expected)
    assert output.rightCommand == pytest.approx(expected)


def test_ComputeAppliesConfiguredQuietDeadbands():
    config = LoadConfig(applyLocalOverride=False)
    config.controller.gravityCompGain = 0.0
    config.controller.kTheta = 10.0
    config.controller.kThetaDot = 10.0
    config.controller.kPhi = 0.0
    config.controller.kPhiDot = 0.0
    config.controller.thetaDeadband = 0.1
    config.controller.thetaDotDeadband = 0.2
    controller = NonLinearController(config)

    quiet = controller.Compute(_BuildValidState(tilt=0.08, tiltRate=-0.15))
    active = controller.Compute(_BuildValidState(tilt=0.3, tiltRate=-0.5))

    assert quiet.leftCommand == pytest.approx(0.0)
    assert quiet.rightCommand == pytest.approx(0.0)
    assert active.leftCommand == pytest.approx(-1.0)
    assert active.rightCommand == pytest.approx(-1.0)


def test_ComputeUsesRawThetaForGravityCompensationEvenInsideDeadband():
    config = LoadConfig(applyLocalOverride=False)
    config.control.maxWheelRate = 100.0
    config.controller.gravityCompGain = 1.0
    config.controller.kTheta = 0.0
    config.controller.kThetaDot = 0.0
    config.controller.kPhi = 0.0
    config.controller.kPhiDot = 0.0
    config.controller.thetaDeadband = 1.0
    config.controller.thetaDotDeadband = 1.0
    controller = NonLinearController(config)

    output = controller.Compute(_BuildValidState(tilt=0.05))
    expected = _AlphaEstimate(config) * math.sin(0.05)

    assert output.leftCommand == pytest.approx(expected)
    assert output.rightCommand == pytest.approx(expected)


def test_ResetKeepsMemorylessControllerDeterministic():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.05, tiltRate=0.1, phi=0.02, phiDot=0.03, timestamp=3.0)
    beforeReset = controller.Compute(state)

    controller.Reset()
    afterReset = controller.Compute(state)

    assert afterReset.leftCommand == pytest.approx(beforeReset.leftCommand)
    assert afterReset.rightCommand == pytest.approx(beforeReset.rightCommand)


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
