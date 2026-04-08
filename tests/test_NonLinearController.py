"""Tests for LegoBalance.NonLinearController."""

from __future__ import annotations

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.LyapunovController import LyapunovController
from LegoBalance.NonLinearController import NonLinearController
from LegoBalance.RobotConfig import LoadConfig


def test_IsPlaceholderTrueByDefault():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    assert controller.IsPlaceholder() is True


def test_ComputeReturnsControlOutput():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = BalanceState(tilt=0.05, valid=True)
    output = controller.Compute(state)
    assert isinstance(output, ControlOutput)
    assert output.mode == ControlMode.Velocity


def test_ComputeReturnsZeroForInvalidState():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    invalid = BalanceState(tilt=1.0, valid=False)
    output = controller.Compute(invalid)
    assert output.leftCommand == 0.0
    assert output.rightCommand == 0.0


def test_PlaceholderBodyReturnsZeros():
    """Until the body is replaced, the placeholder must produce zeros.

    This guards against accidentally landing a half implemented controller
    that does something unsafe.
    """
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = BalanceState(tilt=0.1, tiltRate=0.5, phi=0.2, phiDot=1.0, valid=True)
    output = controller.Compute(state)
    assert output.leftCommand == 0.0
    assert output.rightCommand == 0.0


def test_ResetIsCallable():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    controller.Reset()  # should not raise


def test_LegacyLyapunovControllerAliasStillWorks():
    config = LoadConfig(applyLocalOverride=False)
    controller = LyapunovController(config)
    assert controller.IsPlaceholder() is True
