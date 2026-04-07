"""Tests for LegoBalance.DriveCommandController."""

from __future__ import annotations

import pytest

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.DriveCommandController import DriveCommand, DriveCommandController
from LegoBalance.RobotConfig import LoadConfig


def MakeValidState(tilt: float = 0.0) -> BalanceState:
    return BalanceState(
        tilt=tilt,
        tiltRate=0.0,
        phi=0.0,
        phiDot=0.0,
        timestamp=0.0,
        valid=True,
    )


def test_DefaultsToStop():
    config = LoadConfig(applyLocalOverride=False)
    controller = DriveCommandController(config)
    assert controller.command is DriveCommand.Stop
    out = controller.Compute(MakeValidState())
    assert isinstance(out, ControlOutput)
    assert out.mode is ControlMode.Velocity
    assert out.leftCommand == 0.0
    assert out.rightCommand == 0.0


def test_ForwardProducesPositiveSymmetricVelocity():
    config = LoadConfig(applyLocalOverride=False)
    controller = DriveCommandController(config)
    controller.Forward()
    out = controller.Compute(MakeValidState())
    assert out.leftCommand == pytest.approx(config.drive.testSpeed)
    assert out.rightCommand == pytest.approx(config.drive.testSpeed)
    assert out.leftCommand > 0.0
    assert out.leftCommand == out.rightCommand


def test_BackwardProducesNegativeSymmetricVelocity():
    config = LoadConfig(applyLocalOverride=False)
    controller = DriveCommandController(config)
    controller.Backward()
    out = controller.Compute(MakeValidState())
    assert out.leftCommand == pytest.approx(-config.drive.testSpeed)
    assert out.rightCommand == pytest.approx(-config.drive.testSpeed)
    assert out.leftCommand < 0.0
    assert out.leftCommand == out.rightCommand


def test_StopProducesZeroVelocity():
    config = LoadConfig(applyLocalOverride=False)
    controller = DriveCommandController(config)
    controller.Forward()
    controller.Stop()
    out = controller.Compute(MakeValidState())
    assert out.leftCommand == 0.0
    assert out.rightCommand == 0.0


def test_InvalidStateAlwaysProducesStop():
    config = LoadConfig(applyLocalOverride=False)
    controller = DriveCommandController(config)
    controller.Forward()
    invalid = BalanceState(valid=False)
    out = controller.Compute(invalid)
    assert out.leftCommand == 0.0
    assert out.rightCommand == 0.0


def test_ResetGoesBackToStop():
    config = LoadConfig(applyLocalOverride=False)
    controller = DriveCommandController(config)
    controller.Forward()
    controller.Reset()
    assert controller.command is DriveCommand.Stop
    out = controller.Compute(MakeValidState())
    assert out.leftCommand == 0.0


def test_DriveSpeedIsClippedToMotorMax():
    config = LoadConfig(applyLocalOverride=False)
    config.drive.testSpeed = config.motors.maxAngularRate * 5.0
    controller = DriveCommandController(config)
    controller.Forward()
    out = controller.Compute(MakeValidState())
    assert out.leftCommand == pytest.approx(config.motors.maxAngularRate)


def test_NegativeDriveSpeedRejected():
    config = LoadConfig(applyLocalOverride=False)
    config.drive.testSpeed = -1.0
    with pytest.raises(ValueError):
        DriveCommandController(config)


def test_RejectNonDriveCommand():
    config = LoadConfig(applyLocalOverride=False)
    controller = DriveCommandController(config)
    with pytest.raises(TypeError):
        controller.SetCommand("forward")  # type: ignore[arg-type]
