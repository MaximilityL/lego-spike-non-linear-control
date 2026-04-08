"""Tests for the hub-safe package drive smoke runtime."""

from __future__ import annotations

import pytest

import LegoBalance.HubDriveSmokeRuntime as hub_runtime
from LegoBalance.ControlInterfaces import Measurement as DesktopMeasurement
from LegoBalance.DriveCommandController import (
    DriveCommand as DesktopDriveCommand,
)
from LegoBalance.DriveCommandController import (
    DriveCommandController as DesktopDriveCommandController,
)
from LegoBalance.RobotConfig import LoadConfig
from LegoBalance.SafetyMonitor import SafetyMonitor as DesktopSafetyMonitor
from LegoBalance.StateEstimator import StateEstimator as DesktopStateEstimator


def test_HubRuntimeConfigMatchesDefaultConfig():
    desktop = LoadConfig(applyLocalOverride=False)
    hub = hub_runtime.DefaultConfig()

    assert hub.motors.leftPort == desktop.motors.leftPort
    assert hub.motors.rightPort == desktop.motors.rightPort
    assert hub.motors.forwardSign == desktop.motors.forwardSign
    assert hub.motors.leftEncoderSign == desktop.motors.leftEncoderSign
    assert hub.motors.rightEncoderSign == desktop.motors.rightEncoderSign
    assert hub.motors.maxAngularRate == pytest.approx(desktop.motors.maxAngularRate)
    assert hub.imu.tiltSign == desktop.imu.tiltSign
    assert hub.imu.zeroOffset == pytest.approx(desktop.imu.zeroOffset)
    assert hub.drive.testSpeed == pytest.approx(desktop.drive.testSpeed)
    assert hub.drive.maxTiltForMotion == pytest.approx(desktop.drive.maxTiltForMotion)


def test_HubRuntimeEstimatorMatchesDesktopEstimator():
    desktopConfig = LoadConfig(applyLocalOverride=False)
    hubConfig = hub_runtime.DefaultConfig()
    desktopEstimator = DesktopStateEstimator(desktopConfig)
    hubEstimator = hub_runtime.StateEstimator(hubConfig)

    values = dict(
        tiltAngle=0.12,
        tiltRate=-0.34,
        leftWheelAngle=1.2,
        rightWheelAngle=-0.8,
        leftWheelRate=3.4,
        rightWheelRate=-2.8,
        timestamp=1.23,
        valid=True,
    )
    desktopState = desktopEstimator.Update(DesktopMeasurement(**values))
    hubState = hubEstimator.Update(hub_runtime.Measurement(**values))

    assert hubState.AsList() == pytest.approx(desktopState.AsList())
    assert hubState.timestamp == pytest.approx(desktopState.timestamp)
    assert hubState.valid is desktopState.valid


def test_HubRuntimeControllerAndSafetyMatchDesktopFlow():
    desktopConfig = LoadConfig(applyLocalOverride=False)
    hubConfig = hub_runtime.DefaultConfig()
    desktopController = DesktopDriveCommandController(desktopConfig)
    hubController = hub_runtime.DriveCommandController(hubConfig)
    desktopSafety = DesktopSafetyMonitor(desktopConfig)
    hubSafety = hub_runtime.SafetyMonitor(hubConfig)

    desktopSafety.Arm(currentTime=0.0)
    hubSafety.Arm(currentTime=0.0)
    desktopController.SetCommand(DesktopDriveCommand.Forward)
    hubController.SetCommand(hub_runtime.DriveCommand.Forward)

    desktopState = DesktopStateEstimator(desktopConfig).Update(
        DesktopMeasurement(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01)
    )
    hubState = hub_runtime.StateEstimator(hubConfig).Update(
        hub_runtime.Measurement(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01)
    )

    desktopRaw = desktopController.Compute(desktopState)
    hubRaw = hubController.Compute(hubState)
    desktopSafe = desktopSafety.Check(desktopState, desktopRaw, currentTime=0.01)
    hubSafe = hubSafety.Check(hubState, hubRaw, currentTime=0.01)

    assert hubSafe.leftCommand == pytest.approx(desktopSafe.leftCommand)
    assert hubSafe.rightCommand == pytest.approx(desktopSafe.rightCommand)
