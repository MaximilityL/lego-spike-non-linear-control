"""Tests for the hub-safe package drive smoke defaults."""

from __future__ import annotations

import pytest

import LegoBalance.HubDriveSmokeRuntime as hub_runtime
from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.DriveCommandController import DriveCommand, DriveCommandController
from LegoBalance.RobotConfig import LoadConfig
from LegoBalance.SafetyMonitor import SafetyMonitor
from LegoBalance.StateEstimator import StateEstimator
from scripts.GenerateHubDriveSmokeRuntime import (
    RenderHubDriveSmokeRuntime,
)


def test_HubDefaultConfigModuleIsGeneratedFromDefaultYaml():
    from LegoBalance.RobotConfig import DEFAULT_CONFIG_PATH

    desktop = LoadConfig(path=DEFAULT_CONFIG_PATH, applyLocalOverride=False)
    with open(hub_runtime.__file__, encoding="utf-8") as fh:
        actual = fh.read()
    assert actual == RenderHubDriveSmokeRuntime(desktop, DEFAULT_CONFIG_PATH)


def test_HubDefaultConfigMatchesDesktopDefaultConfig():
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
    assert hub.control.targetTilt == pytest.approx(desktop.control.targetTilt)
    assert hub.controller.lambdaTheta == pytest.approx(desktop.controller.lambdaTheta)
    assert hub.controller.lambdaPhiDot == pytest.approx(desktop.controller.lambdaPhiDot)
    assert hub.controller.lambdaPhi == pytest.approx(desktop.controller.lambdaPhi)
    assert hub.controller.kTheta == pytest.approx(desktop.controller.kTheta)
    assert hub.controller.kThetaDot == pytest.approx(desktop.controller.kThetaDot)
    assert hub.controller.kPhi == pytest.approx(desktop.controller.kPhi)
    assert hub.controller.kPhiDot == pytest.approx(desktop.controller.kPhiDot)
    assert hub.controller.kSigma == pytest.approx(desktop.controller.kSigma)
    assert hub.controller.boundaryLayerWidth == pytest.approx(
        desktop.controller.boundaryLayerWidth
    )
    assert hub.drive.loopPeriodMs == desktop.drive.loopPeriodMs
    assert hub.drive.printEveryN == desktop.drive.printEveryN
    assert hub.drive.stopDurationMs == desktop.drive.stopDurationMs
    assert hub.drive.driveDurationMs == desktop.drive.driveDurationMs
    assert hub.drive.testSpeed == pytest.approx(desktop.drive.testSpeed)
    assert hub.drive.maxTiltForMotion == pytest.approx(desktop.drive.maxTiltForMotion)


def test_SharedEstimatorWorksWithHubDefaultConfig():
    desktopConfig = LoadConfig(applyLocalOverride=False)
    hubConfig = hub_runtime.DefaultConfig()
    desktopEstimator = StateEstimator(desktopConfig)
    hubEstimator = StateEstimator(hubConfig)

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
    desktopState = desktopEstimator.Update(Measurement(**values))
    hubState = hubEstimator.Update(Measurement(**values))

    assert hubState.AsList() == pytest.approx(desktopState.AsList())
    assert hubState.timestamp == pytest.approx(desktopState.timestamp)
    assert hubState.valid is desktopState.valid


def test_SharedControllerAndSafetyWorkWithHubDefaultConfig():
    desktopConfig = LoadConfig(applyLocalOverride=False)
    hubConfig = hub_runtime.DefaultConfig()
    desktopController = DriveCommandController(desktopConfig)
    hubController = DriveCommandController(hubConfig)
    desktopSafety = SafetyMonitor(desktopConfig)
    hubSafety = SafetyMonitor(hubConfig)

    desktopSafety.Arm(currentTime=0.0)
    hubSafety.Arm(currentTime=0.0)
    desktopController.SetCommand(DriveCommand.Forward)
    hubController.SetCommand(DriveCommand.Forward)

    desktopState = StateEstimator(desktopConfig).Update(
        Measurement(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01)
    )
    hubState = StateEstimator(hubConfig).Update(
        Measurement(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01)
    )

    desktopRaw = desktopController.Compute(desktopState)
    hubRaw = hubController.Compute(hubState)
    desktopSafe = desktopSafety.Check(desktopState, desktopRaw, currentTime=0.01)
    hubSafe = hubSafety.Check(hubState, hubRaw, currentTime=0.01)

    assert hubSafe.leftCommand == pytest.approx(desktopSafe.leftCommand)
    assert hubSafe.rightCommand == pytest.approx(desktopSafe.rightCommand)
