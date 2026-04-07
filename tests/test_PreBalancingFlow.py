"""Repo level flow tests for the post sign verification, pre balancing phase.

These tests wire up the same components a real desktop run uses (mock hub
plus the real package abstractions) and validate the four properties that
matter for this stage of the project:

* Measurements flow through the estimator into a valid :class:`BalanceState`
  using the implemented ``[theta, thetaDot, phi, phiDot]`` convention.
* :class:`DriveCommandController` produces the right symmetric command for
  the active high level intent.
* The :class:`SafetyMonitor` clamps and disarms commands when the body
  tilt or tilt rate exceeds the configured limits.
* The drive specific tilt gate (``IsTiltSafeForDriveMotion``) refuses
  motion before the absolute safety ceiling trips.
"""

from __future__ import annotations

import pytest

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import ControlMode, ControlOutput, Measurement
from LegoBalance.DriveCommandController import DriveCommand, DriveCommandController
from LegoBalance.MockAdapters import MockHub
from LegoBalance.RobotConfig import LoadConfig, RobotConfig
from LegoBalance.SafetyMonitor import SafetyMonitor
from LegoBalance.StateEstimator import StateEstimator


def BuildMeasurementFromMockHub(
    hub: MockHub, config: RobotConfig, currentTimeSec: float
) -> Measurement:
    """Same conversion the simulation example uses, kept local for clarity.

    The mock hub stores its tilt and wheel motion in physical units. This
    helper undoes the calibration so the estimator sees the same kind of
    raw measurement it would receive from the hub side adapter.
    """
    rawLeftAngle = config.motors.leftEncoderSign * config.motors.forwardSign * hub.LeftMotor.Angle()
    rawRightAngle = (
        config.motors.rightEncoderSign * config.motors.forwardSign * hub.RightMotor.Angle()
    )
    rawLeftRate = config.motors.leftEncoderSign * config.motors.forwardSign * hub.LeftMotor.Velocity()
    rawRightRate = (
        config.motors.rightEncoderSign * config.motors.forwardSign * hub.RightMotor.Velocity()
    )
    return Measurement(
        tiltAngle=config.imu.tiltSign * (hub.Imu.TiltAngleRadians() - config.imu.zeroOffset),
        tiltRate=config.imu.tiltSign
        * (hub.Imu.TiltRateRadiansPerSec() + config.imu.gyroBias),
        leftWheelAngle=rawLeftAngle,
        rightWheelAngle=rawRightAngle,
        leftWheelRate=rawLeftRate,
        rightWheelRate=rawRightRate,
        timestamp=currentTimeSec,
        valid=True,
    )


def RunDriveStep(
    hub: MockHub,
    config: RobotConfig,
    estimator: StateEstimator,
    controller: DriveCommandController,
    safety: SafetyMonitor,
    timeSec: float,
) -> tuple[BalanceState, ControlOutput]:
    """One desktop side iteration of the drive command path."""
    measurement = BuildMeasurementFromMockHub(hub, config, timeSec)
    state = estimator.Update(measurement)
    if not safety.IsTiltSafeForDriveMotion(state):
        controller.Stop()
    raw = controller.Compute(state)
    safe = safety.Check(state, raw, currentTime=timeSec)
    return state, safe


def test_MeasurementToEstimatorProducesPhiPrimaryState():
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    measurement = BuildMeasurementFromMockHub(MockHub(), config, 0.0)
    state = estimator.Update(measurement)
    assert state.valid is True
    # The four implemented state entries are theta, thetaDot, phi, phiDot.
    assert state.AsList() == [state.tilt, state.tiltRate, state.phi, state.phiDot]


def test_ForwardCommandFlowsThroughPipelineAndProducesPositiveOutput():
    config = LoadConfig(applyLocalOverride=False)
    hub = MockHub()
    estimator = StateEstimator(config)
    controller = DriveCommandController(config)
    safety = SafetyMonitor(config)

    safety.Arm(currentTime=0.0)
    controller.Forward()

    state, safeCommand = RunDriveStep(hub, config, estimator, controller, safety, 0.0)
    assert state.valid is True
    assert safeCommand.mode is ControlMode.Velocity
    assert safeCommand.leftCommand == pytest.approx(config.drive.testSpeed)
    assert safeCommand.rightCommand == pytest.approx(config.drive.testSpeed)


def test_BackwardCommandFlowsThroughPipelineAndProducesNegativeOutput():
    config = LoadConfig(applyLocalOverride=False)
    hub = MockHub()
    estimator = StateEstimator(config)
    controller = DriveCommandController(config)
    safety = SafetyMonitor(config)

    safety.Arm(currentTime=0.0)
    controller.Backward()

    _, safeCommand = RunDriveStep(hub, config, estimator, controller, safety, 0.0)
    assert safeCommand.leftCommand == pytest.approx(-config.drive.testSpeed)
    assert safeCommand.rightCommand == pytest.approx(-config.drive.testSpeed)


def test_StopCommandFlowsThroughPipelineAndProducesZeroOutput():
    config = LoadConfig(applyLocalOverride=False)
    hub = MockHub()
    estimator = StateEstimator(config)
    controller = DriveCommandController(config)
    safety = SafetyMonitor(config)

    safety.Arm(currentTime=0.0)
    controller.Stop()

    _, safeCommand = RunDriveStep(hub, config, estimator, controller, safety, 0.0)
    assert safeCommand.leftCommand == 0.0
    assert safeCommand.rightCommand == 0.0


def test_TiltAboveDriveLimitGatesMotion():
    config = LoadConfig(applyLocalOverride=False)
    controller = DriveCommandController(config)
    safety = SafetyMonitor(config)
    safety.Arm(currentTime=0.0)
    controller.Forward()

    # Hand crafted state with tilt above the drive gate but below the
    # absolute safety ceiling. The drive controller should be stopped by
    # the gate, not by the safety monitor.
    tiltJustAboveDriveGate = config.drive.maxTiltForMotion + 0.05
    assert tiltJustAboveDriveGate < config.control.maxTilt
    state = BalanceState(
        tilt=tiltJustAboveDriveGate, tiltRate=0.0, phi=0.0, phiDot=0.0, valid=True, timestamp=0.0
    )
    assert safety.IsTiltSafeForDriveMotion(state) is False

    # Apply the gate the same way RunDriveStep does.
    if not safety.IsTiltSafeForDriveMotion(state):
        controller.Stop()
    raw = controller.Compute(state)
    safe = safety.Check(state, raw, currentTime=0.0)
    assert safe.leftCommand == 0.0
    assert safe.rightCommand == 0.0
    # The safety monitor itself should not have tripped on this case.
    assert safety.status.tripped is False


def test_TiltAboveAbsoluteCeilingDisarmsSafetyMonitor():
    config = LoadConfig(applyLocalOverride=False)
    controller = DriveCommandController(config)
    safety = SafetyMonitor(config)
    safety.Arm(currentTime=0.0)
    controller.Forward()

    huge = BalanceState(
        tilt=config.control.maxTilt + 0.1,
        tiltRate=0.0,
        phi=0.0,
        phiDot=0.0,
        valid=True,
        timestamp=0.0,
    )
    raw = controller.Compute(huge)
    safe = safety.Check(huge, raw, currentTime=0.0)
    assert safe.leftCommand == 0.0
    assert safety.status.tripped is True


def test_PreBalancingLoopRunsWithoutErrors():
    """A short loop using the same components a smoke flow uses."""
    config = LoadConfig(applyLocalOverride=False)
    hub = MockHub()
    estimator = StateEstimator(config)
    controller = DriveCommandController(config)
    safety = SafetyMonitor(config)
    safety.Arm(currentTime=0.0)

    timeSec = 0.0
    dt = 0.01
    sequence = (
        [DriveCommand.Forward] * 10
        + [DriveCommand.Stop] * 5
        + [DriveCommand.Backward] * 10
        + [DriveCommand.Stop] * 5
    )
    seenAtLeastOneNonZero = False
    for cmd in sequence:
        controller.SetCommand(cmd)
        state, safeCommand = RunDriveStep(
            hub, config, estimator, controller, safety, timeSec
        )
        if abs(safeCommand.leftCommand) > 0.0:
            seenAtLeastOneNonZero = True
        hub.LeftMotor.RunVelocity(safeCommand.leftCommand)
        hub.RightMotor.RunVelocity(safeCommand.rightCommand)
        hub.Step(dt)
        timeSec += dt
        if safety.status.tripped:
            break

    assert seenAtLeastOneNonZero is True
