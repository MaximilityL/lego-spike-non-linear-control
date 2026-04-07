"""End to end mock based test of the controller pipeline.

Wires the same components the simulation example uses and confirms that
they all agree on shapes and types over a few iterations. This catches
interface drift between modules.
"""

from __future__ import annotations

from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.DataLogger import DataLogger
from LegoBalance.LyapunovController import LyapunovController
from LegoBalance.MockAdapters import MockHub
from LegoBalance.RobotConfig import LoadConfig
from LegoBalance.SafetyMonitor import SafetyMonitor
from LegoBalance.StateEstimator import StateEstimator


def test_PipelineRunsForFewIterationsWithoutErrors():
    config = LoadConfig(applyLocalOverride=False)
    hub = MockHub()
    estimator = StateEstimator(config)
    controller = LyapunovController(config)
    safety = SafetyMonitor(config)
    logger = DataLogger(bufferSize=64)

    hub.SetInitialTilt(0.05)
    safety.Arm(currentTime=0.0)

    dt = 0.01
    t = 0.0
    for _ in range(20):
        measurement = Measurement(
            tiltAngle=hub.Imu.TiltAngleRadians(),
            tiltRate=hub.Imu.TiltRateRadiansPerSec(),
            leftWheelAngle=hub.LeftMotor.Angle(),
            rightWheelAngle=hub.RightMotor.Angle(),
            leftWheelRate=hub.LeftMotor.Velocity(),
            rightWheelRate=hub.RightMotor.Velocity(),
            timestamp=t,
            valid=True,
        )
        state = estimator.Update(measurement)
        raw = controller.Compute(state)
        safe = safety.Check(state, raw, currentTime=t)
        hub.LeftMotor.RunVelocity(safe.leftCommand)
        hub.RightMotor.RunVelocity(safe.rightCommand)
        logger.Record(state, safe)
        hub.Step(dt)
        t += dt
        if safety.status.tripped:
            break

    assert len(logger) > 0
    record = logger.Records()[0]
    row = record.AsRow()
    expectedKeys = {
        "timestamp",
        "tilt",
        "tiltRate",
        "wheelPosition",
        "wheelVelocity",
        "stateValid",
        "leftCommand",
        "rightCommand",
        "mode",
    }
    assert expectedKeys.issubset(row.keys())


def test_LoggerCsvWriteCreatesFile(tmp_path):
    config = LoadConfig(applyLocalOverride=False)
    estimator = StateEstimator(config)
    measurement = Measurement(
        tiltAngle=0.1,
        tiltRate=0.2,
        leftWheelAngle=0.0,
        rightWheelAngle=0.0,
        leftWheelRate=0.0,
        rightWheelRate=0.0,
        timestamp=0.0,
        valid=True,
    )
    state = estimator.Update(measurement)
    logger = DataLogger(bufferSize=8)
    logger.Record(state)
    out = tmp_path / "log.csv"
    logger.WriteCsv(out)
    assert out.exists()
    text = out.read_text()
    assert "tilt" in text
