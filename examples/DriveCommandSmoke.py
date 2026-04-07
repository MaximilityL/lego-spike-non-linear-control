"""DriveCommandSmoke.py

Desktop side, mock backed smoke flow for the post sign verification, pre
balancing phase. Wires up the same components a real run would use:

- :class:`LegoBalance.MockAdapters.MockHub` as the toy hub
- :class:`LegoBalance.StateEstimator` as the light sign aware estimator
- :class:`LegoBalance.DriveCommandController` as the open loop drive command
  path (forward / backward / stop)
- :class:`LegoBalance.SafetyMonitor` as the safety filter (with the drive
  specific tilt gate)
- :class:`LegoBalance.DataLogger` as the recorder

Walks through a forward, stop, backward, stop sequence, prints the
implemented state ``[theta, thetaDot, phi, phiDot]`` for each step, and
writes a CSV log to ``logs/DriveCommandSmoke.csv``.

This is intentionally not a balancing controller. The point is to prove
that the wiring through the real package abstractions is correct, that the
estimator state is sane, and that the safety monitor still gates motion
the way the future balancing controller will rely on.

Run with:

    python examples/DriveCommandSmoke.py
"""

from __future__ import annotations

import math
from pathlib import Path

from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.DataLogger import DataLogger
from LegoBalance.DriveCommandController import DriveCommand, DriveCommandController
from LegoBalance.MockAdapters import MockHub
from LegoBalance.RobotConfig import LoadConfig, RobotConfig
from LegoBalance.SafetyMonitor import SafetyMonitor
from LegoBalance.StateEstimator import StateEstimator

SIM_DT_SEC = 0.01
INITIAL_TILT_RAD = 0.02  # ~1.1 deg, well inside the drive gate

# A short scripted sequence so the smoke flow has a defined ending.
DRIVE_SCHEDULE: list[tuple[DriveCommand, int]] = [
    (DriveCommand.Stop, 10),
    (DriveCommand.Forward, 30),
    (DriveCommand.Stop, 10),
    (DriveCommand.Backward, 30),
    (DriveCommand.Stop, 10),
]


def BuildMeasurement(hub: MockHub, config: RobotConfig, currentTimeSec: float) -> Measurement:
    """Pull a Measurement bundle out of the toy hub."""
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
        tiltRate=config.imu.tiltSign * (hub.Imu.TiltRateRadiansPerSec() + config.imu.gyroBias),
        leftWheelAngle=rawLeftAngle,
        rightWheelAngle=rawRightAngle,
        leftWheelRate=rawLeftRate,
        rightWheelRate=rawRightRate,
        timestamp=currentTimeSec,
        valid=True,
    )


def Main() -> int:
    config = LoadConfig()
    hub = MockHub()
    estimator = StateEstimator(config)
    controller = DriveCommandController(config)
    safety = SafetyMonitor(config)
    logger = DataLogger(bufferSize=4096)

    hub.SetInitialTilt(INITIAL_TILT_RAD)
    safety.Arm(currentTime=0.0)

    print("LegoBalance DriveCommandSmoke (pre balancing)")
    print("-" * 60)
    print(f"controller   : {type(controller).__name__}")
    print(f"drive speed  : {controller.driveSpeed:+.2f} rad/s")
    print(f"initial tilt : {math.degrees(INITIAL_TILT_RAD):+.2f} deg")
    print(
        "primary state: [theta (rad), thetaDot (rad/s), phi (rad), phiDot (rad/s)]"
    )
    print()

    timeSec = 0.0
    print(
        f"{'t_s':>6}  {'cmd':<9}  {'theta':>8}  {'thetaDot':>9}  "
        f"{'phi':>8}  {'phiDot':>9}  {'left':>7}  {'right':>7}"
    )

    for command, steps in DRIVE_SCHEDULE:
        controller.SetCommand(command)
        for _ in range(steps):
            measurement = BuildMeasurement(hub, config, timeSec)
            state = estimator.Update(measurement)

            # The drive controller is non balancing. The safety monitor
            # exposes a tighter "is the body upright enough to drive"
            # gate; if it returns False we substitute a stop.
            if not safety.IsTiltSafeForDriveMotion(state):
                controller.Stop()

            rawCommand = controller.Compute(state)
            safeCommand = safety.Check(state, rawCommand, currentTime=timeSec)

            hub.LeftMotor.RunVelocity(safeCommand.leftCommand)
            hub.RightMotor.RunVelocity(safeCommand.rightCommand)
            logger.Record(state, safeCommand, command=command.value)
            hub.Step(SIM_DT_SEC)

            print(
                f"{timeSec:6.2f}  {command.value:<9}  "
                f"{state.tilt:+8.3f}  {state.tiltRate:+9.3f}  "
                f"{state.phi:+8.3f}  {state.phiDot:+9.3f}  "
                f"{safeCommand.leftCommand:+7.2f}  {safeCommand.rightCommand:+7.2f}"
            )
            timeSec += SIM_DT_SEC
            if safety.status.tripped:
                print(f"Safety tripped: {safety.status.reasons}")
                break
        if safety.status.tripped:
            break

    finalState = logger.Records()[-1].state if len(logger) else None
    if finalState is not None:
        print()
        print(
            f"final state  : theta={finalState.tilt:+.3f} rad, "
            f"thetaDot={finalState.tiltRate:+.3f} rad/s, "
            f"phi={finalState.phi:+.3f} rad, phiDot={finalState.phiDot:+.3f} rad/s"
        )
        derivedP = finalState.LinearPosition(config.chassis.wheelRadius)
        derivedPDot = finalState.LinearVelocity(config.chassis.wheelRadius)
        print(
            f"derived (r=0): p={derivedP:+.3f} m, pDot={derivedPDot:+.3f} m/s "
            f"(secondary view, requires wheel radius)"
        )

    csvPath = Path("logs/DriveCommandSmoke.csv")
    logger.WriteCsv(csvPath)
    print(f"log written  : {csvPath}")
    print()
    print("This is the pre balancing motion smoke flow. The robot is not")
    print("balanced; the controller only commands forward/stop/backward at a")
    print("hardware-validated smoke-test magnitude. Run this with the wheels lifted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
