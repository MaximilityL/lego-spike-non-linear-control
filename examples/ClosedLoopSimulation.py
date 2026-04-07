"""ClosedLoopSimulation.py

Desktop side simulation stub. Wires up:

- :class:`LegoBalance.MockAdapters.MockHub` as the toy plant
- :class:`LegoBalance.StateEstimator` as the (currently pass through) estimator
- :class:`LegoBalance.LyapunovController` as the (currently placeholder) controller
- :class:`LegoBalance.SafetyMonitor` as the safety filter
- :class:`LegoBalance.DataLogger` as the recorder

Runs a few hundred iterations and prints a short summary. The point is not
to balance anything (the controller is a placeholder). The point is to
exercise every API on the path from sensors to motors so that the next
person to plug in a real controller has confidence that the wiring is
correct.

Run with:

    python examples/ClosedLoopSimulation.py
"""

from __future__ import annotations

import math
from pathlib import Path

from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.DataLogger import DataLogger
from LegoBalance.LyapunovController import LyapunovController
from LegoBalance.MockAdapters import MockHub
from LegoBalance.RobotConfig import LoadConfig, RobotConfig
from LegoBalance.SafetyMonitor import SafetyMonitor
from LegoBalance.StateEstimator import StateEstimator

SIM_DURATION_SEC = 2.0
SIM_DT_SEC = 0.01
INITIAL_TILT_RAD = 0.05  # 0.05 rad ~ 2.9 deg, well inside the safety envelope


def BuildMeasurement(hub: MockHub, config: RobotConfig, currentTimeSec: float) -> Measurement:
    """Pull a Measurement bundle out of the toy hub.

    The mock hub stores physical tilt and wheel motion. Convert those into
    the raw sensor convention expected by StateEstimator, which then applies
    the same config signs and offsets used for hardware.
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
    controller = LyapunovController(config)
    safety = SafetyMonitor(config)
    logger = DataLogger(bufferSize=int(SIM_DURATION_SEC / SIM_DT_SEC) + 16)

    hub.SetInitialTilt(INITIAL_TILT_RAD)
    safety.Arm(currentTime=0.0)

    print("LegoBalance ClosedLoopSimulation")
    print("-" * 60)
    print(f"controller   : {type(controller).__name__}")
    print(f"placeholder  : {controller.IsPlaceholder()}")
    print(f"sim duration : {SIM_DURATION_SEC:.2f} s at dt={SIM_DT_SEC:.3f} s")
    print(f"initial tilt : {math.degrees(INITIAL_TILT_RAD):+.2f} deg")
    print()

    steps = int(round(SIM_DURATION_SEC / SIM_DT_SEC))
    timeSec = 0.0
    for _ in range(steps):
        measurement = BuildMeasurement(hub, config, timeSec)
        state = estimator.Update(measurement)
        rawCommand = controller.Compute(state)
        safeCommand = safety.Check(state, rawCommand, currentTime=timeSec)

        # Apply commands to the mock motors. The mock implements RunVelocity.
        hub.LeftMotor.RunVelocity(safeCommand.leftCommand)
        hub.RightMotor.RunVelocity(safeCommand.rightCommand)

        logger.Record(state, safeCommand, raw_left=rawCommand.leftCommand)
        hub.Step(SIM_DT_SEC)
        timeSec += SIM_DT_SEC

        if safety.status.tripped:
            print(f"Safety tripped at t={timeSec:.2f}s: {safety.status.reasons}")
            break

    finalState = logger.Records()[-1].state
    derivedP = finalState.LinearPosition(config.chassis.wheelRadius)
    derivedPDot = finalState.LinearVelocity(config.chassis.wheelRadius)
    print(f"records collected : {len(logger)}")
    print(
        f"final state       : theta={finalState.tilt:+.3f} rad, "
        f"thetaDot={finalState.tiltRate:+.3f} rad/s, "
        f"phi={finalState.phi:+.3f} rad, phiDot={finalState.phiDot:+.3f} rad/s"
    )
    print(
        f"derived (r used)  : p={derivedP:+.3f} m, pDot={derivedPDot:+.3f} m/s "
        f"(secondary view, computed from phi using wheel radius)"
    )
    print(f"safety armed      : {safety.status.armed}")
    print(f"safety tripped    : {safety.status.tripped}")
    if safety.status.reasons:
        print(f"safety reasons    : {safety.status.reasons}")

    # Optional CSV dump.
    csvPath = Path("logs/ClosedLoopSimulation.csv")
    logger.WriteCsv(csvPath)
    print(f"log written to    : {csvPath}")
    print()
    print("This run used a placeholder controller. The body of LyapunovController.Compute")
    print("returns zero. The expected outcome is that the toy plant tips over because")
    print("nothing is acting on it. That is the correct, honest behavior for a scaffold.")
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
