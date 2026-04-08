"""HubPackageDriveSmoke.py

Package-backed variant of ``hub/HubDriveSmoke.py`` for Pybricks.

This file intentionally imports the hub-safe ``LegoBalance`` runtime module
instead of carrying the estimator/controller/safety logic inline. It is placed
under ``src/`` rather than ``hub/`` because ``pybricksdev`` resolves multi-file
imports relative to the uploaded entrypoint directory. From here it can find
``src/LegoBalance/HubDriveSmokeRuntime.py`` and upload both files together.

Run it through the desktop plotter:

    python scripts/PlotHubPackageDriveSmoke.py

Or run the hub program directly:

    pybricksdev compile src/HubPackageDriveSmoke.py
    pybricksdev run ble src/HubPackageDriveSmoke.py
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

from LegoBalance.HubDriveSmokeRuntime import (
    DefaultConfig,
    DegPerSecToRadPerSec,
    DegToRad,
    DriveCommand,
    DriveCommandController,
    Measurement,
    RadPerSecToDegPerSec,
    RadToDeg,
    SafetyMonitor,
    StateEstimator,
)

LOOP_PERIOD_MS = 20
PRINT_EVERY_N = 1
STOP_DURATION_MS = 200
DRIVE_DURATION_MS = 3000

DRIVE_SCHEDULE = (
    ("stop", DriveCommand.Stop, STOP_DURATION_MS),
    ("forward", DriveCommand.Forward, DRIVE_DURATION_MS),
    ("stop", DriveCommand.Stop, STOP_DURATION_MS),
    ("backward", DriveCommand.Backward, DRIVE_DURATION_MS),
    ("stop", DriveCommand.Stop, STOP_DURATION_MS),
)


def CenterPressed(hub):
    return Button.CENTER in hub.buttons.pressed()


def MotorPort(name):
    if name == "A":
        return Port.A
    if name == "B":
        return Port.B
    if name == "C":
        return Port.C
    if name == "D":
        return Port.D
    if name == "E":
        return Port.E
    if name == "F":
        return Port.F
    raise ValueError(f"unsupported motor port: {name}")


def MakeMeasurement(hub, leftMotor, rightMotor, timestampSec):
    pitchDeg, _ = hub.imu.tilt()
    _, gyDegPerSec, _ = hub.imu.angular_velocity()
    return Measurement(
        tiltAngle=DegToRad(pitchDeg),
        tiltRate=DegPerSecToRadPerSec(gyDegPerSec),
        leftWheelAngle=DegToRad(leftMotor.angle()),
        rightWheelAngle=DegToRad(rightMotor.angle()),
        leftWheelRate=DegPerSecToRadPerSec(leftMotor.speed()),
        rightWheelRate=DegPerSecToRadPerSec(rightMotor.speed()),
        timestamp=timestampSec,
        valid=True,
    )


def RunVelocity(motor, forwardSign, encoderSign, commandRadPerSec):
    motor.run(forwardSign * encoderSign * RadPerSecToDegPerSec(commandRadPerSec))


def PrintBanner(config):
    print("============================================================")
    print(" HubPackageDriveSmoke : LegoBalance package smoke test")
    print(" This script commands wheel motion. Lift or block the wheels.")
    print(" Press the center button at any time to stop cleanly.")
    print("============================================================")
    print(" Package import:")
    print("   from LegoBalance.HubDriveSmokeRuntime import ...")
    print(" Config:")
    print(f"   loop period         : {LOOP_PERIOD_MS} ms")
    print(f"   telemetry every     : {PRINT_EVERY_N} loop(s)")
    print(f"   drive test speed    : {RadPerSecToDegPerSec(config.drive.testSpeed):.1f} deg/s")
    print(f"   max tilt for motion : {RadToDeg(config.drive.maxTiltForMotion):.1f} deg")
    print(f"   tilt sign           : {config.imu.tiltSign}")
    print(f"   zero offset         : {RadToDeg(config.imu.zeroOffset):.1f} deg")
    print(f"   gyro bias           : {RadPerSecToDegPerSec(config.imu.gyroBias):.1f} deg/s")
    print(f"   forward sign        : {config.motors.forwardSign}")
    print(f"   left encoder sign   : {config.motors.leftEncoderSign}")
    print(f"   right encoder sign  : {config.motors.rightEncoderSign}")
    print(" Schedule:")
    totalMs = 0
    for label, command, durationMs in DRIVE_SCHEDULE:
        if command == DriveCommand.Forward:
            speed = RadPerSecToDegPerSec(config.drive.testSpeed)
        elif command == DriveCommand.Backward:
            speed = -RadPerSecToDegPerSec(config.drive.testSpeed)
        else:
            speed = 0.0
        print(f"   {label:>8s} for {durationMs:>5d} ms at {speed:+6.1f} deg/s")
        totalMs += durationMs
    print(f"   total run time      : {totalMs} ms")
    print(" Post-run diagnostic plot:")
    print("   python scripts/PlotHubPackageDriveSmoke.py")
    print("============================================================")


def Main():
    config = DefaultConfig()
    hub = PrimeHub()
    leftMotor = Motor(MotorPort(config.motors.leftPort))
    rightMotor = Motor(MotorPort(config.motors.rightPort))
    estimator = StateEstimator(config)
    controller = DriveCommandController(config)
    safety = SafetyMonitor(config)

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)

    PrintBanner(config)
    print("Starting in 1.5 s. Hold the robot or block the wheels NOW.")
    wait(1500)

    sw = StopWatch()
    safety.Arm(currentTime=0.0)
    iteration = 0
    safeIterations = 0
    gatedIterations = 0
    previousGated = None
    previousTripped = False
    thetaDeg = 0.0
    thetaDotDegPerSec = 0.0
    phiDeg = 0.0
    phiDotDegPerSec = 0.0

    print(
        "DATA_HEADER,t_s,leg,theta_deg,theta_dot_deg_per_sec,"
        "phi_deg,phi_dot_deg_per_sec,left_cmd_deg_per_sec,right_cmd_deg_per_sec,gate"
    )

    for leg in DRIVE_SCHEDULE:
        legLabel = leg[0]
        legCommand = leg[1]
        legDurationMs = leg[2]

        legStartMs = sw.time()
        legEndMs = legStartMs + legDurationMs
        print("------------------------------------------------------------")
        print(
            f"LEG START : '{legLabel}' command '{legCommand}' for {legDurationMs} ms (t={legStartMs / 1000.0:.2f} s)"
        )
        print("------------------------------------------------------------")

        while sw.time() < legEndMs:
            if CenterPressed(hub):
                print("CENTER BUTTON pressed, aborting smoke flow.")
                leftMotor.stop()
                rightMotor.stop()
                return

            timeSec = sw.time() / 1000.0
            measurement = MakeMeasurement(hub, leftMotor, rightMotor, timeSec)
            state = estimator.Update(measurement)

            controller.SetCommand(legCommand)
            driveGateSafe = safety.IsTiltSafeForDriveMotion(state)
            if not driveGateSafe:
                controller.Stop()

            rawCommand = controller.Compute(state)
            safeCommand = safety.Check(state, rawCommand, currentTime=timeSec)

            thetaDeg = RadToDeg(state.theta)
            thetaDotDegPerSec = RadPerSecToDegPerSec(state.thetaDot)
            phiDeg = RadToDeg(state.phi)
            phiDotDegPerSec = RadPerSecToDegPerSec(state.phiDot)

            gated = (not driveGateSafe) or safety.status.tripped
            if gated:
                gatedIterations += 1
            else:
                safeIterations += 1

            tiltMagnitudeDeg = thetaDeg if thetaDeg >= 0.0 else -thetaDeg
            maxTiltDeg = RadToDeg(config.drive.maxTiltForMotion)
            if previousGated is None:
                if gated:
                    print(
                        f">>> START OVER TILT : |theta|={tiltMagnitudeDeg:.2f} deg > limit {maxTiltDeg:.2f} deg, "
                        "motion gated"
                    )
                else:
                    print(
                        f">>> START SAFE      : |theta|={tiltMagnitudeDeg:.2f} deg <= limit {maxTiltDeg:.2f} deg, "
                        "motion allowed"
                    )
            elif gated and not previousGated:
                print(
                    f">>> OVER TILT       : |theta|={tiltMagnitudeDeg:.2f} deg > limit {maxTiltDeg:.2f} deg, "
                    "MOTION GATED, command forced to 0"
                )
            elif (not gated) and previousGated:
                print(
                    f">>> BACK IN SAFE    : |theta|={tiltMagnitudeDeg:.2f} deg <= limit {maxTiltDeg:.2f} deg, "
                    "motion allowed again"
                )
            previousGated = gated

            if safety.status.tripped and not previousTripped:
                print(f">>> SAFETY TRIPPED  : {safety.status.reasons}")
            previousTripped = safety.status.tripped

            RunVelocity(
                leftMotor,
                config.motors.forwardSign,
                config.motors.leftEncoderSign,
                safeCommand.leftCommand,
            )
            RunVelocity(
                rightMotor,
                config.motors.forwardSign,
                config.motors.rightEncoderSign,
                safeCommand.rightCommand,
            )

            if iteration % PRINT_EVERY_N == 0:
                gateLabel = "GATED" if gated else "SAFE"
                print(
                    f"DATA,{timeSec:.3f},{legLabel},{thetaDeg:+.2f},{thetaDotDegPerSec:+.2f},{phiDeg:+.2f},{phiDotDegPerSec:+.2f},{RadPerSecToDegPerSec(safeCommand.leftCommand):+.2f},{RadPerSecToDegPerSec(safeCommand.rightCommand):+.2f},{gateLabel}"
                )

            iteration += 1
            wait(LOOP_PERIOD_MS)

        print(
            f"LEG END   : '{legLabel}' done at t={sw.time() / 1000.0:.2f} s"
        )

    leftMotor.stop()
    rightMotor.stop()
    leftMotor.brake()
    rightMotor.brake()

    totalIterations = safeIterations + gatedIterations
    if totalIterations <= 0:
        gatedPct = 0.0
    else:
        gatedPct = 100.0 * gatedIterations / totalIterations

    print("============================================================")
    print(" HubPackageDriveSmoke DONE")
    print(f"   total iterations : {totalIterations}")
    print(f"   safe iterations  : {safeIterations}")
    print(f"   gated iterations : {gatedIterations}  ({gatedPct:.1f} %)")
    print(f"   total time       : {sw.time() / 1000.0:.2f} s")
    print(f"   final theta      : {thetaDeg:+.2f} deg")
    print(f"   final theta dot  : {thetaDotDegPerSec:+.2f} deg/s")
    print(f"   final phi        : {phiDeg:+.2f} deg")
    print(f"   final phi dot    : {phiDotDegPerSec:+.2f} deg/s")
    print("   diagnostic plot  : python scripts/PlotHubPackageDriveSmoke.py")
    print("============================================================")


Main()
