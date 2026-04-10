"""HubPackageDriveSmoke.py

Package-backed variant of ``hub/HubDriveSmoke.py`` for Pybricks.

This file intentionally imports the normal ``LegoBalance`` estimator,
controller, safety monitor, state boundary objects, and unit conversions. The
only special hub-side helper is ``LegoBalance.HubDriveSmokeRuntime.DefaultConfig``.
That module is generated from ``configs/Default.yaml`` by the laptop plotter
because the hub cannot parse YAML directly.

It is placed under ``src/`` rather than ``hub/`` because ``pybricksdev`` resolves
multi-file imports relative to the uploaded entrypoint directory. From here it
can find and upload the imported ``LegoBalance`` modules together.

Run it through the desktop plotter:

    python scripts/PlotHubPackageDriveSmoke.py

Or run the hub program directly after regenerating the hub-safe config:

    python scripts/GenerateHubDriveSmokeRuntime.py
    pybricksdev compile src/HubPackageDriveSmoke.py
    pybricksdev run ble src/HubPackageDriveSmoke.py
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.DriveCommandController import DriveCommand, DriveCommandController
from LegoBalance.HubDriveSmokeRuntime import DefaultConfig
from LegoBalance.SafetyMonitor import SafetyMonitor
from LegoBalance.StateEstimator import StateEstimator
from LegoBalance.Units import DegPerSecToRadPerSec, DegToRad, RadPerSecToDegPerSec, RadToDeg


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


def SelectTiltAngleDeg(tiltAxis, pitchDeg, rollDeg):
    axis = str(tiltAxis).strip().lower()
    if axis == "pitch":
        return pitchDeg
    if axis == "roll":
        return rollDeg
    raise ValueError("unsupported imu.tiltAxis: " + str(tiltAxis))


def SelectTiltRateDegPerSec(tiltAxis, gxDegPerSec, gyDegPerSec):
    axis = str(tiltAxis).strip().lower()
    if axis == "pitch":
        return gyDegPerSec
    if axis == "roll":
        return gxDegPerSec
    raise ValueError("unsupported imu.tiltAxis: " + str(tiltAxis))


def HasRightAuxMotor(config):
    return bool(str(getattr(config.motors, "rightAuxPort", "")).strip())


def AverageSignedRightSignal(primaryRawValue, auxRawValue, config):
    signedPrimary = config.motors.rightEncoderSign * primaryRawValue
    if auxRawValue is None:
        return signedPrimary
    signedAux = config.motors.rightAuxEncoderSign * auxRawValue
    return 0.5 * (signedPrimary + signedAux)


def SyntheticRightRawSignal(signedValue, config):
    return config.motors.rightEncoderSign * signedValue


def StopMotors(leftMotor, rightMotor, rightAuxMotor):
    leftMotor.stop()
    rightMotor.stop()
    if rightAuxMotor is not None:
        rightAuxMotor.stop()


def BrakeMotors(leftMotor, rightMotor, rightAuxMotor):
    leftMotor.brake()
    rightMotor.brake()
    if rightAuxMotor is not None:
        rightAuxMotor.brake()


def MakeMeasurement(hub, leftMotor, rightMotor, rightAuxMotor, timestampSec, config):
    pitchDeg, rollDeg = hub.imu.tilt()
    gxDegPerSec, gyDegPerSec, _ = hub.imu.angular_velocity()
    tiltAngleDeg = SelectTiltAngleDeg(config.imu.tiltAxis, pitchDeg, rollDeg)
    tiltRateDegPerSec = SelectTiltRateDegPerSec(
        config.imu.tiltAxis,
        gxDegPerSec,
        gyDegPerSec,
    )
    rightAuxAngleDeg = None if rightAuxMotor is None else rightAuxMotor.angle()
    rightAuxSpeedDps = None if rightAuxMotor is None else rightAuxMotor.speed()
    signedRightAngleDeg = AverageSignedRightSignal(rightMotor.angle(), rightAuxAngleDeg, config)
    signedRightSpeedDps = AverageSignedRightSignal(rightMotor.speed(), rightAuxSpeedDps, config)
    return Measurement(
        tiltAngle=DegToRad(tiltAngleDeg),
        tiltRate=DegPerSecToRadPerSec(tiltRateDegPerSec),
        leftWheelAngle=DegToRad(leftMotor.angle()),
        rightWheelAngle=DegToRad(SyntheticRightRawSignal(signedRightAngleDeg, config)),
        leftWheelRate=DegPerSecToRadPerSec(leftMotor.speed()),
        rightWheelRate=DegPerSecToRadPerSec(SyntheticRightRawSignal(signedRightSpeedDps, config)),
        timestamp=timestampSec,
        valid=True,
    )


def RunVelocity(motor, forwardSign, encoderSign, commandRadPerSec):
    motor.run(forwardSign * encoderSign * RadPerSecToDegPerSec(commandRadPerSec))


def BuildDriveSchedule(config):
    return (
        ("stop", DriveCommand.Stop, config.drive.stopDurationMs),
        ("forward", DriveCommand.Forward, config.drive.driveDurationMs),
        ("stop", DriveCommand.Stop, config.drive.stopDurationMs),
        ("backward", DriveCommand.Backward, config.drive.driveDurationMs),
        ("stop", DriveCommand.Stop, config.drive.stopDurationMs),
    )


def PrintBanner(config):
    driveSchedule = BuildDriveSchedule(config)
    print("============================================================")
    print(" HubPackageDriveSmoke : LegoBalance package smoke test")
    print(" This script commands wheel motion. Lift or block the wheels.")
    print(" Press the center button at any time to stop cleanly.")
    print("============================================================")
    print(" Package import:")
    print("   from LegoBalance.StateEstimator import StateEstimator")
    print("   from LegoBalance.DriveCommandController import DriveCommandController")
    print("   from LegoBalance.SafetyMonitor import SafetyMonitor")
    print("   from LegoBalance.HubDriveSmokeRuntime import DefaultConfig")
    print("   HubDriveSmokeRuntime is generated from configs/Default.yaml")
    print(" Config:")
    print(f"   loop period         : {config.drive.loopPeriodMs} ms")
    print(f"   telemetry every     : {config.drive.printEveryN} loop(s)")
    print(f"   drive test speed    : {RadPerSecToDegPerSec(config.drive.testSpeed):.1f} deg/s")
    print(f"   max tilt for motion : {RadToDeg(config.drive.maxTiltForMotion):.1f} deg")
    print(f"   tilt sign           : {config.imu.tiltSign}")
    print(f"   zero offset         : {RadToDeg(config.imu.zeroOffset):.1f} deg")
    print(f"   gyro bias           : {RadPerSecToDegPerSec(config.imu.gyroBias):.1f} deg/s")
    print(f"   forward sign        : {config.motors.forwardSign}")
    print(f"   left encoder sign   : {config.motors.leftEncoderSign}")
    print(f"   right encoder sign  : {config.motors.rightEncoderSign}")
    print(f"   right aux port      : {config.motors.rightAuxPort or '(disabled)'}")
    print(f"   right aux enc sign  : {config.motors.rightAuxEncoderSign}")
    print(" Schedule:")
    totalMs = 0
    for label, command, durationMs in driveSchedule:
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
    driveSchedule = BuildDriveSchedule(config)
    hub = PrimeHub()
    leftMotor = Motor(MotorPort(config.motors.leftPort))
    rightMotor = Motor(MotorPort(config.motors.rightPort))
    rightAuxMotor = None
    if HasRightAuxMotor(config):
        rightAuxMotor = Motor(MotorPort(config.motors.rightAuxPort))
    estimator = StateEstimator(config)
    controller = DriveCommandController(config)
    safety = SafetyMonitor(config)

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    if rightAuxMotor is not None:
        rightAuxMotor.reset_angle(0)

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

    for leg in driveSchedule:
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
                StopMotors(leftMotor, rightMotor, rightAuxMotor)
                return

            timeSec = sw.time() / 1000.0
            measurement = MakeMeasurement(
                hub,
                leftMotor,
                rightMotor,
                rightAuxMotor,
                timeSec,
                config,
            )
            state = estimator.Update(measurement)

            controller.SetCommand(legCommand)
            driveGateSafe = safety.IsTiltSafeForDriveMotion(state)
            if not driveGateSafe:
                controller.Stop()

            rawCommand = controller.Compute(state)
            safeCommand = safety.Check(state, rawCommand, currentTime=timeSec)

            thetaDeg = RadToDeg(state.tilt)
            thetaDotDegPerSec = RadPerSecToDegPerSec(state.tiltRate)
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
            if rightAuxMotor is not None:
                RunVelocity(
                    rightAuxMotor,
                    config.motors.forwardSign,
                    config.motors.rightAuxEncoderSign,
                    safeCommand.rightCommand,
                )

            if iteration % config.drive.printEveryN == 0:
                gateLabel = "GATED" if gated else "SAFE"
                print(
                    f"DATA,{timeSec:.3f},{legLabel},{thetaDeg:+.2f},{thetaDotDegPerSec:+.2f},{phiDeg:+.2f},{phiDotDegPerSec:+.2f},{RadPerSecToDegPerSec(safeCommand.leftCommand):+.2f},{RadPerSecToDegPerSec(safeCommand.rightCommand):+.2f},{gateLabel}"
                )

            iteration += 1
            wait(config.drive.loopPeriodMs)

        print(
            f"LEG END   : '{legLabel}' done at t={sw.time() / 1000.0:.2f} s"
        )

    StopMotors(leftMotor, rightMotor, rightAuxMotor)
    BrakeMotors(leftMotor, rightMotor, rightAuxMotor)

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
