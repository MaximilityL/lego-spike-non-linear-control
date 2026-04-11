"""HubPackageBalance.py

Package-backed real balancing run for the SPIKE Prime hub under Pybricks.

This script imports the shared LegoBalance estimator, config-selected balance
controller, safety monitor, unit conversions, and generated hub-safe config
helper. It is the package-backed counterpart of the desktop closed-loop
balance simulation, but it runs against the real hub IMU and the real wheel
motors.

Run it through the desktop plotter:

    python scripts/PlotHubPackageBalance.py

Or run it directly after regenerating the hub-safe config:

    python scripts/GenerateHubDriveSmokeRuntime.py
    pybricksdev compile src/HubPackageBalance.py
    pybricksdev run ble src/HubPackageBalance.py
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

from LegoBalance.BalanceControllerFactory import BuildBalanceController
from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.HubDriveSmokeRuntime import DefaultConfig
from LegoBalance.SafetyMonitor import SafetyMonitor
from LegoBalance.StateEstimator import StateEstimator
from LegoBalance.Units import DegPerSecToRadPerSec, DegToRad, RadPerSecToDegPerSec, RadToDeg

START_DELAY_MS = 1500
RUN_DURATION_MS = 50000


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
    # Pybricks returns tilt as (pitch, roll). The matching gyro components
    # are gy for pitch and gx for roll.
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


def ControllerLabel(config, controller):
    algorithm = str(getattr(config.controller, "algorithm", "")).strip().lower()
    if algorithm == "pid":
        return "PidController"
    if algorithm in ("tanh", "nonlinear"):
        return "NonLinearController"
    controllerClass = getattr(controller, "__class__", None)
    name = getattr(controllerClass, "__name__", "")
    if name:
        return name
    return "BalanceController"


def PrintBanner(config, loopPeriodMs, telemetryEveryN, controller):
    controllerName = ControllerLabel(config, controller)
    print("============================================================")
    print(" HubPackageBalance : real package-backed balancing run")
    print(" This script uses the shared balance controller selected by config.")
    print(" Start with the robot near upright and be ready to catch it.")
    print(" Press the center button at any time to stop cleanly.")
    print("============================================================")
    print(" Package import:")
    print("   from LegoBalance.BalanceControllerFactory import BuildBalanceController")
    print("   from LegoBalance.StateEstimator import StateEstimator")
    print("   from LegoBalance.SafetyMonitor import SafetyMonitor")
    print("   from LegoBalance.HubDriveSmokeRuntime import DefaultConfig")
    print(" Config:")
    print(f"   loop period         : {loopPeriodMs} ms")
    print(f"   telemetry every     : {telemetryEveryN} loop(s)")
    print(f"   run duration        : {RUN_DURATION_MS} ms")
    print(f"   target tilt         : {RadToDeg(config.control.targetTilt):+.2f} deg")
    print(f"   max tilt            : {RadToDeg(config.control.maxTilt):.1f} deg")
    print(f"   max wheel rate      : {RadPerSecToDegPerSec(config.control.maxWheelRate):.1f} deg/s")
    print(f"   tilt axis           : {config.imu.tiltAxis}")
    print(f"   tilt sign           : {config.imu.tiltSign}")
    print(f"   zero offset         : {RadToDeg(config.imu.zeroOffset):.1f} deg")
    print(f"   gyro bias           : {RadPerSecToDegPerSec(config.imu.gyroBias):.1f} deg/s")
    print(f"   forward sign        : {config.motors.forwardSign}")
    print(f"   left encoder sign   : {config.motors.leftEncoderSign}")
    print(f"   right encoder sign  : {config.motors.rightEncoderSign}")
    print(f"   right aux port      : {config.motors.rightAuxPort or '(disabled)'}")
    print(f"   right aux enc sign  : {config.motors.rightAuxEncoderSign}")
    print(f" Controller selected   : {controllerName}")
    print(f" Controller algorithm  : {config.controller.algorithm}")
    if str(config.controller.algorithm).strip().lower() == "pid":
        print(" Controller gains (PidController):")
        print(f"   pidKp               : {config.controller.pidKp}")
        print(f"   pidKi               : {config.controller.pidKi}")
        print(f"   pidKd               : {config.controller.pidKd}")
        print(f"   pidKs               : {config.controller.pidKs}")
        print(f"   pidIntegralStep     : {config.controller.pidIntegralStep}")
        print(f"   pidIntegralLimit    : {config.controller.pidIntegralLimit}")
        print(f"   pidPositionTargetDeg: {config.controller.pidPositionTargetDeg}")
    else:
        print(" Controller gains (NonLinearController):")
        print(f"   bodyMass            : {config.chassis.bodyMass}")
        print(f"   bodyHeightCoM       : {config.chassis.bodyHeightCoM}")
        print(f"   bodyInertia (CoM)   : {config.chassis.bodyInertia}")
        print(f"   wheelRadius         : {config.chassis.wheelRadius}")
        print(f"   innerNaturalFreq    : {config.controller.innerNaturalFrequency}")
        print(f"   innerDampingRatio   : {config.controller.innerDampingRatio}")
        print(f"   surfaceGain         : {config.controller.surfaceGain}")
        print(f"   robustGain          : {config.controller.robustGain}")
        print(f"   boundaryLayerWidth  : {config.controller.boundaryLayerWidth}")
        print(f"   outerPositionGain   : {config.controller.outerPositionGain}")
        print(f"   outerVelocityGain   : {config.controller.outerVelocityGain}")
        print(
            f"   maxRefTiltOffset    : {RadToDeg(config.controller.maxReferenceTiltOffset):.2f} deg"
        )
        print(f"   actuatorTau         : {config.controller.actuatorTau}")
        print(f"   thetaDotFilterAlpha : {config.controller.thetaDotFilterAlpha}")
    print(" Post-run diagnostic plot:")
    print("   python scripts/PlotHubPackageBalance.py")
    print("============================================================")


def Main():
    config = DefaultConfig()
    loopPeriodMs = int(1000.0 / config.control.loopRate + 0.5)
    if loopPeriodMs < 1:
        loopPeriodMs = 1
    telemetryEveryN = config.drive.printEveryN
    if telemetryEveryN < 1:
        telemetryEveryN = 1

    hub = PrimeHub()
    leftMotor = Motor(MotorPort(config.motors.leftPort))
    rightMotor = Motor(MotorPort(config.motors.rightPort))
    rightAuxMotor = None
    if HasRightAuxMotor(config):
        rightAuxMotor = Motor(MotorPort(config.motors.rightAuxPort))
    estimator = StateEstimator(config)
    controller = BuildBalanceController(config)
    safety = SafetyMonitor(config)

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    if rightAuxMotor is not None:
        rightAuxMotor.reset_angle(0)
    estimator.Reset()
    controller.Reset()

    PrintBanner(config, loopPeriodMs, telemetryEveryN, controller)
    print("Starting in 1.5 s. Hold the robot near upright NOW.")
    wait(START_DELAY_MS)

    sw = StopWatch()
    nextTickMs = loopPeriodMs
    safety.Arm(currentTime=0.0)
    iteration = 0
    previousTripped = False
    previousArmed = True
    finalThetaDeg = 0.0
    finalThetaDotDegPerSec = 0.0
    finalPhiDeg = 0.0
    finalPhiDotDegPerSec = 0.0
    finalRawCmdDegPerSec = 0.0
    finalSafeCmdDegPerSec = 0.0

    print(
        "DATA_HEADER,t_s,theta_ref_deg,theta_deg,theta_dot_deg_per_sec,"
        "phi_deg,phi_dot_deg_per_sec,raw_cmd_deg_per_sec,safe_cmd_deg_per_sec,status"
    )

    while sw.time() < RUN_DURATION_MS:
        if CenterPressed(hub):
            print("CENTER BUTTON pressed, aborting balance run.")
            StopMotors(leftMotor, rightMotor, rightAuxMotor)
            return

        timeSec = sw.time() / 1000.0
        measurement = MakeMeasurement(hub, leftMotor, rightMotor, rightAuxMotor, timeSec, config)
        state = estimator.Update(measurement)
        rawCommand = controller.Compute(state)
        safeCommand = safety.Check(state, rawCommand, currentTime=timeSec)

        finalThetaDeg = RadToDeg(state.tilt)
        finalThetaDotDegPerSec = RadPerSecToDegPerSec(state.tiltRate)
        finalPhiDeg = RadToDeg(state.phi)
        finalPhiDotDegPerSec = RadPerSecToDegPerSec(state.phiDot)
        finalRawCmdDegPerSec = RadPerSecToDegPerSec(rawCommand.leftCommand)
        finalSafeCmdDegPerSec = RadPerSecToDegPerSec(safeCommand.leftCommand)

        if safety.status.tripped and not previousTripped:
            print(f">>> SAFETY TRIPPED  : {safety.status.reasons}")
        previousTripped = safety.status.tripped

        if safety.status.armed != previousArmed:
            if safety.status.armed:
                print(">>> SAFETY ARMED")
            else:
                print(">>> SAFETY DISARMED")
        previousArmed = safety.status.armed

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

        if iteration % telemetryEveryN == 0:
            if safety.status.tripped:
                statusLabel = "TRIPPED"
            elif safety.status.armed:
                statusLabel = "ACTIVE"
            else:
                statusLabel = "DISARMED"
            print(
                f"DATA,{timeSec:.3f},{RadToDeg(config.control.targetTilt):+.2f},{finalThetaDeg:+.2f},{finalThetaDotDegPerSec:+.2f},{finalPhiDeg:+.2f},{finalPhiDotDegPerSec:+.2f},{finalRawCmdDegPerSec:+.2f},{finalSafeCmdDegPerSec:+.2f},{statusLabel}"
            )

        iteration += 1
        if safety.status.tripped:
            break
        remainingMs = nextTickMs - sw.time()
        if remainingMs > 0:
            wait(remainingMs)
        else:
            nextTickMs = sw.time()
        nextTickMs += loopPeriodMs

    StopMotors(leftMotor, rightMotor, rightAuxMotor)
    BrakeMotors(leftMotor, rightMotor, rightAuxMotor)

    print("============================================================")
    print(" HubPackageBalance DONE")
    print(f"   total iterations : {iteration}")
    print(f"   total time       : {sw.time() / 1000.0:.2f} s")
    print(f"   final theta      : {finalThetaDeg:+.2f} deg")
    print(f"   final theta dot  : {finalThetaDotDegPerSec:+.2f} deg/s")
    print(f"   final phi        : {finalPhiDeg:+.2f} deg")
    print(f"   final phi dot    : {finalPhiDotDegPerSec:+.2f} deg/s")
    print(f"   final raw cmd    : {finalRawCmdDegPerSec:+.2f} deg/s")
    print(f"   final safe cmd   : {finalSafeCmdDegPerSec:+.2f} deg/s")
    print(f"   safety armed     : {safety.status.armed}")
    print(f"   safety tripped   : {safety.status.tripped}")
    if safety.status.reasons:
        print(f"   safety reasons   : {safety.status.reasons}")
    print("   diagnostic plot  : python scripts/PlotHubPackageBalance.py")
    print("============================================================")


Main()
