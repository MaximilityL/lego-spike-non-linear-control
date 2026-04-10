"""HubPackageBalance.py

Package-backed real balancing run for the SPIKE Prime hub under Pybricks.

This script imports the shared LegoBalance estimator, nonlinear controller,
safety monitor, unit conversions, and generated hub-safe config helper. It is
the package-backed counterpart of the desktop closed-loop balance simulation,
but it runs against the real hub IMU and the real wheel motors.

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

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.HubDriveSmokeRuntime import DefaultConfig
from LegoBalance.NonLinearController import NonLinearController
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


def MakeMeasurement(hub, leftMotor, rightMotor, timestampSec, config):
    pitchDeg, rollDeg = hub.imu.tilt()
    gxDegPerSec, gyDegPerSec, _ = hub.imu.angular_velocity()
    tiltAngleDeg = SelectTiltAngleDeg(config.imu.tiltAxis, pitchDeg, rollDeg)
    tiltRateDegPerSec = SelectTiltRateDegPerSec(
        config.imu.tiltAxis,
        gxDegPerSec,
        gyDegPerSec,
    )
    return Measurement(
        tiltAngle=DegToRad(tiltAngleDeg),
        tiltRate=DegPerSecToRadPerSec(tiltRateDegPerSec),
        leftWheelAngle=DegToRad(leftMotor.angle()),
        rightWheelAngle=DegToRad(rightMotor.angle()),
        leftWheelRate=DegPerSecToRadPerSec(leftMotor.speed()),
        rightWheelRate=DegPerSecToRadPerSec(rightMotor.speed()),
        timestamp=timestampSec,
        valid=True,
    )


def BuildControllerState(state, targetTiltRad):
    if targetTiltRad == 0.0:
        return state
    return BalanceState(
        tilt=state.tilt - targetTiltRad,
        tiltRate=state.tiltRate,
        phi=state.phi,
        phiDot=state.phiDot,
        timestamp=state.timestamp,
        valid=state.valid,
    )


def RunVelocity(motor, forwardSign, encoderSign, commandRadPerSec):
    motor.run(forwardSign * encoderSign * RadPerSecToDegPerSec(commandRadPerSec))


def PrintBanner(config, loopPeriodMs, telemetryEveryN):
    print("============================================================")
    print(" HubPackageBalance : real package-backed balancing run")
    print(" This script uses the shared NonLinearController on the hub.")
    print(" Start with the robot near upright and be ready to catch it.")
    print(" Press the center button at any time to stop cleanly.")
    print("============================================================")
    print(" Package import:")
    print("   from LegoBalance.StateEstimator import StateEstimator")
    print("   from LegoBalance.NonLinearController import NonLinearController")
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
    estimator = StateEstimator(config)
    controller = NonLinearController(config)
    safety = SafetyMonitor(config)

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    estimator.Reset()
    controller.Reset()

    PrintBanner(config, loopPeriodMs, telemetryEveryN)
    print("Starting in 1.5 s. Hold the robot near upright NOW.")
    wait(START_DELAY_MS)

    sw = StopWatch()
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
            leftMotor.stop()
            rightMotor.stop()
            return

        timeSec = sw.time() / 1000.0
        measurement = MakeMeasurement(hub, leftMotor, rightMotor, timeSec, config)
        state = estimator.Update(measurement)
        controllerState = BuildControllerState(state, config.control.targetTilt)
        rawCommand = controller.Compute(controllerState)
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
        wait(loopPeriodMs)

    leftMotor.stop()
    rightMotor.stop()
    leftMotor.brake()
    rightMotor.brake()

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
