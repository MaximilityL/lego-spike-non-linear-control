"""HubPackageBalanceBuffered.py

Package-backed balancing run that buffers telemetry on the hub in RAM for
the entire run, then dumps every sample over BLE once the run finishes.

This is the buffered counterpart of src/HubPackageBalance.py. The live
version prints a telemetry row every loop iteration, which blocks the
100 Hz control loop over BLE and forces printEveryN >= 2 just to keep the
loop timing honest. This buffered version skips the per-iteration prints
entirely and records every control iteration (no decimation), so the
laptop-side plot gets the full 100 Hz resolution for analysis.

Run it through the desktop plotter:

    python scripts/PlotHubPackageBalanceBuffered.py

Or run it directly after regenerating the hub-safe config:

    python scripts/GenerateHubDriveSmokeRuntime.py
    pybricksdev run ble src/HubPackageBalanceBuffered.py
"""

import ustruct

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

from LegoBalance.BalanceControllerFactory import BuildBalanceController
from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.HubDriveSmokeRuntime import DefaultConfig
from LegoBalance.SafetyMonitor import SafetyMonitor
from LegoBalance.StateEstimator import StateEstimator
from LegoBalance.Units import DegPerSecToRadPerSec, DegToRad, RadPerSecToDegPerSec, RadToDeg

START_DELAY_MS = 1500
RUN_DURATION_MS = 25000

STATUS_ACTIVE = 0
STATUS_TRIPPED = 1
STATUS_DISARMED = 2


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
    raise ValueError("unsupported motor port: " + str(name))


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


def ComputeBufferCapacity(loopPeriodMs, runDurationMs):
    # Grow the buffer a little past the theoretical upper bound so the
    # occasional extra iteration from jitter never overwrites the tail.
    margin = 100
    period = loopPeriodMs if loopPeriodMs > 0 else 1
    return int(runDurationMs // period) + margin


def MakeFloatBuffer(capacity):
    # Pybricks MicroPython does not ship the ``array`` module, so each
    # float32 channel is a plain bytearray accessed via ustruct.pack_into.
    # Four bytes per sample; the buffer starts zeroed.
    return bytearray(capacity * 4)


def WriteFloat(buf, index, value):
    ustruct.pack_into("<f", buf, index * 4, value)


def ReadFloat(buf, index):
    return ustruct.unpack_from("<f", buf, index * 4)[0]


def StatusCode(safetyStatus):
    if safetyStatus.tripped:
        return STATUS_TRIPPED
    if not safetyStatus.armed:
        return STATUS_DISARMED
    return STATUS_ACTIVE


def StatusLabel(code):
    if code == STATUS_TRIPPED:
        return "TRIPPED"
    if code == STATUS_DISARMED:
        return "DISARMED"
    return "ACTIVE"


def PrintBanner(config, loopPeriodMs, bufferCapacity, controller):
    controllerName = ControllerLabel(config, controller)
    print("============================================================")
    print(" HubPackageBalanceBuffered : buffered package-backed balance run")
    print(" Telemetry is buffered on the hub and dumped after the run.")
    print(" Start with the robot near upright and be ready to catch it.")
    print(" Press the center button at any time to stop cleanly.")
    print("============================================================")
    print(" Package import:")
    print("   from LegoBalance.BalanceControllerFactory import BuildBalanceController")
    print("   from LegoBalance.StateEstimator import StateEstimator")
    print("   from LegoBalance.SafetyMonitor import SafetyMonitor")
    print("   from LegoBalance.HubDriveSmokeRuntime import DefaultConfig")
    print(" Config:")
    print("   loop period         : " + str(loopPeriodMs) + " ms")
    print("   run duration        : " + str(RUN_DURATION_MS) + " ms")
    print("   buffer capacity     : " + str(bufferCapacity) + " samples")
    print("   target tilt         : " + "{:+.2f}".format(RadToDeg(config.control.targetTilt)) + " deg")
    print("   max tilt            : " + "{:.1f}".format(RadToDeg(config.control.maxTilt)) + " deg")
    print("   max wheel rate      : " + "{:.1f}".format(RadPerSecToDegPerSec(config.control.maxWheelRate)) + " deg/s")
    print("   tilt axis           : " + str(config.imu.tiltAxis))
    print("   tilt sign           : " + str(config.imu.tiltSign))
    print("   zero offset         : " + "{:.2f}".format(RadToDeg(config.imu.zeroOffset)) + " deg")
    print("   gyro bias           : " + "{:.2f}".format(RadPerSecToDegPerSec(config.imu.gyroBias)) + " deg/s")
    print("   forward sign        : " + str(config.motors.forwardSign))
    print("   left encoder sign   : " + str(config.motors.leftEncoderSign))
    print("   right encoder sign  : " + str(config.motors.rightEncoderSign))
    print("   right aux port      : " + (config.motors.rightAuxPort or "(disabled)"))
    print("   right aux enc sign  : " + str(config.motors.rightAuxEncoderSign))
    print(" Controller selected   : " + controllerName)
    print(" Controller algorithm  : " + str(config.controller.algorithm))
    if str(config.controller.algorithm).strip().lower() == "pid":
        print(" Controller gains (PidController):")
        print("   pidKp               : " + str(config.controller.pidKp))
        print("   pidKi               : " + str(config.controller.pidKi))
        print("   pidKd               : " + str(config.controller.pidKd))
        print("   pidKs               : " + str(config.controller.pidKs))
        print("   pidIntegralStep     : " + str(config.controller.pidIntegralStep))
        print("   pidIntegralLimit    : " + str(config.controller.pidIntegralLimit))
        print("   pidPositionTargetDeg: " + str(config.controller.pidPositionTargetDeg))
    else:
        print(" Controller gains (NonLinearController):")
        print("   bodyMass            : " + str(config.chassis.bodyMass))
        print("   bodyHeightCoM       : " + str(config.chassis.bodyHeightCoM))
        print("   bodyInertia (CoM)   : " + str(config.chassis.bodyInertia))
        print("   wheelRadius         : " + str(config.chassis.wheelRadius))
        print("   innerNaturalFreq    : " + str(config.controller.innerNaturalFrequency))
        print("   innerDampingRatio   : " + str(config.controller.innerDampingRatio))
        print("   surfaceGain         : " + str(config.controller.surfaceGain))
        print("   robustGain          : " + str(config.controller.robustGain))
        print("   boundaryLayerWidth  : " + str(config.controller.boundaryLayerWidth))
        print("   outerPositionGain   : " + str(config.controller.outerPositionGain))
        print("   outerVelocityGain   : " + str(config.controller.outerVelocityGain))
        print("   maxRefTiltOffset    : " + "{:.2f}".format(RadToDeg(config.controller.maxReferenceTiltOffset)) + " deg")
        print("   actuatorTau         : " + str(config.controller.actuatorTau))
        print("   thetaDotFilterAlpha : " + str(config.controller.thetaDotFilterAlpha))
    print(" Post-run diagnostic plot:")
    print("   python scripts/PlotHubPackageBalanceBuffered.py")
    print("============================================================")


def DumpBuffer(
    writeIndex,
    timeBuf,
    thetaRefBuf,
    thetaBuf,
    thetaDotBuf,
    phiBuf,
    phiDotBuf,
    rawCmdBuf,
    safeCmdBuf,
    statusBuf,
):
    print(
        "DATA_HEADER,t_s,theta_ref_deg,theta_deg,theta_dot_deg_per_sec,"
        "phi_deg,phi_dot_deg_per_sec,raw_cmd_deg_per_sec,safe_cmd_deg_per_sec,status"
    )
    print("BUFFER_BEGIN,samples=" + str(writeIndex))
    for i in range(writeIndex):
        print(
            "DATA,"
            + "{:.3f}".format(ReadFloat(timeBuf, i)) + ","
            + "{:+.2f}".format(ReadFloat(thetaRefBuf, i)) + ","
            + "{:+.2f}".format(ReadFloat(thetaBuf, i)) + ","
            + "{:+.2f}".format(ReadFloat(thetaDotBuf, i)) + ","
            + "{:+.2f}".format(ReadFloat(phiBuf, i)) + ","
            + "{:+.2f}".format(ReadFloat(phiDotBuf, i)) + ","
            + "{:+.2f}".format(ReadFloat(rawCmdBuf, i)) + ","
            + "{:+.2f}".format(ReadFloat(safeCmdBuf, i)) + ","
            + StatusLabel(statusBuf[i])
        )
        # Let BLE drain every so often; bursting 2500 lines back to back
        # can overflow the notification queue on some stacks.
        if (i & 0x3F) == 0x3F:
            wait(2)
    print("BUFFER_END")


class EffectsState:
    # Pybricks MicroPython is happier with plain classes than dataclasses, so
    # everything the effects pipeline needs lives here as ordinary attributes.
    def __init__(self, effectsCfg):
        windowSize = int(effectsCfg.qualityRmsWindowSamples)
        if windowSize < 1:
            windowSize = 1
        self.windowSize = windowSize
        self.sqBuffer = [0.0] * windowSize
        self.bufferIndex = 0
        self.windowFilled = 0
        self.sqSum = 0.0
        self.eyesBucket = 0
        self.mouthCategory = 0
        self.lastThetaDeg = None
        self.lastChimeMs = -1000000
        self.lastLightHue = -1


def DrawFace(hub, bucket, mouth):
    # Horizontal face: eyes on row 1, mouth on rows 3-4. Bucket sign is the
    # opposite of phi sign so the eyes look back where the robot came from
    # instead of where it rolled. Mouth is smile/neutral/frown driven by
    # balance quality (RMS theta error) thresholds.
    hub.display.off()
    if mouth > 0:
        # Smile: corners lifted on row 3, center dropped on row 4.
        hub.display.pixel(3, 1, 60)
        hub.display.pixel(3, 3, 60)
        hub.display.pixel(4, 2, 60)
    elif mouth < 0:
        # Frown: center lifted on row 3, corners dropped on row 4.
        hub.display.pixel(3, 2, 60)
        hub.display.pixel(4, 1, 60)
        hub.display.pixel(4, 3, 60)
    else:
        # Neutral: flat bar on row 3.
        hub.display.pixel(3, 1, 60)
        hub.display.pixel(3, 2, 60)
        hub.display.pixel(3, 3, 60)
    if bucket < 0:
        hub.display.pixel(1, 2, 100)
        hub.display.pixel(1, 4, 100)
    elif bucket > 0:
        hub.display.pixel(1, 0, 100)
        hub.display.pixel(1, 2, 100)
    else:
        hub.display.pixel(1, 1, 100)
        hub.display.pixel(1, 3, 100)


def InitializeEffects(hub, effectsCfg):
    if not effectsCfg.enabled:
        return None
    state = EffectsState(effectsCfg)
    if effectsCfg.eyesEnabled:
        DrawFace(hub, state.eyesBucket, state.mouthCategory)
    return state


def UpdateEffects(state, effectsCfg, hub, thetaDeg, phiDeg, nowMs):
    if state is None or not effectsCfg.enabled:
        return

    if effectsCfg.eyesEnabled:
        enter = effectsCfg.eyesPhiDeadbandDeg
        # Exit threshold is smaller than enter so phi has to rebound clearly
        # past center before the eyes return, giving a clean hysteresis band.
        exitGap = enter * 0.4
        newBucket = state.eyesBucket
        if state.eyesBucket == 0:
            if phiDeg >= enter:
                newBucket = 1
            elif phiDeg <= -enter:
                newBucket = -1
        elif state.eyesBucket > 0:
            if phiDeg <= exitGap:
                newBucket = 0
        else:
            if phiDeg >= -exitGap:
                newBucket = 0
        if newBucket != state.eyesBucket:
            state.eyesBucket = newBucket
            DrawFace(hub, state.eyesBucket, state.mouthCategory)

    if effectsCfg.qualityLightEnabled:
        sq = thetaDeg * thetaDeg
        state.sqSum -= state.sqBuffer[state.bufferIndex]
        state.sqBuffer[state.bufferIndex] = sq
        state.sqSum += sq
        state.bufferIndex = (state.bufferIndex + 1) % state.windowSize
        if state.windowFilled < state.windowSize:
            state.windowFilled += 1
        meanSq = state.sqSum / state.windowFilled
        if meanSq < 0.0:
            meanSq = 0.0
        rmsDeg = meanSq ** 0.5
        greenT = effectsCfg.qualityGreenThresholdDeg
        redT = effectsCfg.qualityRedThresholdDeg
        span = redT - greenT
        if rmsDeg <= greenT or span <= 0.0:
            ratio = 0.0
        elif rmsDeg >= redT:
            ratio = 1.0
        else:
            ratio = (rmsDeg - greenT) / span
        hue = int(120.0 - 120.0 * ratio)
        # Quantize to 10 degree hue steps so the LED does not twitch every
        # loop iteration when the error is slowly drifting.
        quantized = (hue // 10) * 10
        if quantized != state.lastLightHue:
            state.lastLightHue = quantized
            hub.light.on(Color(quantized, 100, 100))

        if rmsDeg <= greenT:
            newMouth = 1
        elif rmsDeg >= redT:
            newMouth = -1
        else:
            newMouth = 0
        if effectsCfg.eyesEnabled and newMouth != state.mouthCategory:
            state.mouthCategory = newMouth
            DrawFace(hub, state.eyesBucket, state.mouthCategory)

    if effectsCfg.uprightChimeEnabled and state.lastThetaDeg is not None:
        prev = state.lastThetaDeg
        crossed = (prev > 0.0 and thetaDeg <= 0.0) or (prev < 0.0 and thetaDeg >= 0.0)
        if crossed and (nowMs - state.lastChimeMs) >= effectsCfg.uprightChimeMinIntervalMs:
            # Pybricks Speaker.beep is synchronous and has no stop method, so
            # the call blocks for uprightChimeDurationMs. Keep the duration
            # small compared to the control loop period.
            hub.speaker.beep(
                effectsCfg.uprightChimeFrequencyHz,
                effectsCfg.uprightChimeDurationMs,
            )
            state.lastChimeMs = nowMs

    state.lastThetaDeg = thetaDeg


def ShutdownEffects(state, hub):
    if state is None:
        return
    hub.display.off()
    hub.light.off()


def Main():
    config = DefaultConfig()
    loopPeriodMs = int(1000.0 / config.control.loopRate + 0.5)
    if loopPeriodMs < 1:
        loopPeriodMs = 1
    bufferCapacity = ComputeBufferCapacity(loopPeriodMs, RUN_DURATION_MS)

    timeBuf = MakeFloatBuffer(bufferCapacity)
    thetaRefBuf = MakeFloatBuffer(bufferCapacity)
    thetaBuf = MakeFloatBuffer(bufferCapacity)
    thetaDotBuf = MakeFloatBuffer(bufferCapacity)
    phiBuf = MakeFloatBuffer(bufferCapacity)
    phiDotBuf = MakeFloatBuffer(bufferCapacity)
    rawCmdBuf = MakeFloatBuffer(bufferCapacity)
    safeCmdBuf = MakeFloatBuffer(bufferCapacity)
    statusBuf = bytearray(bufferCapacity)

    hub = PrimeHub()
    leftMotor = Motor(MotorPort(config.motors.leftPort))
    rightMotor = Motor(MotorPort(config.motors.rightPort))
    rightAuxMotor = None
    if HasRightAuxMotor(config):
        rightAuxMotor = Motor(MotorPort(config.motors.rightAuxPort))
    estimator = StateEstimator(config)
    controller = BuildBalanceController(config)
    safety = SafetyMonitor(config)
    effectsState = InitializeEffects(hub, config.effects)

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    if rightAuxMotor is not None:
        rightAuxMotor.reset_angle(0)
    estimator.Reset()
    controller.Reset()

    PrintBanner(config, loopPeriodMs, bufferCapacity, controller)
    print("Starting in 1.5 s. Hold the robot near upright NOW.")
    wait(START_DELAY_MS)

    thetaRefDeg = RadToDeg(config.control.targetTilt)

    sw = StopWatch()
    nextTickMs = loopPeriodMs
    safety.Arm(currentTime=0.0)
    iteration = 0
    writeIndex = 0
    previousTripped = False
    previousArmed = True
    abortedByButton = False

    finalThetaDeg = 0.0
    finalThetaDotDegPerSec = 0.0
    finalPhiDeg = 0.0
    finalPhiDotDegPerSec = 0.0
    finalRawCmdDegPerSec = 0.0
    finalSafeCmdDegPerSec = 0.0

    while sw.time() < RUN_DURATION_MS:
        if CenterPressed(hub):
            abortedByButton = True
            break

        timeSec = sw.time() / 1000.0
        measurement = MakeMeasurement(hub, leftMotor, rightMotor, rightAuxMotor, timeSec, config)
        state = estimator.Update(measurement)
        rawCommand = controller.Compute(state)
        safeCommand = safety.Check(state, rawCommand, currentTime=timeSec)

        thetaDeg = RadToDeg(state.tilt)
        thetaDotDegPerSec = RadPerSecToDegPerSec(state.tiltRate)
        phiDeg = RadToDeg(state.phi)
        phiDotDegPerSec = RadPerSecToDegPerSec(state.phiDot)
        rawCmdDegPerSec = RadPerSecToDegPerSec(rawCommand.leftCommand)
        safeCmdDegPerSec = RadPerSecToDegPerSec(safeCommand.leftCommand)

        UpdateEffects(
            effectsState,
            config.effects,
            hub,
            thetaDeg,
            phiDeg,
            sw.time(),
        )

        finalThetaDeg = thetaDeg
        finalThetaDotDegPerSec = thetaDotDegPerSec
        finalPhiDeg = phiDeg
        finalPhiDotDegPerSec = phiDotDegPerSec
        finalRawCmdDegPerSec = rawCmdDegPerSec
        finalSafeCmdDegPerSec = safeCmdDegPerSec

        if writeIndex < bufferCapacity:
            WriteFloat(timeBuf, writeIndex, timeSec)
            WriteFloat(thetaRefBuf, writeIndex, thetaRefDeg)
            WriteFloat(thetaBuf, writeIndex, thetaDeg)
            WriteFloat(thetaDotBuf, writeIndex, thetaDotDegPerSec)
            WriteFloat(phiBuf, writeIndex, phiDeg)
            WriteFloat(phiDotBuf, writeIndex, phiDotDegPerSec)
            WriteFloat(rawCmdBuf, writeIndex, rawCmdDegPerSec)
            WriteFloat(safeCmdBuf, writeIndex, safeCmdDegPerSec)
            statusBuf[writeIndex] = StatusCode(safety.status)
            writeIndex += 1

        if safety.status.tripped and not previousTripped:
            # One short live marker is fine; this only fires on a trip.
            print(">>> SAFETY TRIPPED  : " + str(safety.status.reasons))
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
    ShutdownEffects(effectsState, hub)

    if abortedByButton:
        print("CENTER BUTTON pressed, aborting balance run.")

    print("============================================================")
    print(" HubPackageBalanceBuffered RUN DONE")
    print("   total iterations : " + str(iteration))
    print("   buffered samples : " + str(writeIndex) + " / " + str(bufferCapacity))
    print("   total time       : " + "{:.2f}".format(sw.time() / 1000.0) + " s")
    print("   final theta      : " + "{:+.2f}".format(finalThetaDeg) + " deg")
    print("   final theta dot  : " + "{:+.2f}".format(finalThetaDotDegPerSec) + " deg/s")
    print("   final phi        : " + "{:+.2f}".format(finalPhiDeg) + " deg")
    print("   final phi dot    : " + "{:+.2f}".format(finalPhiDotDegPerSec) + " deg/s")
    print("   final raw cmd    : " + "{:+.2f}".format(finalRawCmdDegPerSec) + " deg/s")
    print("   final safe cmd   : " + "{:+.2f}".format(finalSafeCmdDegPerSec) + " deg/s")
    print("   safety armed     : " + str(safety.status.armed))
    print("   safety tripped   : " + str(safety.status.tripped))
    if safety.status.reasons:
        print("   safety reasons   : " + str(safety.status.reasons))
    print("============================================================")

    # Let BLE settle before the bulk dump so the first rows are not lost.
    wait(200)
    DumpBuffer(
        writeIndex,
        timeBuf,
        thetaRefBuf,
        thetaBuf,
        thetaDotBuf,
        phiBuf,
        phiDotBuf,
        rawCmdBuf,
        safeCmdBuf,
        statusBuf,
    )
    print("============================================================")
    print(" BUFFER DUMP COMPLETE")
    print("   diagnostic plot : python scripts/PlotHubPackageBalanceBuffered.py")
    print("============================================================")


Main()
