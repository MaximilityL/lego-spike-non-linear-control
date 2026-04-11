"""HubPackageBalanceCalibrationSweep.py

Automated single-parameter sweep to find the best imu.zeroOffset OR the
best imu.gyroBias for the balancer. Set ``SWEEP_PARAMETER`` below to
pick which parameter this run tunes. The other parameter is taken from
``configs/Default.yaml`` (via LegoBalance.HubDriveSmokeRuntime.DefaultConfig)
and left untouched for the whole run.

The hub keeps balancing continuously and the config is mutated live on
window boundaries, so the robot never stops moving between tests. No
per sample telemetry is sent back, only one summary line per window and
a final ranking once the sweep finishes.

Search strategy (per window, one phase only):

1. BASELINE: test delta=0, record baselineScore.
2. Bracket in alternating pairs: +1*step, -1*step, +2*step, -2*step, ...
3. After each complete bracket pair (k >= 2) check the last two pairs:
   - If both +(k-1) and +k beat baseline AND both -(k-1) and -k lost
     to baseline, commit to the + direction. The search switches to
     unidirectional climbing: +(k+1), +(k+2), ... until the phase cap.
   - Symmetric commit to the - direction.
   - Otherwise advance to bracket level k+1 and keep probing both sides.
4. If no direction gets confirmed, the bracket pattern just runs to the
   phase cap and the single best window wins regardless.

The state estimator reads ``config.imu.zeroOffset`` and
``config.imu.gyroBias`` on every iteration, so mutating the swept field
between windows is enough for the new calibration to take effect next
tick. The controller and safety monitor are built once up front and
keep running across every window. Phi drift is scored per window
relative to the phi value captured at window start so encoder drift
between windows does not bias later scores.

If the safety monitor trips (the robot actually falls) the sweep ends
immediately. Press the center button any time to abort cleanly.

To tune the OTHER parameter, edit SWEEP_PARAMETER below and rerun:

    # sweep zeroOffset with the current gyroBias
    SWEEP_PARAMETER = "zeroOffset"

    # sweep gyroBias with the current zeroOffset
    SWEEP_PARAMETER = "gyroBias"

Run it directly after regenerating the hub-safe config:

    python scripts/GenerateHubDriveSmokeRuntime.py
    pybricksdev run ble src/HubPackageBalanceCalibrationSweep.py

Or through the desktop launcher which handles the regenerate step:

    python scripts/RunHubPackageBalanceCalibrationSweep.py
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

# "zeroOffset" or "gyroBias". Only one parameter is swept per run; the
# other one is taken from configs/Default.yaml and left alone.
SWEEP_PARAMETER = "gyroBias"

START_DELAY_MS = 1500
# Each window is split into a stabilize phase (robot settles to the new
# balance point, no samples counted) and a score phase (only this half
# feeds the window score). This prevents the transient after a delta
# change from polluting the score.
STABILIZE_MS = 15000
SCORE_MS = 15000
WINDOW_TOTAL_MS = STABILIZE_MS + SCORE_MS
ZERO_OFFSET_STEP_RAD = 0.0035
ZERO_OFFSET_POINT_COUNT = 200
GYRO_BIAS_STEP_RAD_PER_SEC = 0.008
GYRO_BIAS_POINT_COUNT = 100
TILT_SCORE_WEIGHT = 1.0
PHI_SCORE_WEIGHT = 0.1
TRIPPED_PENALTY = 1000.0


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


def Fmt4(value):
    # Pybricks MicroPython supports basic format strings; four decimals
    # here keeps radians readable at the bench.
    return "{:+.4f}".format(value)


def Fmt2(value):
    return "{:+.2f}".format(value)


def PrintBanner(config, loopPeriodMs, sweepName, baselineValue, stepSize,
                maxWindows, valueUnit):
    approxMaxSec = maxWindows * (WINDOW_TOTAL_MS / 1000.0)
    print("============================================================")
    print(" HubPackageBalanceCalibrationSweep")
    print(" Single-parameter adaptive sweep.")
    print(" Tunes ONE of imu.zeroOffset or imu.gyroBias; the other is")
    print(" taken from configs/Default.yaml and left alone.")
    print(" Probes +k and -k alternately. Once two consecutive bracket")
    print(" levels agree that one direction wins, the search commits")
    print(" and climbs that direction to the phase cap.")
    print(" Press the center button any time to abort the sweep.")
    print("============================================================")
    print(" Sweeping parameter      : imu." + sweepName)
    print(" Baseline imu.zeroOffset : " + Fmt4(config.imu.zeroOffset) + " rad  ("
          + Fmt2(RadToDeg(config.imu.zeroOffset)) + " deg)")
    print(" Baseline imu.gyroBias   : " + Fmt4(config.imu.gyroBias) + " rad/s ("
          + Fmt2(RadPerSecToDegPerSec(config.imu.gyroBias)) + " deg/s)")
    print(" Sweep baseline value    : " + Fmt4(baselineValue) + " " + valueUnit)
    print(" Step size               : " + Fmt4(stepSize) + " " + valueUnit)
    print(" Phase window cap        : " + str(maxWindows) + " windows")
    print(" Loop period             : " + str(loopPeriodMs) + " ms")
    print(" Stabilize / score       : " + str(STABILIZE_MS) + " / "
          + str(SCORE_MS) + " ms")
    print(" Worst case balance run  : " + "{:.1f}".format(approxMaxSec) + " s")
    print(" Controller algorithm    : " + str(config.controller.algorithm))
    print("============================================================")


def WindowScore(meanAbsTiltDeg, meanAbsPhiDeg):
    return TILT_SCORE_WEIGHT * meanAbsTiltDeg + PHI_SCORE_WEIGHT * meanAbsPhiDeg


def PrintWindowSummary(testIndex, maxTests, stateLabel, zeroOffset, gyroBias,
                       meanAbsTiltDeg, meanAbsPhiDeg, score, elapsedSec, tripped):
    tag = "TRIPPED" if tripped else "OK"
    print(
        "[" + str(testIndex) + "/" + str(maxTests) + "] " + stateLabel
        + "  zo=" + Fmt4(zeroOffset) + " rad"
        + "  gb=" + Fmt4(gyroBias) + " rad/s"
        + "  " + tag
        + "  t=" + "{:.2f}".format(elapsedSec) + "s"
        + "  |tilt|=" + Fmt2(meanAbsTiltDeg) + "deg"
        + "  |phi|=" + Fmt2(meanAbsPhiDeg) + "deg"
        + "  score=" + "{:.3f}".format(score)
    )


def Main():
    if SWEEP_PARAMETER not in ("zeroOffset", "gyroBias"):
        raise ValueError("SWEEP_PARAMETER must be 'zeroOffset' or 'gyroBias'")

    config = DefaultConfig()
    loopPeriodMs = int(1000.0 / config.control.loopRate + 0.5)
    if loopPeriodMs < 1:
        loopPeriodMs = 1

    baselineZeroOffset = config.imu.zeroOffset
    baselineGyroBias = config.imu.gyroBias

    if SWEEP_PARAMETER == "zeroOffset":
        stepSize = ZERO_OFFSET_STEP_RAD
        maxWindows = ZERO_OFFSET_POINT_COUNT
        baselineValue = baselineZeroOffset
        valueUnit = "rad"
        paramShort = "zo"
    else:
        stepSize = GYRO_BIAS_STEP_RAD_PER_SEC
        maxWindows = GYRO_BIAS_POINT_COUNT
        baselineValue = baselineGyroBias
        valueUnit = "rad/s"
        paramShort = "gb"

    hub = PrimeHub()
    leftMotor = Motor(MotorPort(config.motors.leftPort))
    rightMotor = Motor(MotorPort(config.motors.rightPort))
    rightAuxMotor = None
    if HasRightAuxMotor(config):
        rightAuxMotor = Motor(MotorPort(config.motors.rightAuxPort))

    estimator = StateEstimator(config)
    controller = BuildBalanceController(config)
    safety = SafetyMonitor(config)

    PrintBanner(
        config, loopPeriodMs, SWEEP_PARAMETER, baselineValue, stepSize,
        maxWindows, valueUnit,
    )
    print("Starting in 1.5 s. Hold the robot near upright NOW.")
    wait(START_DELAY_MS)

    # Start at the baseline. The other imu field stays at its configured
    # value for the whole run.
    config.imu.zeroOffset = baselineZeroOffset
    config.imu.gyroBias = baselineGyroBias

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    if rightAuxMotor is not None:
        rightAuxMotor.reset_angle(0)
    estimator.Reset()
    controller.Reset()

    sw = StopWatch()
    safety.Arm(currentTime=0.0)
    nextTickMs = loopPeriodMs

    # Search state:
    # phaseState: BASELINE -> BRACKET_PLUS -> BRACKET_MINUS -> BRACKET_PLUS
    #             -> BRACKET_MINUS -> (commit) -> CLIMB_PLUS / CLIMB_MINUS
    phaseState = "BASELINE"
    bracketK = 0
    climbMultiplier = 0
    currentDelta = 0.0
    baselineScore = None
    plusScorePrev = None
    plusScoreCurr = None
    minusScorePrev = None
    minusScoreCurr = None
    phaseWindowsUsed = 0

    testIndex = 1
    windowStartMs = 0
    windowStartPhi = 0.0
    windowStartPhiInitialized = False

    winSumAbsTilt = 0.0
    winSumAbsPhi = 0.0
    winSampleCount = 0

    bestScore = None
    bestValue = baselineValue
    bestDelta = 0.0
    bestMeanTiltDeg = 0.0
    bestMeanPhiDeg = 0.0
    bestTripped = False

    abortedByButton = False
    sweepComplete = False

    print("------------------------------------------------------------")
    print(" Adaptive search on imu." + SWEEP_PARAMETER)
    print("------------------------------------------------------------")
    print(
        "[" + str(testIndex) + "/" + str(maxWindows) + "] baseline"
        + "  delta=" + Fmt4(currentDelta)
        + "  " + paramShort + "=" + Fmt4(baselineValue) + " " + valueUnit
    )

    while True:
        if CenterPressed(hub):
            abortedByButton = True
            break

        nowMs = sw.time()
        timeSec = nowMs / 1000.0
        measurement = MakeMeasurement(hub, leftMotor, rightMotor, rightAuxMotor, timeSec, config)
        state = estimator.Update(measurement)
        rawCommand = controller.Compute(state)
        safeCommand = safety.Check(state, rawCommand, currentTime=timeSec)

        windowElapsedMs = nowMs - windowStartMs
        inScoringPhase = windowElapsedMs >= STABILIZE_MS

        if inScoringPhase:
            if not windowStartPhiInitialized:
                windowStartPhi = state.phi
                windowStartPhiInitialized = True
            winSumAbsTilt += abs(state.tilt - config.control.targetTilt)
            winSumAbsPhi += abs(state.phi - windowStartPhi)
            winSampleCount += 1

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

        tripped = safety.status.tripped
        windowFull = windowElapsedMs >= WINDOW_TOTAL_MS

        if windowFull or tripped:
            if winSampleCount > 0:
                meanAbsTiltDeg = RadToDeg(winSumAbsTilt / winSampleCount)
                meanAbsPhiDeg = RadToDeg(winSumAbsPhi / winSampleCount)
            else:
                meanAbsTiltDeg = 0.0
                meanAbsPhiDeg = 0.0
            score = WindowScore(meanAbsTiltDeg, meanAbsPhiDeg)
            if tripped:
                if windowElapsedMs < STABILIZE_MS:
                    # Fell during stabilization: no credit.
                    score += TRIPPED_PENALTY
                else:
                    scoreElapsedMs = windowElapsedMs - STABILIZE_MS
                    survivedFraction = scoreElapsedMs / float(SCORE_MS)
                    if survivedFraction > 1.0:
                        survivedFraction = 1.0
                    score += TRIPPED_PENALTY * (1.0 - survivedFraction)
            elapsedSec = windowElapsedMs / 1000.0
            phaseWindowsUsed += 1

            if SWEEP_PARAMETER == "zeroOffset":
                currentValue = config.imu.zeroOffset
            else:
                currentValue = config.imu.gyroBias

            PrintWindowSummary(
                testIndex, maxWindows, phaseState.lower(),
                config.imu.zeroOffset, config.imu.gyroBias,
                meanAbsTiltDeg, meanAbsPhiDeg, score, elapsedSec, tripped,
            )
            if bestScore is None or score < bestScore:
                bestScore = score
                bestValue = currentValue
                bestDelta = currentDelta
                bestMeanTiltDeg = meanAbsTiltDeg
                bestMeanPhiDeg = meanAbsPhiDeg
                bestTripped = tripped

            if tripped:
                break

            phaseDone = False
            nextDelta = currentDelta

            if phaseState == "BASELINE":
                baselineScore = score
                bracketK = 1
                phaseState = "BRACKET_PLUS"
                nextDelta = bracketK * stepSize
            elif phaseState == "BRACKET_PLUS":
                plusScorePrev = plusScoreCurr
                plusScoreCurr = score
                phaseState = "BRACKET_MINUS"
                nextDelta = -bracketK * stepSize
            elif phaseState == "BRACKET_MINUS":
                minusScorePrev = minusScoreCurr
                minusScoreCurr = score

                # After each complete pair, check the last TWO pairs for
                # a consistent direction. That requires bracketK >= 2.
                commitPlus = False
                commitMinus = False
                if bracketK >= 2 and plusScorePrev is not None and minusScorePrev is not None:
                    plusWinsCurr = plusScoreCurr < baselineScore
                    plusWinsPrev = plusScorePrev < baselineScore
                    minusWinsCurr = minusScoreCurr < baselineScore
                    minusWinsPrev = minusScorePrev < baselineScore
                    if plusWinsCurr and plusWinsPrev and (not minusWinsCurr) and (not minusWinsPrev):
                        commitPlus = True
                    elif minusWinsCurr and minusWinsPrev and (not plusWinsCurr) and (not plusWinsPrev):
                        commitMinus = True

                if commitPlus:
                    phaseState = "CLIMB_PLUS"
                    climbMultiplier = bracketK + 1
                    nextDelta = climbMultiplier * stepSize
                    print("  -> + direction confirmed on two pairs, climbing +")
                elif commitMinus:
                    phaseState = "CLIMB_MINUS"
                    climbMultiplier = bracketK + 1
                    nextDelta = -climbMultiplier * stepSize
                    print("  -> - direction confirmed on two pairs, climbing -")
                else:
                    bracketK += 1
                    phaseState = "BRACKET_PLUS"
                    nextDelta = bracketK * stepSize
            elif phaseState == "CLIMB_PLUS":
                climbMultiplier += 1
                nextDelta = climbMultiplier * stepSize
            elif phaseState == "CLIMB_MINUS":
                climbMultiplier += 1
                nextDelta = -climbMultiplier * stepSize

            if not phaseDone and phaseWindowsUsed >= maxWindows:
                phaseDone = True
                print("  -> reached phase window cap, sweep done")

            if phaseDone:
                sweepComplete = True
                break

            currentDelta = nextDelta
            if SWEEP_PARAMETER == "zeroOffset":
                config.imu.zeroOffset = baselineZeroOffset + currentDelta
                newValue = config.imu.zeroOffset
            else:
                config.imu.gyroBias = baselineGyroBias + currentDelta
                newValue = config.imu.gyroBias
            testIndex += 1
            print(
                "[" + str(testIndex) + "/" + str(maxWindows) + "] " + phaseState.lower()
                + "  delta=" + Fmt4(currentDelta)
                + "  " + paramShort + "=" + Fmt4(newValue) + " " + valueUnit
            )

            windowStartMs = sw.time()
            windowStartPhiInitialized = False
            winSumAbsTilt = 0.0
            winSumAbsPhi = 0.0
            winSampleCount = 0

        remainingMs = nextTickMs - sw.time()
        if remainingMs > 0:
            wait(remainingMs)
        else:
            nextTickMs = sw.time()
        nextTickMs += loopPeriodMs

    StopMotors(leftMotor, rightMotor, rightAuxMotor)
    BrakeMotors(leftMotor, rightMotor, rightAuxMotor)

    print("============================================================")
    print(" CALIBRATION SWEEP DONE")
    print("   parameter swept     : imu." + SWEEP_PARAMETER)
    if abortedByButton:
        print("   aborted by center button at test " + str(testIndex) + "/" + str(maxWindows))
    elif sweepComplete:
        print("   completed phase (" + str(phaseWindowsUsed) + "/" + str(maxWindows) + " windows)")
    else:
        print("   stopped early (safety tripped) at test " + str(testIndex) + "/" + str(maxWindows))
        if safety.status.reasons:
            print("   trip reasons        : " + str(safety.status.reasons))
    print("   baseline zeroOffset : " + Fmt4(baselineZeroOffset) + " rad ("
          + Fmt2(RadToDeg(baselineZeroOffset)) + " deg)")
    print("   baseline gyroBias   : " + Fmt4(baselineGyroBias) + " rad/s ("
          + Fmt2(RadPerSecToDegPerSec(baselineGyroBias)) + " deg/s)")
    if bestScore is not None:
        if SWEEP_PARAMETER == "zeroOffset":
            print("   best imu.zeroOffset : " + Fmt4(bestValue) + " rad ("
                  + Fmt2(RadToDeg(bestValue)) + " deg)")
            print("     delta from base   : " + Fmt4(bestDelta) + " rad")
        else:
            print("   best imu.gyroBias   : " + Fmt4(bestValue) + " rad/s ("
                  + Fmt2(RadPerSecToDegPerSec(bestValue)) + " deg/s)")
            print("     delta from base   : " + Fmt4(bestDelta) + " rad/s")
        print("     score             : " + "{:.3f}".format(bestScore))
        print("     mean |tilt|       : " + Fmt2(bestMeanTiltDeg) + " deg")
        print("     mean |phi|        : " + Fmt2(bestMeanPhiDeg) + " deg")
        print("     tripped           : " + str(bestTripped))
    else:
        print("   best                : (no windows scored)")
    print(" Copy the best value into configs/Default.yaml then regenerate")
    print(" the hub runtime with scripts/GenerateHubDriveSmokeRuntime.py.")
    print(" To tune the other parameter, flip SWEEP_PARAMETER at the top")
    print(" of this file and rerun.")
    print("============================================================")


Main()
