"""HubDriveSmoke.py

Pre balancing forward / backward / stop smoke test for the SPIKE Prime hub
under Pybricks. This is the hub side counterpart of
``examples/DriveCommandSmoke.py``.

What this script does:

1. Reads the IMU pitch and the angular velocity, applying the same sign
   convention used by ``configs/Default.yaml``.
2. Reads both motor encoder angles and rates, applies the encoder signs,
   and computes the implemented wheel motion state ``phi`` and ``phiDot``
   as the mean of the two.
3. Walks through a fixed schedule of validated wheel velocity commands:
   stop, forward, stop, backward, stop.
4. Refuses to issue motion commands if the body tilt has wandered past the
   pre balancing motion limit (``MAX_TILT_FOR_MOTION_DEG``).
5. Prints the implemented state ``[theta, thetaDot, phi, phiDot]`` (in
   degrees and deg/s for readability) on every iteration, and prints a
   loud message every time the drive gate flips between SAFE and OVER
   TILT so you always know which mode you are in.
6. Streams ``DATA`` rows that ``scripts/PlotHubDriveSmoke.py`` can turn into
   a post-run diagnostic plot of the estimator state.
7. Stops cleanly on the center button.

SAFETY: this script commands wheel motion. Block the wheels or hold the
robot in your hand the first time you run it. The default speed has been
validated on the real build, but the robot will still try to roll. The
Lyapunov balancing controller is NOT yet implemented. This script is a pre
balancing motion check, nothing more.

How to run:

    Browser: open code.pybricks.com, paste this file, click run.
    CLI:     pybricksdev run ble hub/HubDriveSmoke.py

Post-run diagnostic plot from your laptop:

    python scripts/PlotHubDriveSmoke.py
"""

# All imports must come from the pybricks namespace. Any other import will
# fail on the hub.
from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

# ---------------------------------------------------------------------------
# Configuration. Mirror configs/Default.yaml as closely as possible. The hub
# side cannot read YAML so the values are hard coded here. Update both files
# at the same time when you change them.
# ---------------------------------------------------------------------------
LEFT_PORT = Port.B
RIGHT_PORT = Port.A
LOOP_PERIOD_MS = 20            # 50 Hz, comfortable on the hub
PRINT_EVERY_N = 1              # print one telemetry line out of every N iterations

# IMU configuration from Default.yaml
TILT_SIGN = -1
ZERO_OFFSET_DEG = -60.0
GYRO_BIAS_DEG_PER_SEC = 0.0

# Encoder/forward signs from Default.yaml
FORWARD_SIGN = -1
LEFT_ENCODER_SIGN = 1
RIGHT_ENCODER_SIGN = -1

# Drive command magnitudes (mirror configs/Default.yaml drive section).
DRIVE_TEST_SPEED_DPS = 1000.0
MAX_TILT_FOR_MOTION_DEG = 50.0

# Schedule: list of (label, signed speed in deg/s, duration ms).
STOP_DURATION_MS = 200
DRIVE_DURATION_MS = 3000

DRIVE_SCHEDULE = (
    ("stop",      0.0,                        STOP_DURATION_MS),
    ("forward",  +DRIVE_TEST_SPEED_DPS,      DRIVE_DURATION_MS),
    ("stop",      0.0,                        STOP_DURATION_MS),
    ("backward", -DRIVE_TEST_SPEED_DPS,      DRIVE_DURATION_MS),
    ("stop",      0.0,                        STOP_DURATION_MS),
)


def CenterPressed(hub):
    return Button.CENTER in hub.buttons.pressed()


def PrintBanner():
    print("============================================================")
    print(" HubDriveSmoke : PRE BALANCING smoke test")
    print(" This script commands wheel motion. Lift or block the wheels.")
    print(" Press the center button at any time to stop cleanly.")
    print("============================================================")
    print(" Config:")
    print("   loop period         : {} ms".format(LOOP_PERIOD_MS))
    print("   telemetry every     : {} loop(s)".format(PRINT_EVERY_N))
    print("   drive test speed    : {} deg/s".format(DRIVE_TEST_SPEED_DPS))
    print("   max tilt for motion : {} deg".format(MAX_TILT_FOR_MOTION_DEG))
    print("   tilt sign           : {}".format(TILT_SIGN))
    print("   zero offset         : {} deg".format(ZERO_OFFSET_DEG))
    print("   gyro bias           : {} deg/s".format(GYRO_BIAS_DEG_PER_SEC))
    print("   forward sign        : {}".format(FORWARD_SIGN))
    print("   left encoder sign   : {}".format(LEFT_ENCODER_SIGN))
    print("   right encoder sign  : {}".format(RIGHT_ENCODER_SIGN))
    print(" Schedule:")
    totalMs = 0
    for entry in DRIVE_SCHEDULE:
        print("   {:>8s} for {:>5d} ms at {:+6.1f} deg/s".format(
            entry[0], entry[2], entry[1])
        )
        totalMs += entry[2]
    print("   total run time      : {} ms".format(totalMs))
    print(" Post-run diagnostic plot:")
    print("   python scripts/PlotHubDriveSmoke.py")
    print("============================================================")


def Main():
    hub = PrimeHub()
    leftMotor = Motor(LEFT_PORT)
    rightMotor = Motor(RIGHT_PORT)

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)

    PrintBanner()
    print("Starting in 1.5 s. Hold the robot or block the wheels NOW.")
    wait(1500)

    sw = StopWatch()
    iteration = 0

    # Counters used in the final summary so the user knows how often the
    # gate fired across the run.
    safeIterations = 0
    gatedIterations = 0
    # Track the previous gate state so we can announce transitions
    # loudly. Start as None so the very first iteration always prints
    # the current mode.
    previousGated = None

    print(
        "DATA_HEADER,t_s,leg,theta_deg,theta_dot_deg_per_sec,"
        "phi_deg,phi_dot_deg_per_sec,left_cmd_deg_per_sec,right_cmd_deg_per_sec,gate"
    )

    for leg in DRIVE_SCHEDULE:
        legLabel = leg[0]
        legSpeedDps = leg[1]
        legDurationMs = leg[2]

        legStartMs = sw.time()
        legEndMs = legStartMs + legDurationMs
        print("------------------------------------------------------------")
        print(
            "LEG START : '{}' wanted {:+.1f} deg/s for {} ms (t={:.2f} s)".format(
                legLabel, legSpeedDps, legDurationMs, legStartMs / 1000.0
            )
        )
        print("------------------------------------------------------------")

        while sw.time() < legEndMs:
            if CenterPressed(hub):
                print("CENTER BUTTON pressed, aborting smoke flow.")
                leftMotor.stop()
                rightMotor.stop()
                return

            # ----- Read IMU. -----
            pitchDeg, _ = hub.imu.tilt()
            _, gy, _ = hub.imu.angular_velocity()
            thetaDeg = TILT_SIGN * pitchDeg + ZERO_OFFSET_DEG
            thetaDotDegPerSec = TILT_SIGN * gy - GYRO_BIAS_DEG_PER_SEC

            # ----- Read encoders, compute phi / phiDot. -----
            leftAngleDeg = leftMotor.angle()
            rightAngleDeg = rightMotor.angle()
            leftSpeedDps = leftMotor.speed()
            rightSpeedDps = rightMotor.speed()

            signedLeftAngleDeg = LEFT_ENCODER_SIGN * leftAngleDeg
            signedRightAngleDeg = RIGHT_ENCODER_SIGN * rightAngleDeg
            signedLeftSpeedDps = LEFT_ENCODER_SIGN * leftSpeedDps
            signedRightSpeedDps = RIGHT_ENCODER_SIGN * rightSpeedDps
            phiDeg = FORWARD_SIGN * (signedLeftAngleDeg + signedRightAngleDeg) / 2.0
            phiDotDegPerSec = (
                FORWARD_SIGN * (signedLeftSpeedDps + signedRightSpeedDps) / 2.0
            )

            # ----- Drive gate: refuse motion if tilt is too large. -----
            tiltMagnitudeDeg = thetaDeg if thetaDeg >= 0.0 else -thetaDeg
            gated = tiltMagnitudeDeg > MAX_TILT_FOR_MOTION_DEG
            commandDps = 0.0 if gated else legSpeedDps

            if gated:
                gatedIterations += 1
            else:
                safeIterations += 1

            # ----- Loud transition messages. -----
            #
            # The very first iteration prints the current mode unconditionally
            # so the operator immediately sees whether the body started in
            # the safe envelope. After that, only transitions are announced
            # so the terminal is not flooded.
            if previousGated is None:
                if gated:
                    print(
                        ">>> START OVER TILT : |theta|={:.2f} deg > limit {:.2f} deg, "
                        "motion gated".format(tiltMagnitudeDeg, MAX_TILT_FOR_MOTION_DEG)
                    )
                else:
                    print(
                        ">>> START SAFE      : |theta|={:.2f} deg <= limit {:.2f} deg, "
                        "motion allowed".format(tiltMagnitudeDeg, MAX_TILT_FOR_MOTION_DEG)
                    )
            elif gated and not previousGated:
                print(
                    ">>> OVER TILT       : |theta|={:.2f} deg > limit {:.2f} deg, "
                    "MOTION GATED, command forced to 0".format(
                        tiltMagnitudeDeg, MAX_TILT_FOR_MOTION_DEG
                    )
                )
            elif (not gated) and previousGated:
                print(
                    ">>> BACK IN SAFE    : |theta|={:.2f} deg <= limit {:.2f} deg, "
                    "motion allowed again".format(
                        tiltMagnitudeDeg, MAX_TILT_FOR_MOTION_DEG
                    )
                )
            previousGated = gated

            # Mirror the motor command sign for the motor whose encoder is
            # also mirrored, so positive commandDps means forward motion.
            leftMotor.run(FORWARD_SIGN * LEFT_ENCODER_SIGN * commandDps)
            rightMotor.run(FORWARD_SIGN * RIGHT_ENCODER_SIGN * commandDps)

            if iteration % PRINT_EVERY_N == 0:
                gateLabel = "GATED" if gated else "SAFE"
                print(
                    "DATA,{:.3f},{},{:+.2f},{:+.2f},{:+.2f},{:+.2f},{:+.2f},{:+.2f},{}".format(
                        sw.time() / 1000.0,
                        legLabel,
                        thetaDeg,
                        thetaDotDegPerSec,
                        phiDeg,
                        phiDotDegPerSec,
                        commandDps,
                        commandDps,
                        gateLabel,
                    )
                )

            iteration += 1
            wait(LOOP_PERIOD_MS)

        print(
            "LEG END   : '{}' done at t={:.2f} s".format(legLabel, sw.time() / 1000.0)
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
    print(" HubDriveSmoke DONE")
    print("   total iterations : {}".format(totalIterations))
    print("   safe iterations  : {}".format(safeIterations))
    print("   gated iterations : {}  ({:.1f} %)".format(gatedIterations, gatedPct))
    print("   total time       : {:.2f} s".format(sw.time() / 1000.0))
    print("   final theta      : {:+.2f} deg".format(thetaDeg))
    print("   final theta dot  : {:+.2f} deg/s".format(thetaDotDegPerSec))
    print("   final phi        : {:+.2f} deg".format(phiDeg))
    print("   final phi dot    : {:+.2f} deg/s".format(phiDotDegPerSec))
    print("   diagnostic plot  : python scripts/PlotHubDriveSmoke.py")
    print("============================================================")


Main()
