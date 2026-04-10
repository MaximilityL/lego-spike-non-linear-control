"""HubDriveSmoke.py

Pre balancing forward / backward / stop smoke test for the SPIKE Prime hub
under Pybricks. This is the hub side counterpart of
``examples/DriveCommandSmoke.py``.

What this script does:

1. Reads the IMU pitch and the angular velocity, applying the same sign
   convention used by ``configs/Default.yaml``.
2. Reads the axle motor encoder angles and rates, applies the encoder signs,
   and computes the implemented wheel motion state ``phi`` and ``phiDot``
   from the left motor plus the averaged right-side motor pair.
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
RIGHT_PORT = Port.F
RIGHT_AUX_PORT = Port.D
LOOP_PERIOD_MS = 20            # 50 Hz, comfortable on the hub
PRINT_EVERY_N = 5              # print one telemetry line out of every N iterations

# IMU configuration from Default.yaml
TILT_SIGN = 1
ZERO_OFFSET_DEG = -91.445
GYRO_BIAS_DEG_PER_SEC = 0.0

# Encoder/forward signs from Default.yaml
FORWARD_SIGN = -1
LEFT_ENCODER_SIGN = 1
RIGHT_ENCODER_SIGN = -1
RIGHT_AUX_ENCODER_SIGN = -1

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
    print(f"   loop period         : {LOOP_PERIOD_MS} ms")
    print(f"   telemetry every     : {PRINT_EVERY_N} loop(s)")
    print(f"   drive test speed    : {DRIVE_TEST_SPEED_DPS} deg/s")
    print(f"   max tilt for motion : {MAX_TILT_FOR_MOTION_DEG} deg")
    print(f"   tilt sign           : {TILT_SIGN}")
    print(f"   zero offset         : {ZERO_OFFSET_DEG} deg")
    print(f"   gyro bias           : {GYRO_BIAS_DEG_PER_SEC} deg/s")
    print(f"   forward sign        : {FORWARD_SIGN}")
    print(f"   left encoder sign   : {LEFT_ENCODER_SIGN}")
    print(f"   right encoder sign  : {RIGHT_ENCODER_SIGN}")
    print("   right aux port      : D")
    print(f"   right aux enc sign  : {RIGHT_AUX_ENCODER_SIGN}")
    print(" Schedule:")
    totalMs = 0
    for entry in DRIVE_SCHEDULE:
        print(f"   {entry[0]:>8s} for {entry[2]:>5d} ms at {entry[1]:+6.1f} deg/s"
        )
        totalMs += entry[2]
    print(f"   total run time      : {totalMs} ms")
    print(" Post-run diagnostic plot:")
    print("   python scripts/PlotHubDriveSmoke.py")
    print("============================================================")


def Main():
    hub = PrimeHub()
    leftMotor = Motor(LEFT_PORT)
    rightMotor = Motor(RIGHT_PORT)
    rightAuxMotor = Motor(RIGHT_AUX_PORT)

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    rightAuxMotor.reset_angle(0)

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
        "phi_deg,phi_dot_deg_per_sec,left_cmd_deg_per_sec,right_cmd_deg_per_sec,"
        "gate,right_aux_angle_deg,right_aux_rate_deg_per_sec"
    )

    for leg in DRIVE_SCHEDULE:
        legLabel = leg[0]
        legSpeedDps = leg[1]
        legDurationMs = leg[2]

        legStartMs = sw.time()
        legEndMs = legStartMs + legDurationMs
        print("------------------------------------------------------------")
        print(
            f"LEG START : '{legLabel}' wanted {legSpeedDps:+.1f} deg/s for {legDurationMs} ms (t={legStartMs / 1000.0:.2f} s)"
        )
        print("------------------------------------------------------------")

        while sw.time() < legEndMs:
            if CenterPressed(hub):
                print("CENTER BUTTON pressed, aborting smoke flow.")
                leftMotor.stop()
                rightMotor.stop()
                rightAuxMotor.stop()
                return

            # ----- Read IMU. -----
            _, rollDeg = hub.imu.tilt()
            gx, _, _ = hub.imu.angular_velocity()
            thetaDeg = TILT_SIGN * rollDeg + ZERO_OFFSET_DEG
            thetaDotDegPerSec = TILT_SIGN * gx - GYRO_BIAS_DEG_PER_SEC

            # ----- Read encoders, compute phi / phiDot. -----
            leftAngleDeg = leftMotor.angle()
            rightAngleDeg = rightMotor.angle()
            rightAuxAngleDeg = rightAuxMotor.angle()
            leftSpeedDps = leftMotor.speed()
            rightSpeedDps = rightMotor.speed()
            rightAuxSpeedDps = rightAuxMotor.speed()

            signedLeftAngleDeg = LEFT_ENCODER_SIGN * leftAngleDeg
            signedRightAngleDeg = (
                RIGHT_ENCODER_SIGN * rightAngleDeg
                + RIGHT_AUX_ENCODER_SIGN * rightAuxAngleDeg
            ) / 2.0
            signedLeftSpeedDps = LEFT_ENCODER_SIGN * leftSpeedDps
            signedRightSpeedDps = (
                RIGHT_ENCODER_SIGN * rightSpeedDps
                + RIGHT_AUX_ENCODER_SIGN * rightAuxSpeedDps
            ) / 2.0
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
                        f">>> START OVER TILT : |theta|={tiltMagnitudeDeg:.2f} deg > limit {MAX_TILT_FOR_MOTION_DEG:.2f} deg, "
                        "motion gated"
                    )
                else:
                    print(
                        f">>> START SAFE      : |theta|={tiltMagnitudeDeg:.2f} deg <= limit {MAX_TILT_FOR_MOTION_DEG:.2f} deg, "
                        "motion allowed"
                    )
            elif gated and not previousGated:
                print(
                    f">>> OVER TILT       : |theta|={tiltMagnitudeDeg:.2f} deg > limit {MAX_TILT_FOR_MOTION_DEG:.2f} deg, "
                    "MOTION GATED, command forced to 0"
                )
            elif (not gated) and previousGated:
                print(
                    f">>> BACK IN SAFE    : |theta|={tiltMagnitudeDeg:.2f} deg <= limit {MAX_TILT_FOR_MOTION_DEG:.2f} deg, "
                    "motion allowed again"
                )
            previousGated = gated

            # Mirror the motor command sign for the motor whose encoder is
            # also mirrored, so positive commandDps means forward motion.
            leftMotor.run(FORWARD_SIGN * LEFT_ENCODER_SIGN * commandDps)
            rightMotor.run(FORWARD_SIGN * RIGHT_ENCODER_SIGN * commandDps)
            rightAuxMotor.run(FORWARD_SIGN * RIGHT_AUX_ENCODER_SIGN * commandDps)

            if iteration % PRINT_EVERY_N == 0:
                gateLabel = "GATED" if gated else "SAFE"
                print(
                    f"DATA,{sw.time() / 1000.0:.3f},{legLabel},{thetaDeg:+.2f},{thetaDotDegPerSec:+.2f},{phiDeg:+.2f},{phiDotDegPerSec:+.2f},{commandDps:+.2f},{commandDps:+.2f},{gateLabel},{RIGHT_AUX_ENCODER_SIGN * rightAuxAngleDeg:+.2f},{RIGHT_AUX_ENCODER_SIGN * rightAuxSpeedDps:+.2f}"
                )

            iteration += 1
            wait(LOOP_PERIOD_MS)

        print(
            f"LEG END   : '{legLabel}' done at t={sw.time() / 1000.0:.2f} s"
        )

    leftMotor.stop()
    rightMotor.stop()
    rightAuxMotor.stop()
    leftMotor.brake()
    rightMotor.brake()
    rightAuxMotor.brake()

    totalIterations = safeIterations + gatedIterations
    if totalIterations <= 0:
        gatedPct = 0.0
    else:
        gatedPct = 100.0 * gatedIterations / totalIterations

    print("============================================================")
    print(" HubDriveSmoke DONE")
    print(f"   total iterations : {totalIterations}")
    print(f"   safe iterations  : {safeIterations}")
    print(f"   gated iterations : {gatedIterations}  ({gatedPct:.1f} %)")
    print(f"   total time       : {sw.time() / 1000.0:.2f} s")
    print(f"   final theta      : {thetaDeg:+.2f} deg")
    print(f"   final theta dot  : {thetaDotDegPerSec:+.2f} deg/s")
    print(f"   final phi        : {phiDeg:+.2f} deg")
    print(f"   final phi dot    : {phiDotDegPerSec:+.2f} deg/s")
    print("   diagnostic plot  : python scripts/PlotHubDriveSmoke.py")
    print("============================================================")


Main()
