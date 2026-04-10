"""HubMain.py

Minimal Pybricks bring up program. Run this on the SPIKE Prime hub once you
have flashed Pybricks. It does the following:

1. Initializes the hub.
2. Initializes the drive motors on ports B, F, and the optional right-side
   axle assist motor on D.
3. Reads the IMU roll and the angular velocity.
4. Reads the encoder angles.
5. Streams plot friendly telemetry rows to the Pybricks Code terminal.
6. Stops cleanly when the center button is pressed.

This file is intentionally self contained. Pybricks programs cannot import
from the desktop side ``LegoBalance`` package. The structure here mirrors
the desktop side abstractions on purpose so that porting future code from
the desktop is mechanical.

How to run:

    Browser: open code.pybricks.com, paste this file, click run.
    CLI:     pybricksdev run ble hub/HubMain.py

Safety: this program does not command the motors. It only reads sensors. It
is safe to run with the wheels on the ground.

For live plots from a laptop, run:

    python scripts/PlotHubMainLive.py
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
LOOP_PERIOD_MS = 20  # 50 Hz, comfortable on the hub for a sensor only loop
PRINT_EVERY_N = 1    # print one line out of every N iterations
WHEEL_RADIUS_M = 0.0285
FORWARD_SIGN = -1
LEFT_ENCODER_SIGN = 1
RIGHT_ENCODER_SIGN = -1
RIGHT_AUX_ENCODER_SIGN = -1

# IMU configuration from Default.yaml
TILT_SIGN = 1
ZERO_OFFSET_DEG = -92.5  # calibrated zero in degrees. This is a best effort number from the real robot. You may need to adjust it for your build. The hub IMU is not perfectly repeat
GYRO_BIAS_DEG_PER_SEC = 0.0
DEG_TO_RAD = 0.017453292519943295


def Main():
    # ----- Initialization -----
    hub = PrimeHub()
    leftMotor = Motor(LEFT_PORT)
    rightMotor = Motor(RIGHT_PORT)
    rightAuxMotor = Motor(RIGHT_AUX_PORT)

    # Reset encoder origins so we start at zero.
    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    rightAuxMotor.reset_angle(0)

    sw = StopWatch()
    nextTickMs = LOOP_PERIOD_MS
    iteration = 0

    print("HubMain ready. Press center button to stop.")
    print(
        "DATA_HEADER,t_s,theta_deg,theta_dot_deg_per_sec,p_m,p_dot_m_per_sec,"
        "left_angle_deg,left_rate_deg_per_sec,right_angle_deg,right_rate_deg_per_sec,"
        "right_aux_angle_deg,right_aux_rate_deg_per_sec"
    )

    # ----- Main loop -----
    while True:
        # Stop on center button.
        if Button.CENTER in hub.buttons.pressed():
            break

        # Read IMU. Pybricks returns degrees and deg/s.
        _, rollDeg = hub.imu.tilt()
        gx, _, _ = hub.imu.angular_velocity()

        # Apply IMU configuration corrections
        tiltDeg = TILT_SIGN * rollDeg + ZERO_OFFSET_DEG
        tiltRateDegPerSec = TILT_SIGN * gx - GYRO_BIAS_DEG_PER_SEC

        # Read encoders. Pybricks returns degrees and deg/s.
        leftAngleDeg = leftMotor.angle()
        rightAngleDeg = rightMotor.angle()
        rightAuxAngleDeg = rightAuxMotor.angle()
        leftSpeedDps = leftMotor.speed()
        rightSpeedDps = rightMotor.speed()
        rightAuxSpeedDps = rightAuxMotor.speed()

        if iteration % PRINT_EVERY_N == 0:
            # Print every Nth iteration so the terminal is not flooded. Lines
            # beginning with DATA are consumed by scripts/PlotHubMainLive.py.
            # If one motor is mirrored, flip its encoder sign here so p is
            # positive when the wheel base rolls forward along the floor.
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
            meanWheelAngleDeg = (signedLeftAngleDeg + signedRightAngleDeg) / 2.0
            meanWheelSpeedDps = (signedLeftSpeedDps + signedRightSpeedDps) / 2.0
            thetaDeg = tiltDeg
            thetaDotDegPerSec = tiltRateDegPerSec
            pM = FORWARD_SIGN * meanWheelAngleDeg * DEG_TO_RAD * WHEEL_RADIUS_M
            pDotMPerSec = FORWARD_SIGN * meanWheelSpeedDps * DEG_TO_RAD * WHEEL_RADIUS_M
            print(
                f"DATA,{sw.time() / 1000.0:.3f},{thetaDeg:.6f},{thetaDotDegPerSec:.6f},{pM:.6f},{pDotMPerSec:.6f},{signedLeftAngleDeg:.6f},{signedLeftSpeedDps:.6f},{signedRightAngleDeg:.6f},{signedRightSpeedDps:.6f},{RIGHT_AUX_ENCODER_SIGN * rightAuxAngleDeg:.6f},{RIGHT_AUX_ENCODER_SIGN * rightAuxSpeedDps:.6f}"
            )

        iteration += 1
        # Sleep until the next tick.
        while sw.time() < nextTickMs:
            wait(1)
        nextTickMs += LOOP_PERIOD_MS

    # ----- Shutdown -----
    leftMotor.stop()
    rightMotor.stop()
    rightAuxMotor.stop()
    print("HubMain stopped cleanly.")


Main()
