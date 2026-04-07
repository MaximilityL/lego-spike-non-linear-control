"""HubMain.py

Minimal Pybricks bring up program. Run this on the SPIKE Prime hub once you
have flashed Pybricks. It does the following:

1. Initializes the hub.
2. Initializes the two drive motors on ports A and B.
3. Reads the IMU pitch and the angular velocity.
4. Reads the encoder angles.
5. Prints one line per loop iteration to the Pybricks Code terminal.
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
LEFT_PORT = Port.A
RIGHT_PORT = Port.B
LOOP_PERIOD_MS = 20  # 50 Hz, comfortable on the hub for a sensor only loop
PRINT_EVERY_N = 5    # print one line out of every N iterations


def Main():
    # ----- Initialization -----
    hub = PrimeHub()
    leftMotor = Motor(LEFT_PORT)
    rightMotor = Motor(RIGHT_PORT)

    # Reset encoder origins so we start at zero.
    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)

    sw = StopWatch()
    nextTickMs = LOOP_PERIOD_MS
    iteration = 0

    print("HubMain ready. Press center button to stop.")

    # ----- Main loop -----
    while True:
        # Stop on center button.
        if Button.CENTER in hub.buttons.pressed():
            break

        # Read IMU. Pybricks returns degrees and deg/s.
        pitchDeg, rollDeg = hub.imu.tilt()
        gx, gy, gz = hub.imu.angular_velocity()

        # Read encoders. Pybricks returns degrees and deg/s.
        leftAngleDeg = leftMotor.angle()
        rightAngleDeg = rightMotor.angle()
        leftSpeedDps = leftMotor.speed()
        rightSpeedDps = rightMotor.speed()

        if iteration % PRINT_EVERY_N == 0:
            # Print every Nth iteration so the terminal is not flooded.
            print(
                "t={:.2f}s pitch={:+.1f} roll={:+.1f} gy={:+.1f} "
                "L={:+d}/{:+d} R={:+d}/{:+d}".format(
                    sw.time() / 1000.0,
                    pitchDeg,
                    rollDeg,
                    gy,
                    int(leftAngleDeg),
                    int(leftSpeedDps),
                    int(rightAngleDeg),
                    int(rightSpeedDps),
                )
            )

        iteration += 1
        # Sleep until the next tick.
        while sw.time() < nextTickMs:
            wait(1)
        nextTickMs += LOOP_PERIOD_MS

    # ----- Shutdown -----
    leftMotor.stop()
    rightMotor.stop()
    print("HubMain stopped cleanly.")


Main()
