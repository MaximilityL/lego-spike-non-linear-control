"""HubImuTest.py

Stream IMU data to the Pybricks Code terminal at a fixed rate. Useful for:

- visually verifying which IMU axis tracks body tilt on your build
- watching the gyro for bias and noise
- checking the sign convention before running motors

This program does not command the motors. It is safe to run with the wheels
on the ground.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button
from pybricks.tools import StopWatch, wait

LOOP_PERIOD_MS = 20  # 50 Hz
PRINT_EVERY_N = 5    # 10 Hz to the terminal


def Main():
    hub = PrimeHub()
    sw = StopWatch()
    nextTickMs = LOOP_PERIOD_MS
    iteration = 0

    print("HubImuTest streaming. Press center button to stop.")
    print("columns: t, pitch_deg, roll_deg, gx, gy, gz, ax, ay, az")

    while True:
        if Button.CENTER in hub.buttons.pressed():
            break

        pitchDeg, rollDeg = hub.imu.tilt()
        gx, gy, gz = hub.imu.angular_velocity()
        ax, ay, az = hub.imu.acceleration()

        if iteration % PRINT_EVERY_N == 0:
            print(
                "{:.2f},{:+.1f},{:+.1f},{:+.1f},{:+.1f},{:+.1f},{:+.0f},{:+.0f},{:+.0f}".format(
                    sw.time() / 1000.0,
                    pitchDeg,
                    rollDeg,
                    gx,
                    gy,
                    gz,
                    ax,
                    ay,
                    az,
                )
            )

        iteration += 1
        while sw.time() < nextTickMs:
            wait(1)
        nextTickMs += LOOP_PERIOD_MS

    print("HubImuTest stopped.")


Main()
