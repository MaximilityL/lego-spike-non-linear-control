"""HubEncoderTest.py

Stream encoder data from both drive motors to the Pybricks Code terminal.
The motors are not commanded. Move the wheels by hand and watch the angles
change. Useful for:

- verifying that both motors are detected
- checking that the encoder direction matches your sign convention
- making sure the connectors are seated

Safe to run anywhere.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

LEFT_PORT = Port.A
RIGHT_PORT = Port.B
LOOP_PERIOD_MS = 50
PRINT_EVERY_N = 1


def Main():
    hub = PrimeHub()
    leftMotor = Motor(LEFT_PORT)
    rightMotor = Motor(RIGHT_PORT)

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)

    sw = StopWatch()
    nextTickMs = LOOP_PERIOD_MS
    iteration = 0

    print("HubEncoderTest. Move the wheels by hand. Press center button to stop.")
    print("columns: t, leftAngleDeg, leftSpeedDps, rightAngleDeg, rightSpeedDps")

    while True:
        if Button.CENTER in hub.buttons.pressed():
            break

        la = leftMotor.angle()
        ls = leftMotor.speed()
        ra = rightMotor.angle()
        rs = rightMotor.speed()

        if iteration % PRINT_EVERY_N == 0:
            print(
                "{:.2f},{:+d},{:+d},{:+d},{:+d}".format(
                    sw.time() / 1000.0, int(la), int(ls), int(ra), int(rs)
                )
            )

        iteration += 1
        while sw.time() < nextTickMs:
            wait(1)
        nextTickMs += LOOP_PERIOD_MS

    print("HubEncoderTest stopped.")


Main()
