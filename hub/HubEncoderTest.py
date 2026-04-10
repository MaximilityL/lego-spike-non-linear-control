"""HubEncoderTest.py

Stream encoder data from all axle motors to the Pybricks Code terminal.
The motors are not commanded. Move the wheels by hand and watch the angles
change. Useful for:

- verifying that all drive motors are detected
- checking that the encoder direction matches your sign convention
- making sure the connectors are seated

Safe to run anywhere.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

LEFT_PORT = Port.B
RIGHT_PORT = Port.F
RIGHT_AUX_PORT = Port.D
LOOP_PERIOD_MS = 50
PRINT_EVERY_N = 1


def Main():
    hub = PrimeHub()
    leftMotor = Motor(LEFT_PORT)
    rightMotor = Motor(RIGHT_PORT)
    rightAuxMotor = Motor(RIGHT_AUX_PORT)

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    rightAuxMotor.reset_angle(0)

    sw = StopWatch()
    nextTickMs = LOOP_PERIOD_MS
    iteration = 0

    print("HubEncoderTest. Move the wheels by hand. Press center button to stop.")
    print(
        "columns: t, leftAngleDeg, leftSpeedDps, rightAngleDeg, rightSpeedDps, "
        "rightAuxAngleDeg, rightAuxSpeedDps"
    )

    while True:
        if Button.CENTER in hub.buttons.pressed():
            break

        la = leftMotor.angle()
        ls = leftMotor.speed()
        ra = rightMotor.angle()
        rs = rightMotor.speed()
        raa = rightAuxMotor.angle()
        ras = rightAuxMotor.speed()

        if iteration % PRINT_EVERY_N == 0:
            print(
                f"{sw.time() / 1000.0:.2f},{int(la):+d},{int(ls):+d},{int(ra):+d},{int(rs):+d},{int(raa):+d},{int(ras):+d}"
            )

        iteration += 1
        while sw.time() < nextTickMs:
            wait(1)
        nextTickMs += LOOP_PERIOD_MS

    print("HubEncoderTest stopped.")


Main()
