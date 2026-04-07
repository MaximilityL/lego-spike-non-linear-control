"""HubMotorTest.py

Safe motor smoke test. Spins both motors gently in alternating directions
for a fixed number of cycles, then stops. Designed to confirm:

- both motors are connected on the expected ports
- both motors respond to commands
- the encoder readings change as the motors move
- the center button still stops the program

SAFETY: block the wheels or hold the robot in your hand before running this
program. The default rate is small but the robot will still try to roll on
the floor.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

# ---------------------------------------------------------------------------
# Configuration. Mirror configs/Default.yaml.
# ---------------------------------------------------------------------------
LEFT_PORT = Port.A
RIGHT_PORT = Port.B
GENTLE_SPEED_DPS = 90       # 90 deg/s = 0.25 rev/s. Very gentle.
LEG_DURATION_MS = 800       # spin in one direction for this long
LEG_COUNT = 4               # forward, back, forward, back
LOOP_PERIOD_MS = 20


def CenterPressed(hub):
    return Button.CENTER in hub.buttons.pressed()


def DriveBoth(leftMotor, rightMotor, speedDps):
    leftMotor.run(speedDps)
    rightMotor.run(speedDps)


def StopBoth(leftMotor, rightMotor):
    leftMotor.stop()
    rightMotor.stop()


def Main():
    hub = PrimeHub()
    leftMotor = Motor(LEFT_PORT)
    rightMotor = Motor(RIGHT_PORT)

    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)

    print("HubMotorTest starting. Block the wheels first.")
    print("Center button stops the program at any time.")
    wait(1500)  # give you time to react if you forgot

    sw = StopWatch()

    for leg in range(LEG_COUNT):
        if CenterPressed(hub):
            break
        direction = 1 if (leg % 2 == 0) else -1
        speed = direction * GENTLE_SPEED_DPS
        print("leg {}: speed {} deg/s".format(leg, speed))
        DriveBoth(leftMotor, rightMotor, speed)

        legEndMs = sw.time() + LEG_DURATION_MS
        while sw.time() < legEndMs:
            if CenterPressed(hub):
                break
            wait(LOOP_PERIOD_MS)

    StopBoth(leftMotor, rightMotor)
    leftMotor.brake()
    rightMotor.brake()

    finalLeft = leftMotor.angle()
    finalRight = rightMotor.angle()
    print("HubMotorTest done. left={} deg, right={} deg".format(finalLeft, finalRight))


Main()
