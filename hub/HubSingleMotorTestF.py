"""HubSingleMotorTestF.py

Single motor smoke test for the motor on Port F. Use this when you only have
one motor connected and you want to confirm:

- the motor on Port F is detected
- the encoder readings change as the motor moves
- the motor responds to a gentle velocity command
- the center button still stops the program

The two motor versions of this test (HubMotorTest.py and HubMain.py) assume
ports A and B and are kept around for the eventual two wheel build. This file
is the current bring up test while only one motor is wired.

SAFETY: lift the motor or hold it in your hand before running. The default
rate is small but the motor will still try to spin.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

# ---------------------------------------------------------------------------
# Configuration. Single motor on Port F.
# ---------------------------------------------------------------------------
MOTOR_PORT = Port.F
GENTLE_SPEED_DPS = 90       # 90 deg/s = 0.25 rev/s. Very gentle.
LEG_DURATION_MS = 800       # spin in one direction for this long
LEG_COUNT = 4               # forward, back, forward, back
LOOP_PERIOD_MS = 20


def CenterPressed(hub):
    return Button.CENTER in hub.buttons.pressed()


def Main():
    hub = PrimeHub()
    motor = Motor(MOTOR_PORT)

    motor.reset_angle(0)

    print("HubSingleMotorTestF starting on Port F.")
    print("Lift the motor first. Center button stops at any time.")
    wait(1500)  # give you time to react if you forgot

    sw = StopWatch()

    for leg in range(LEG_COUNT):
        if CenterPressed(hub):
            break
        direction = 1 if (leg % 2 == 0) else -1
        speed = direction * GENTLE_SPEED_DPS
        print("leg {}: speed {} deg/s".format(leg, speed))
        motor.run(speed)

        legEndMs = sw.time() + LEG_DURATION_MS
        while sw.time() < legEndMs:
            if CenterPressed(hub):
                break
            wait(LOOP_PERIOD_MS)

        # Print encoder while moving so you can see it tracking.
        print("  angle={} deg, speed={} dps".format(int(motor.angle()), int(motor.speed())))

    motor.stop()
    motor.brake()

    finalAngle = motor.angle()
    print("HubSingleMotorTestF done. final angle = {} deg".format(finalAngle))


Main()
