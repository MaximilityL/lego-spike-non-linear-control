"""HubBluetoothTest.py

Bluetooth terminal smoke test for a SPIKE Prime hub running Pybricks.

Run this through Pybricks Code or ``pybricksdev run ble``. If you can see
the ping lines arrive in the terminal, the Bluetooth upload/run/print path
is working. The hub light also changes color on each ping so you have a
local heartbeat even if you are watching the hub instead of the terminal.

This program does not command any motors. It is safe to run with the robot
on the ground.

How to run:

    Browser: open code.pybricks.com, paste this file, click run.
    CLI:     pybricksdev run ble hub/HubBluetoothTest.py
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color
from pybricks.tools import StopWatch, wait

TEST_DURATION_MS = 15000
LOOP_PERIOD_MS = 50
PING_PERIOD_MS = 1000


def CenterPressed(hub):
    return Button.CENTER in hub.buttons.pressed()


def SetHeartbeatLight(hub, pingCount):
    if pingCount % 2 == 0:
        hub.light.on(Color.GREEN)
    else:
        hub.light.on(Color.BLUE)


def Main():
    hub = PrimeHub()
    sw = StopWatch()
    nextPingMs = 0
    pingCount = 0

    print("HubBluetoothTest starting.")
    print("Watch for one ping per second in this terminal.")
    print("Press center to stop early; otherwise the test runs for 15 seconds.")

    while sw.time() < TEST_DURATION_MS:
        if CenterPressed(hub):
            print("Center button pressed; stopping early.")
            break

        nowMs = sw.time()
        if nowMs >= nextPingMs:
            SetHeartbeatLight(hub, pingCount)
            print("bluetooth ping {} at {:.1f}s".format(pingCount, nowMs / 1000.0))
            pingCount += 1
            nextPingMs += PING_PERIOD_MS

        wait(LOOP_PERIOD_MS)

    hub.light.on(Color.GREEN)
    print("HubBluetoothTest done.")
    print("PASS if the terminal showed the ping lines without long gaps.")
    wait(500)
    hub.light.off()


Main()
