"""ReadSensors.py

Tiny example that reads from the desktop side mock hub for a few iterations
and prints the values. The mock hub is used so this works without any
hardware. The structure mirrors what a hub side script would do.

Run with:

    python examples/ReadSensors.py
"""

from __future__ import annotations

import time

from LegoBalance.MockAdapters import MockHub


def Main() -> int:
    hub = MockHub()
    hub.SetInitialTilt(0.02)
    print("t,    pitch_deg, gy_dps, leftAngleDeg, rightAngleDeg")
    for _ in range(10):
        pitchDeg, _ = hub.TiltDegrees()
        _, gyDps, _ = hub.AngularVelocityDegreesPerSec()
        leftAngleRad = hub.LeftMotor.Angle()
        rightAngleRad = hub.RightMotor.Angle()
        print(
            f"{hub.Now():5.2f}, {pitchDeg:+8.2f}, {gyDps:+7.2f}, "
            f"{leftAngleRad:+12.4f}, {rightAngleRad:+12.4f}"
        )
        # The mock plant only advances when we call Step.
        hub.Step(0.05)
        time.sleep(0.01)
    hub.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
