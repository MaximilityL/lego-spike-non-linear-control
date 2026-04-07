"""MotorSmokeTest.py

Desktop side dry run of the hardware test runner using mocks. This is the
hardware free counterpart to ``hub/HubMotorTest.py``. It is the right script
to run before you ever touch a real motor, to confirm that the test runner
itself is working.

Run with:

    python examples/MotorSmokeTest.py
"""

from __future__ import annotations

from LegoBalance.HardwareTestRunner import HardwareTestRunner
from LegoBalance.MockAdapters import MockHub


def Main() -> int:
    hub = MockHub()
    runner = HardwareTestRunner(
        hub=hub,
        leftMotor=hub.LeftMotor,
        rightMotor=hub.RightMotor,
        imu=hub.Imu,
        sleepFn=lambda _seconds: None,  # do not actually sleep in the dry run
    )
    report = runner.Run(gentleVelocityRadPerSec=1.0)
    print("LegoBalance: motor smoke test (mock)")
    print("-" * 60)
    print(report.Summary())
    return 0 if report.Passed else 1


if __name__ == "__main__":
    raise SystemExit(Main())
