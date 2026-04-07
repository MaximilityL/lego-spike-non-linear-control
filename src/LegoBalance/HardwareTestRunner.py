"""HardwareTestRunner.

Coordinates a small fixed list of hardware oriented sanity tests, mostly so
that the application code does not have to repeat the same boilerplate. The
runner takes a hub plus the two motors and walks through them in a fixed
order. Each test is independent and can be skipped.

The runner does not import from ``pybricks``. It uses the abstract
interfaces and so it can be exercised against the mock adapters in unit
tests, then re used unchanged on the hub by handing it real adapters.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field

from .HubInterface import HubInterface
from .ImuInterface import ImuInterface
from .MotorInterface import MotorInterface
from .Saturation import SaturateSymmetric


@dataclass
class HardwareTestResult:
    name: str
    passed: bool
    detail: str = ""


@dataclass
class HardwareTestReport:
    results: list[HardwareTestResult] = field(default_factory=list)

    @property
    def Passed(self) -> bool:
        return all(r.passed for r in self.results)

    def Summary(self) -> str:
        lines = []
        for r in self.results:
            mark = "PASS" if r.passed else "FAIL"
            line = f"  [{mark}] {r.name}"
            if r.detail:
                line += f"  {r.detail}"
            lines.append(line)
        lines.append("")
        lines.append(f"Overall: {'PASS' if self.Passed else 'FAIL'}")
        return "\n".join(lines)


SleepFn = Callable[[float], None]


class HardwareTestRunner:
    """Run a fixed sequence of small hardware checks."""

    def __init__(
        self,
        hub: HubInterface,
        leftMotor: MotorInterface,
        rightMotor: MotorInterface,
        imu: ImuInterface | None = None,
        sleepFn: SleepFn = time.sleep,
    ) -> None:
        self.hub = hub
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.imu = imu
        self.sleepFn = sleepFn

    def Run(self, gentleVelocityRadPerSec: float = 1.0) -> HardwareTestReport:
        """Execute all tests and return a populated report.

        ``gentleVelocityRadPerSec`` is the small wheel velocity used in the
        motor smoke test. It is clipped to ``+/- 3 rad/s`` here as a hard
        upper bound regardless of what the caller passed in.
        """
        report = HardwareTestReport()
        gentle = SaturateSymmetric(gentleVelocityRadPerSec, 3.0)

        report.results.append(self._TestHubConnected())
        report.results.append(self._TestEncoderReadable())
        report.results.append(self._TestImuReadable())
        report.results.append(self._TestMotorSpinSafely(gentle))
        report.results.append(self._TestMotorStop())
        return report

    # ----- Individual checks. -----
    def _TestHubConnected(self) -> HardwareTestResult:
        try:
            connected = self.hub.IsConnected()
        except Exception as exc:
            return HardwareTestResult("hub connected", False, f"raised: {exc!r}")
        return HardwareTestResult(
            "hub connected",
            bool(connected),
            "OK" if connected else "hub.IsConnected returned False",
        )

    def _TestEncoderReadable(self) -> HardwareTestResult:
        try:
            leftAngle = self.leftMotor.Angle()
            rightAngle = self.rightMotor.Angle()
        except Exception as exc:
            return HardwareTestResult("encoder readable", False, f"raised: {exc!r}")
        return HardwareTestResult(
            "encoder readable",
            True,
            f"left={leftAngle:.3f} rad, right={rightAngle:.3f} rad",
        )

    def _TestImuReadable(self) -> HardwareTestResult:
        if self.imu is None:
            return HardwareTestResult("imu readable", True, "skipped (no imu given)")
        try:
            tilt = self.imu.TiltAngleRadians()
            rate = self.imu.TiltRateRadiansPerSec()
        except Exception as exc:
            return HardwareTestResult("imu readable", False, f"raised: {exc!r}")
        return HardwareTestResult(
            "imu readable",
            True,
            f"tilt={tilt:.3f} rad, rate={rate:.3f} rad/s",
        )

    def _TestMotorSpinSafely(self, gentleVelocityRadPerSec: float) -> HardwareTestResult:
        try:
            self.leftMotor.RunVelocity(gentleVelocityRadPerSec)
            self.rightMotor.RunVelocity(gentleVelocityRadPerSec)
            self.sleepFn(0.5)
        except Exception as exc:
            return HardwareTestResult("motor gentle spin", False, f"raised: {exc!r}")
        return HardwareTestResult(
            "motor gentle spin",
            True,
            f"both motors at {gentleVelocityRadPerSec:.2f} rad/s for 0.5 s",
        )

    def _TestMotorStop(self) -> HardwareTestResult:
        try:
            self.leftMotor.Stop()
            self.rightMotor.Stop()
        except Exception as exc:
            return HardwareTestResult("motor stop", False, f"raised: {exc!r}")
        return HardwareTestResult("motor stop", True, "both motors released")
