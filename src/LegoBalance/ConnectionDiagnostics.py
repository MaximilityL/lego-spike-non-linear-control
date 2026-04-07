"""Connection diagnostics.

A small driver that walks through the basic checks you want to make before
trusting a hub. The checks themselves are trivial. The point is to have one
documented, testable place that says yes or no.

The diagnostics talk to the hub through the abstract :class:`HubInterface`,
so they can run against the real hardware adapter or against
:class:`LegoBalance.MockAdapters.MockHub` in tests.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .HubInterface import HubInterface


@dataclass
class CheckResult:
    """One named check and its outcome."""

    name: str
    passed: bool
    detail: str = ""


@dataclass
class DiagnosticsReport:
    """Aggregate result of one diagnostics run."""

    checks: list[CheckResult] = field(default_factory=list)

    @property
    def Passed(self) -> bool:
        return all(check.passed for check in self.checks)

    def Summary(self) -> str:
        lines = []
        for check in self.checks:
            mark = "PASS" if check.passed else "FAIL"
            line = f"  [{mark}] {check.name}"
            if check.detail:
                line += f"  {check.detail}"
            lines.append(line)
        lines.append("")
        lines.append(f"Overall: {'PASS' if self.Passed else 'FAIL'}")
        return "\n".join(lines)


class ConnectionDiagnostics:
    """Run a small fixed sequence of checks against a hub."""

    def __init__(self, hub: HubInterface) -> None:
        self.hub = hub

    def Run(self) -> DiagnosticsReport:
        """Execute all checks and return a fully populated report."""
        report = DiagnosticsReport()
        report.checks.append(self._CheckConnected())
        report.checks.append(self._CheckBattery())
        report.checks.append(self._CheckTimeMonotonic())
        report.checks.append(self._CheckImuReadable())
        return report

    def _CheckConnected(self) -> CheckResult:
        try:
            connected = self.hub.IsConnected()
        except Exception as exc:
            return CheckResult("hub.IsConnected", False, f"raised: {exc!r}")
        return CheckResult(
            "hub.IsConnected",
            bool(connected),
            "connected" if connected else "no connection reported",
        )

    def _CheckBattery(self) -> CheckResult:
        try:
            volts = self.hub.BatteryVoltage()
        except Exception as exc:
            return CheckResult("hub.BatteryVoltage", False, f"raised: {exc!r}")
        ok = isinstance(volts, (int, float)) and volts > 0
        return CheckResult("hub.BatteryVoltage", bool(ok), f"{volts:.2f} V")

    def _CheckTimeMonotonic(self) -> CheckResult:
        try:
            t0 = self.hub.Now()
            t1 = self.hub.Now()
        except Exception as exc:
            return CheckResult("hub.Now", False, f"raised: {exc!r}")
        ok = t1 >= t0
        return CheckResult("hub.Now monotonic", ok, f"t0={t0:.4f}, t1={t1:.4f}")

    def _CheckImuReadable(self) -> CheckResult:
        try:
            tilt = self.hub.TiltDegrees()
            rates = self.hub.AngularVelocityDegreesPerSec()
        except Exception as exc:
            return CheckResult("hub.IMU readable", False, f"raised: {exc!r}")
        ok = (
            isinstance(tilt, tuple)
            and len(tilt) == 2
            and isinstance(rates, tuple)
            and len(rates) == 3
        )
        return CheckResult(
            "hub.IMU readable",
            ok,
            f"tilt={tilt}, rates={rates}" if ok else "wrong shapes",
        )
