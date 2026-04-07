"""Tests for LegoBalance.ConnectionDiagnostics."""

from __future__ import annotations

from LegoBalance.ConnectionDiagnostics import ConnectionDiagnostics, DiagnosticsReport
from LegoBalance.HubInterface import HubInterface
from LegoBalance.MockAdapters import MockHub


def test_DiagnosticsPassesAgainstMockHub():
    hub = MockHub()
    diagnostics = ConnectionDiagnostics(hub)
    report = diagnostics.Run()
    assert isinstance(report, DiagnosticsReport)
    assert report.Passed is True
    assert all(check.passed for check in report.checks)


def test_DiagnosticsCatchesUnconnectedHub():
    hub = MockHub()
    hub.Shutdown()
    diagnostics = ConnectionDiagnostics(hub)
    report = diagnostics.Run()
    # The first check (IsConnected) should fail.
    assert report.Passed is False
    failedNames = [c.name for c in report.checks if not c.passed]
    assert "hub.IsConnected" in failedNames


class _BrokenHub(HubInterface):
    """A hub that raises on every call. Used to verify exception handling."""

    def IsConnected(self) -> bool:
        raise RuntimeError("nope")

    def BatteryVoltage(self) -> float:
        raise RuntimeError("nope")

    def Now(self) -> float:
        raise RuntimeError("nope")

    def TiltDegrees(self):
        raise RuntimeError("nope")

    def AngularVelocityDegreesPerSec(self):
        raise RuntimeError("nope")

    def Shutdown(self) -> None:
        return None


def test_DiagnosticsHandlesExceptions():
    diagnostics = ConnectionDiagnostics(_BrokenHub())
    report = diagnostics.Run()
    assert report.Passed is False
    for check in report.checks:
        assert check.passed is False
        assert "raised" in check.detail


def test_SummaryStringContainsCheckNames():
    hub = MockHub()
    diagnostics = ConnectionDiagnostics(hub)
    report = diagnostics.Run()
    summary = report.Summary()
    assert "hub.IsConnected" in summary
    assert "Overall" in summary
