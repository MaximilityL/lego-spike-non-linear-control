"""Tests for LegoBalance.SafetyMonitor."""

from __future__ import annotations

import math

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.RobotConfig import LoadConfig
from LegoBalance.SafetyMonitor import SafetyMonitor


def MakeSafeState() -> BalanceState:
    return BalanceState(
        tilt=0.05,
        tiltRate=0.1,
        phi=0.0,
        phiDot=0.0,
        timestamp=0.0,
        valid=True,
    )


def MakeCommand(left: float = 1.0, right: float = 1.0, ts: float = 0.0) -> ControlOutput:
    return ControlOutput(
        leftCommand=left, rightCommand=right, mode=ControlMode.Velocity, timestamp=ts
    )


def test_DefaultsAreDisarmed():
    config = LoadConfig(applyLocalOverride=False)
    monitor = SafetyMonitor(config)
    assert monitor.status.armed is False
    out = monitor.Check(MakeSafeState(), MakeCommand())
    # Disarmed monitor must zero the command.
    assert out.leftCommand == 0.0
    assert out.rightCommand == 0.0


def test_ArmedPassesThroughSafeCommand():
    config = LoadConfig(applyLocalOverride=False)
    monitor = SafetyMonitor(config)
    monitor.Arm(currentTime=0.0)
    out = monitor.Check(MakeSafeState(), MakeCommand(left=2.0, right=2.0), currentTime=0.01)
    assert out.leftCommand == 2.0
    assert out.rightCommand == 2.0


def test_TripsOnExcessiveTilt():
    config = LoadConfig(applyLocalOverride=False)
    monitor = SafetyMonitor(config)
    monitor.Arm(currentTime=0.0)
    bigTilt = BalanceState(
        tilt=config.control.maxTilt + 0.1,
        tiltRate=0.0,
        valid=True,
        timestamp=0.0,
    )
    out = monitor.Check(bigTilt, MakeCommand(), currentTime=0.0)
    assert out.leftCommand == 0.0
    assert out.rightCommand == 0.0
    assert monitor.status.tripped is True
    assert any("tilt" in r for r in monitor.status.reasons)


def test_TripsOnExcessiveTiltRate():
    config = LoadConfig(applyLocalOverride=False)
    monitor = SafetyMonitor(config)
    monitor.Arm(currentTime=0.0)
    fast = BalanceState(
        tilt=0.0,
        tiltRate=config.control.maxTiltRate * 2.0,
        valid=True,
        timestamp=0.0,
    )
    monitor.Check(fast, MakeCommand(), currentTime=0.0)
    assert monitor.status.tripped is True


def test_SaturatesCommandToWheelLimit():
    config = LoadConfig(applyLocalOverride=False)
    monitor = SafetyMonitor(config)
    monitor.Arm(currentTime=0.0)
    huge = MakeCommand(left=1e6, right=-1e6)
    out = monitor.Check(MakeSafeState(), huge, currentTime=0.0)
    assert out.leftCommand == config.control.maxWheelRate
    assert out.rightCommand == -config.control.maxWheelRate


def test_WatchdogTripsOnStaleness():
    config = LoadConfig(applyLocalOverride=False)
    monitor = SafetyMonitor(config)
    monitor.Arm(currentTime=0.0)
    longLater = config.control.watchdogTimeout + 1.0
    monitor.Check(MakeSafeState(), MakeCommand(), currentTime=longLater)
    assert monitor.status.tripped is True
    assert any("watchdog" in r for r in monitor.status.reasons)


def test_DisarmStopsCommands():
    config = LoadConfig(applyLocalOverride=False)
    monitor = SafetyMonitor(config)
    monitor.Arm(currentTime=0.0)
    monitor.Disarm()
    out = monitor.Check(MakeSafeState(), MakeCommand(left=1.0), currentTime=0.0)
    assert out.leftCommand == 0.0


def test_InvalidStateTrips():
    config = LoadConfig(applyLocalOverride=False)
    monitor = SafetyMonitor(config)
    monitor.Arm(currentTime=0.0)
    invalid = BalanceState(valid=False)
    out = monitor.Check(invalid, MakeCommand(), currentTime=0.0)
    assert out.leftCommand == 0.0
    assert monitor.status.tripped is True


def test_NonFiniteStateTrips():
    config = LoadConfig(applyLocalOverride=False)
    monitor = SafetyMonitor(config)
    monitor.Arm(currentTime=0.0)
    invalid = BalanceState(tilt=math.nan, tiltRate=0.0, valid=True)
    out = monitor.Check(invalid, MakeCommand(), currentTime=0.0)
    assert out.leftCommand == 0.0
    assert monitor.status.tripped is True
    assert any("not finite" in reason for reason in monitor.status.reasons)


def test_NonFiniteCommandTrips():
    config = LoadConfig(applyLocalOverride=False)
    monitor = SafetyMonitor(config)
    monitor.Arm(currentTime=0.0)
    out = monitor.Check(MakeSafeState(), MakeCommand(left=math.nan, right=1.0), currentTime=0.0)
    assert out.leftCommand == 0.0
    assert monitor.status.tripped is True
    assert any("not finite" in reason for reason in monitor.status.reasons)
