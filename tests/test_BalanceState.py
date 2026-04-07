"""Tests for LegoBalance.BalanceState."""

from __future__ import annotations

from LegoBalance.BalanceState import BalanceState, StateBounds


def test_DefaultsAreSafe():
    state = BalanceState()
    assert state.tilt == 0.0
    assert state.tiltRate == 0.0
    assert state.wheelPosition == 0.0
    assert state.wheelVelocity == 0.0
    assert state.valid is False


def test_AsListShape():
    state = BalanceState(tilt=0.1, tiltRate=0.2, wheelPosition=0.3, wheelVelocity=0.4)
    assert state.AsList() == [0.1, 0.2, 0.3, 0.4]


def test_CopyIsIndependent():
    state = BalanceState(tilt=1.0, valid=True)
    clone = state.Copy()
    clone.tilt = 999.0
    assert state.tilt == 1.0
    assert state.valid is True


def test_StateBoundsHasSensibleDefaults():
    bounds = StateBounds()
    assert bounds.tiltMax > 0
    assert bounds.tiltRateMax > 0
    assert bounds.wheelVelocityMax > 0
