"""Tests for LegoBalance.BalanceState."""

from __future__ import annotations

import pytest

from LegoBalance.BalanceState import BalanceState, StateBounds


def test_DefaultsAreSafe():
    state = BalanceState()
    assert state.tilt == 0.0
    assert state.tiltRate == 0.0
    assert state.phi == 0.0
    assert state.phiDot == 0.0
    assert state.valid is False


def test_AsListShape():
    state = BalanceState(tilt=0.1, tiltRate=0.2, phi=0.3, phiDot=0.4)
    assert state.AsList() == [0.1, 0.2, 0.3, 0.4]


def test_ThetaAndThetaDotAliases():
    state = BalanceState(tilt=0.11, tiltRate=0.22, phi=0.33, phiDot=0.44)
    assert state.theta == pytest.approx(0.11)
    assert state.thetaDot == pytest.approx(0.22)


def test_CopyIsIndependent():
    state = BalanceState(tilt=1.0, valid=True, phi=2.0, phiDot=3.0)
    clone = state.Copy()
    clone.tilt = 999.0
    clone.phi = 999.0
    assert state.tilt == 1.0
    assert state.phi == 2.0
    assert state.valid is True


def test_LinearPositionConvertsPhiToMeters():
    state = BalanceState(phi=2.0, phiDot=4.0)
    radius = 0.05
    assert state.LinearPosition(radius) == pytest.approx(0.10)
    assert state.LinearVelocity(radius) == pytest.approx(0.20)


def test_LinearHelpersRejectNonPositiveRadius():
    state = BalanceState(phi=1.0, phiDot=1.0)
    with pytest.raises(ValueError):
        state.LinearPosition(0.0)
    with pytest.raises(ValueError):
        state.LinearVelocity(-0.01)


def test_StateBoundsHasSensibleDefaults():
    bounds = StateBounds()
    assert bounds.tiltMax > 0
    assert bounds.tiltRateMax > 0
    assert bounds.phiDotMax > 0
