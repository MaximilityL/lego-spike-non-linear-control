"""Tests for LegoBalance.Units."""

from __future__ import annotations

import math

import pytest

from LegoBalance.Units import (
    DegPerSecToRadPerSec,
    DegToRad,
    LinearDistanceToWheelAngle,
    MmPerSecToMPerSec,
    MPerSecToMmPerSec,
    RadPerSecToDegPerSec,
    RadToDeg,
    WheelAngleToLinearDistance,
)


def test_DegRadRoundTrip():
    for deg in (-180.0, -90.0, 0.0, 45.0, 360.0):
        assert RadToDeg(DegToRad(deg)) == pytest.approx(deg)


def test_DegToRadKnownValues():
    assert DegToRad(180.0) == pytest.approx(math.pi)
    assert DegToRad(90.0) == pytest.approx(math.pi / 2.0)


def test_DegPerSecMatchesDegToRad():
    assert DegPerSecToRadPerSec(360.0) == pytest.approx(2.0 * math.pi)
    assert RadPerSecToDegPerSec(math.pi) == pytest.approx(180.0)


def test_MmPerSecConversion():
    assert MmPerSecToMPerSec(1000.0) == pytest.approx(1.0)
    assert MPerSecToMmPerSec(1.0) == pytest.approx(1000.0)


def test_WheelDistanceRoundTrip():
    radius = 0.028
    distance = 0.5
    angle = LinearDistanceToWheelAngle(distance, radius)
    assert WheelAngleToLinearDistance(angle, radius) == pytest.approx(distance)


def test_LinearDistanceRejectsBadRadius():
    with pytest.raises(ValueError):
        LinearDistanceToWheelAngle(1.0, 0.0)
