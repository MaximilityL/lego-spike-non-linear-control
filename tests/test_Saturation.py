"""Tests for LegoBalance.Saturation."""

from __future__ import annotations

import pytest

from LegoBalance.Saturation import SaturateScalar, SaturateSymmetric, SaturateVector


def test_ScalarInsideRange():
    assert SaturateScalar(0.5, -1.0, 1.0) == 0.5


def test_ScalarClampLow():
    assert SaturateScalar(-2.0, -1.0, 1.0) == -1.0


def test_ScalarClampHigh():
    assert SaturateScalar(3.0, -1.0, 1.0) == 1.0


def test_ScalarRejectsInvertedBounds():
    with pytest.raises(ValueError):
        SaturateScalar(0.0, 1.0, -1.0)


def test_SymmetricBasic():
    assert SaturateSymmetric(0.0, 5.0) == 0.0
    assert SaturateSymmetric(7.0, 5.0) == 5.0
    assert SaturateSymmetric(-9.0, 5.0) == -5.0


def test_SymmetricRejectsNegativeBound():
    with pytest.raises(ValueError):
        SaturateSymmetric(0.0, -1.0)


def test_VectorBasic():
    out = SaturateVector([1.0, 2.0, 3.0], [0.0, 0.0, 0.0], [1.5, 1.5, 1.5])
    assert out == [1.0, 1.5, 1.5]


def test_VectorLengthMismatch():
    with pytest.raises(ValueError):
        SaturateVector([1.0, 2.0], [0.0], [1.0, 1.0])
