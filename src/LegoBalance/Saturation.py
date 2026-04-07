"""Saturation utilities.

A saturation step is the simplest and most important safety primitive in any
control loop. It is also one of the easiest to test in isolation, so it lives
here as its own module rather than being inlined into the controller.
"""

from __future__ import annotations

from collections.abc import Iterable


def SaturateScalar(value: float, lowerBound: float, upperBound: float) -> float:
    """Clamp a scalar to the closed interval ``[lowerBound, upperBound]``.

    Raises ``ValueError`` if the bounds are inverted, because that almost
    always indicates a configuration mistake rather than an intentional
    no op.
    """
    if lowerBound > upperBound:
        raise ValueError(
            f"SaturateScalar received inverted bounds: {lowerBound} > {upperBound}"
        )
    if value < lowerBound:
        return lowerBound
    if value > upperBound:
        return upperBound
    return value


def SaturateSymmetric(value: float, magnitudeBound: float) -> float:
    """Clamp a scalar to ``[-magnitudeBound, magnitudeBound]``.

    Convenience wrapper for the very common symmetric case.
    """
    if magnitudeBound < 0.0:
        raise ValueError(f"magnitudeBound must be non negative, got {magnitudeBound}")
    return SaturateScalar(value, -magnitudeBound, magnitudeBound)


def SaturateVector(
    values: Iterable[float],
    lowerBounds: Iterable[float],
    upperBounds: Iterable[float],
) -> list[float]:
    """Component wise saturation of a vector.

    All three iterables must have the same length. Returned as a list so the
    caller can index into it without surprises.
    """
    valuesList = list(values)
    lowerList = list(lowerBounds)
    upperList = list(upperBounds)
    if not (len(valuesList) == len(lowerList) == len(upperList)):
        raise ValueError(
            "SaturateVector requires equal length values, lowerBounds, and upperBounds"
        )
    return [
        SaturateScalar(v, lo, hi)
        for v, lo, hi in zip(valuesList, lowerList, upperList, strict=True)
    ]
