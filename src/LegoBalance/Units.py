"""Unit conversions used throughout the desktop side code.

Pybricks reports angles in degrees and angular rates in degrees per second.
The rest of this project uses radians and radians per second. The functions
here are the only place that conversion happens, so that sign and unit bugs
have one place to live.
"""

PI = 3.141592653589793


def DegToRad(deg: float) -> float:
    """Convert degrees to radians."""
    return deg * PI / 180.0


def RadToDeg(rad: float) -> float:
    """Convert radians to degrees."""
    return rad * 180.0 / PI


def DegPerSecToRadPerSec(degPerSec: float) -> float:
    """Convert deg/s to rad/s."""
    return DegToRad(degPerSec)


def RadPerSecToDegPerSec(radPerSec: float) -> float:
    """Convert rad/s to deg/s."""
    return RadToDeg(radPerSec)


def MmPerSecToMPerSec(mmPerSec: float) -> float:
    """Convert mm/s (Pybricks acceleration units use mm based length) to m/s."""
    return mmPerSec / 1000.0


def MPerSecToMmPerSec(mPerSec: float) -> float:
    """Convert m/s to mm/s."""
    return mPerSec * 1000.0


def WheelAngleToLinearDistance(wheelAngleRad: float, wheelRadius: float) -> float:
    """Map wheel rotation in radians to linear distance traveled.

    Assumes pure rolling. The caller is responsible for averaging the two
    wheels and for the sign convention.
    """
    return wheelAngleRad * wheelRadius


def LinearDistanceToWheelAngle(distanceM: float, wheelRadius: float) -> float:
    """Map linear distance to wheel rotation in radians."""
    if wheelRadius <= 0.0:
        raise ValueError("wheelRadius must be positive")
    return distanceM / wheelRadius
