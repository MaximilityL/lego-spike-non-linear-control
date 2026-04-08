"""LegoBalance package.

Shared control logic for the LEGO SPIKE inverted pendulum starter project.
The package avoids importing from ``pybricks``. Desktop-only helpers are kept
out of the Pybricks import path so hub smoke tests can import the estimator and
controller directly.
"""

# ruff: noqa: E402

import sys

__version__ = "1.1.1"

try:
    _PROPERTY_TYPE = property
except NameError:
    _HAS_PROPERTY = False
else:
    _HAS_PROPERTY = True
    del _PROPERTY_TYPE

_IMPLEMENTATION = getattr(sys, "implementation", None)
_IS_MICROPYTHON = (
    getattr(_IMPLEMENTATION, "name", "") == "micropython"
    or not _HAS_PROPERTY
)

if _IS_MICROPYTHON:
    __all__ = []
else:
    from LegoBalance.BalanceState import BalanceState, StateBounds
    from LegoBalance.ControlInterfaces import ControlMode, ControlOutput, Measurement
    from LegoBalance.ControllerBase import ControllerBase
    from LegoBalance.DataLogger import DataLogger
    from LegoBalance.DriveCommandController import DriveCommand, DriveCommandController
    from LegoBalance.HubInterface import HubInterface
    from LegoBalance.ImuInterface import ImuInterface
    from LegoBalance.LyapunovController import LyapunovController
    from LegoBalance.NonLinearController import NonLinearController
    from LegoBalance.MotorInterface import MotorInterface
    from LegoBalance.RobotConfig import LoadConfig, RobotConfig
    from LegoBalance.SafetyMonitor import SafetyMonitor, SafetyStatus
    from LegoBalance.Saturation import SaturateScalar, SaturateVector
    from LegoBalance.StateEstimator import StateEstimator
    from LegoBalance.Units import DegPerSecToRadPerSec, DegToRad, RadPerSecToDegPerSec, RadToDeg

    __all__ = [
        "BalanceState",
        "ControlMode",
        "ControlOutput",
        "ControllerBase",
        "DataLogger",
        "DegPerSecToRadPerSec",
        "DegToRad",
        "DriveCommand",
        "DriveCommandController",
        "HubInterface",
        "ImuInterface",
        "LoadConfig",
        "LyapunovController",
        "Measurement",
        "MotorInterface",
        "NonLinearController",
        "RadPerSecToDegPerSec",
        "RadToDeg",
        "RobotConfig",
        "SafetyMonitor",
        "SafetyStatus",
        "SaturateScalar",
        "SaturateVector",
        "StateBounds",
        "StateEstimator",
    ]
