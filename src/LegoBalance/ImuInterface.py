"""Abstract ImuInterface.

A small wrapper over the SPIKE Prime IMU that hides the choice of axis and
sign convention. The desktop side controller and estimator only see SI
radians and radians per second.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class ImuInterface(ABC):
    """IMU surface used by the estimator."""

    @abstractmethod
    def TiltAngleRadians(self) -> float:
        """Return the body tilt angle in radians.

        ``0`` means upright. The mapping from raw IMU axis to body tilt is
        configured in :class:`LegoBalance.RobotConfig.ImuConfig`.
        """

    @abstractmethod
    def TiltRateRadiansPerSec(self) -> float:
        """Return the body tilt rate in radians per second."""

    @abstractmethod
    def IsCalibrated(self) -> bool:
        """Return ``True`` once zero offset and gyro bias are known."""

    @abstractmethod
    def Calibrate(self, sampleCount: int = 200) -> None:
        """Estimate zero offset and gyro bias by averaging samples.

        The robot must be held still while this runs. Implementations
        should refuse to run while the body is moving.
        """
