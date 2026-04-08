"""Robot configuration loader.

Loads ``configs/Default.yaml`` and an optional ``configs/local.yaml`` override.
The result is a typed ``RobotConfig`` dataclass that the rest of the project
uses instead of dictionaries.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = REPO_ROOT / "configs" / "Default.yaml"
LOCAL_CONFIG_PATH = REPO_ROOT / "configs" / "local.yaml"


@dataclass
class ChassisConfig:
    wheelRadius: float = 0.0285
    wheelBase: float = 0.12
    bodyMass: float = 0.5
    bodyHeightCoM: float = 0.08
    bodyInertia: float = 0.002


@dataclass
class MotorsConfig:
    leftPort: str = "B"
    rightPort: str = "A"
    maxAngularRate: float = 17.453292519943295
    maxDuty: float = 80.0
    encoderCountsPerRev: int = 360
    forwardSign: int = -1
    leftEncoderSign: int = 1
    rightEncoderSign: int = -1


@dataclass
class ImuConfig:
    tiltAxis: str = "pitch"
    tiltSign: int = -1
    zeroOffset: float = -1.0471975511965976
    gyroBias: float = 0.0


@dataclass
class EstimatorConfig:
    filterType: str = "complementary"
    alpha: float = 0.98
    loopRate: float = 100.0


@dataclass
class ControlConfig:
    loopRate: float = 100.0
    maxTilt: float = 2.0
    maxTiltRate: float = 10.0
    maxWheelRate: float = 50.0
    watchdogTimeout: float = 0.2


@dataclass
class DriveConfig:
    """Settings for the pre balancing drive command path.

    These defaults feed the package-backed hub smoke flow directly. The
    forward/backward/stop smoke flow is still a bench test with the wheels
    lifted, not autonomous motion.
    """

    loopPeriodMs: int = 20
    printEveryN: int = 1
    stopDurationMs: int = 50
    driveDurationMs: int = 5000
    testSpeed: float = 30.0
    maxTiltForMotion: float = 2.0


@dataclass
class LoggingConfig:
    enabled: bool = True
    level: str = "info"
    bufferSize: int = 2048


@dataclass
class RobotConfig:
    """Top level typed view of the YAML config."""

    name: str = "LegoBalance Mk0"
    description: str = ""
    chassis: ChassisConfig = field(default_factory=ChassisConfig)
    motors: MotorsConfig = field(default_factory=MotorsConfig)
    imu: ImuConfig = field(default_factory=ImuConfig)
    estimator: EstimatorConfig = field(default_factory=EstimatorConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    drive: DriveConfig = field(default_factory=DriveConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)

    def Validate(self) -> None:
        """Cheap consistency checks. Raises ``ValueError`` on failure."""
        if self.chassis.wheelRadius <= 0:
            raise ValueError("chassis.wheelRadius must be positive")
        if self.chassis.wheelBase <= 0:
            raise ValueError("chassis.wheelBase must be positive")
        if self.motors.maxAngularRate <= 0:
            raise ValueError("motors.maxAngularRate must be positive")
        if self.motors.forwardSign not in (-1, 1):
            raise ValueError("motors.forwardSign must be +1 or -1")
        if self.motors.leftEncoderSign not in (-1, 1):
            raise ValueError("motors.leftEncoderSign must be +1 or -1")
        if self.motors.rightEncoderSign not in (-1, 1):
            raise ValueError("motors.rightEncoderSign must be +1 or -1")
        if not (0.0 < self.estimator.alpha < 1.0):
            raise ValueError("estimator.alpha must lie strictly between 0 and 1")
        if self.control.loopRate <= 0:
            raise ValueError("control.loopRate must be positive")
        if self.control.maxTilt <= 0:
            raise ValueError("control.maxTilt must be positive")
        if self.imu.tiltSign not in (-1, 1):
            raise ValueError("imu.tiltSign must be +1 or -1")
        if self.drive.testSpeed < 0:
            raise ValueError("drive.testSpeed must be non negative")
        if self.drive.loopPeriodMs <= 0:
            raise ValueError("drive.loopPeriodMs must be positive")
        if self.drive.printEveryN <= 0:
            raise ValueError("drive.printEveryN must be positive")
        if self.drive.stopDurationMs < 0:
            raise ValueError("drive.stopDurationMs must be non negative")
        if self.drive.driveDurationMs < 0:
            raise ValueError("drive.driveDurationMs must be non negative")
        if self.drive.maxTiltForMotion <= 0:
            raise ValueError("drive.maxTiltForMotion must be positive")
        if self.drive.maxTiltForMotion > self.control.maxTilt:
            raise ValueError(
                "drive.maxTiltForMotion must be less than or equal to control.maxTilt"
            )


def DeepMerge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Recursively merge ``override`` into ``base`` and return the result.

    Both arguments are left untouched. Lists are not merged element wise.
    """
    result = dict(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = DeepMerge(result[key], value)
        else:
            result[key] = value
    return result


def _DictToConfig(data: dict[str, Any]) -> RobotConfig:
    robotSection = data.get("robot", {}) or {}
    return RobotConfig(
        name=robotSection.get("name", "LegoBalance Mk0"),
        description=robotSection.get("description", ""),
        chassis=ChassisConfig(**(data.get("chassis", {}) or {})),
        motors=MotorsConfig(**(data.get("motors", {}) or {})),
        imu=ImuConfig(**(data.get("imu", {}) or {})),
        estimator=EstimatorConfig(**(data.get("estimator", {}) or {})),
        control=ControlConfig(**(data.get("control", {}) or {})),
        drive=DriveConfig(**(data.get("drive", {}) or {})),
        logging=LoggingConfig(**(data.get("logging", {}) or {})),
    )


def LoadConfig(path: Path | None = None, applyLocalOverride: bool = True) -> RobotConfig:
    """Load a ``RobotConfig`` from disk.

    Args:
        path: Optional explicit YAML path. If omitted the default config
            shipped under ``configs/Default.yaml`` is used.
        applyLocalOverride: If ``True`` and ``configs/local.yaml`` exists,
            its values are deep merged on top of the base. Set to ``False``
            in tests to keep determinism.

    Returns:
        A validated ``RobotConfig`` instance.
    """
    basePath = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    if not basePath.exists():
        raise FileNotFoundError(f"Config file not found at {basePath}")
    with basePath.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}

    if applyLocalOverride and path is None and LOCAL_CONFIG_PATH.exists():
        with LOCAL_CONFIG_PATH.open("r", encoding="utf-8") as fh:
            override = yaml.safe_load(fh) or {}
        data = DeepMerge(data, override)

    config = _DictToConfig(data)
    config.Validate()
    return config
