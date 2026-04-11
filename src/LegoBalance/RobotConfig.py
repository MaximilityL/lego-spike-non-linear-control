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
VALID_MOTOR_PORTS = ("A", "B", "C", "D", "E", "F")


@dataclass
class ChassisConfig:
    wheelRadius: float = 0.0285
    wheelBase: float = 0.11
    bodyMass: float = 0.276
    bodyHeightCoM: float = 0.105
    # bodyInertia is the pitch inertia of the chassis about the body center
    # of mass. It is the nominal value used by the geometry aware nonlinear
    # controller to build its reduced tilt model. The parallel axis term
    # m*bodyHeightCoM^2 is added internally to obtain the axle referred
    # moment of inertia. If this value is left at zero the box fallback
    # below is used.
    bodyInertia: float = 0.00337
    # Optional box body dimensions used to fall back to a rectangular
    # approximation of the pitch inertia when bodyInertia is unknown or
    # left at zero. Both must be positive for the fallback to trigger.
    bodyLength: float = 0.0
    bodyHeight: float = 0.0


@dataclass
class MotorsConfig:
    leftPort: str = "B"
    rightPort: str = "F"
    rightAuxPort: str = ""
    maxAngularRate: float = 17.453292519943295
    maxDuty: float = 90.0
    encoderCountsPerRev: int = 360
    forwardSign: int = -1
    leftEncoderSign: int = 1
    rightEncoderSign: int = -1
    rightAuxEncoderSign: int = 1


def _NormalizeRequiredMotorPort(value: object, fieldName: str) -> str:
    portName = str(value).strip().upper()
    if portName not in VALID_MOTOR_PORTS:
        raise ValueError(f"{fieldName} must be one of {VALID_MOTOR_PORTS}")
    return portName


def _NormalizeOptionalMotorPort(value: object, fieldName: str) -> str:
    portName = str(value).strip().upper()
    if not portName:
        return ""
    if portName not in VALID_MOTOR_PORTS:
        raise ValueError(f"{fieldName} must be empty or one of {VALID_MOTOR_PORTS}")
    return portName


@dataclass
class ImuConfig:
    tiltAxis: str = "roll"
    tiltSign: int = 1
    zeroOffset: float = -1.5960163345
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
    maxWheelRate: float = 17.44
    watchdogTimeout: float = 0.2
    targetTilt: float = -0.019


@dataclass
class ControllerConfig:
    """Tuning constants for the balance controller selected by config."""

    algorithm: str = "pid"
    gravityCompGain: float = 0.5        # preserved for backward compat; unused by balance controllers
    # ----- Legacy tanh composite variable fields (no longer the active law) -----
    # These are retained only so existing YAML files, the hub runtime mirror,
    # and the HubPackageBalance banner keep loading. The current
    # NonLinearController is the geometry aware robust law below and does not
    # read any of these fields.
    kTheta: float = 75.0
    kThetaDot: float = 4.5
    kPhi: float = 0.0
    kPhiDot: float = 0.0
    sScale: float = 14.0
    thetaDeadband: float = 0.0
    thetaDotDeadband: float = 0.0
    # ----- Shared fields used by the new NonLinearController -----
    # Effective wheel speed time constant inferred from the Port F velocity
    # sweep. The geometry aware controller requires this to invert the
    # actuator lag when mapping desired chassis acceleration to a wheel
    # velocity command.
    actuatorTau: float = 0.2
    # Optional first order IIR low pass filter on the measured tilt rate.
    # Use only to cut noise above the control bandwidth; heavy filtering
    # adds phase lag and hurts stability.
    thetaDotFilterAlpha: float = 0.0
    # ----- Geometry aware robust nonlinear controller fields -----
    # Inner loop nominal stiffness and damping. These are specified as the
    # natural frequency and damping ratio of the small signal error
    # dynamics eDDot + kDamping*eDot + lambda*e = 0 used by the controller.
    innerNaturalFrequency: float = 6.0
    innerDampingRatio: float = 1.0
    # Sliding surface gain on the sin(e) term, in units of 1/s. It sets
    # how much the tilt error contributes to the surface variable s that
    # feeds the smooth robust correction.
    surfaceGain: float = 3.0
    # Magnitude of the smooth robust correction acceleration, in rad/s^2.
    # Bigger values improve disturbance rejection and tolerance to model
    # mismatch but can feel aggressive near upright.
    robustGain: float = 2.0
    # Boundary layer width of the tanh smoothing. Strictly positive. A
    # very small value makes the robust term look like sign(s) and causes
    # chatter; a large value softens it and can slow the response.
    boundaryLayerWidth: float = 0.5
    # Outer loop proportional and derivative gains on the chassis linear
    # position and velocity. These bias the tilt reference so the robot
    # slowly returns toward the origin. They must be slower than the
    # inner loop to avoid fighting the fast anti fall law.
    outerPositionGain: float = 0.0
    outerVelocityGain: float = 0.0
    # Symmetric cap on the outer loop tilt reference offset, in radians.
    # Prevents the recentering loop from asking for unreasonable lean.
    maxReferenceTiltOffset: float = 0.1
    # ----- PID baseline controller fields -----
    pidKp: float = 11.0                 # proportional gain, matching the referenced example
    pidKi: float = 4.2                  # integral gain, matching the referenced example
    pidKd: float = 92.0                 # derivative gain, matching the referenced example
    pidKs: float = -0.6                 # wheel-position correction gain from the example
    pidIntegralStep: float = 0.25       # discrete integral increment scale from the example
    pidIntegralLimit: float = 50.0      # simple anti-windup clamp on the integral state
    pidPositionTargetDeg: float = 0.0   # target wheel position used by the start term


@dataclass
class DriveConfig:
    """Settings for the pre balancing drive command path.

    These defaults feed the package-backed hub smoke flow directly. The
    forward/backward/stop smoke flow is still a bench test with the wheels
    lifted, not autonomous motion.
    """

    loopPeriodMs: int = 20
    printEveryN: int = 20
    stopDurationMs: int = 50
    driveDurationMs: int = 5000
    testSpeed: float = 17.44
    maxTiltForMotion: float = 0.8726646259971648


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
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    drive: DriveConfig = field(default_factory=DriveConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)

    def Validate(self) -> None:
        """Cheap consistency checks. Raises ``ValueError`` on failure."""
        if self.chassis.wheelRadius <= 0:
            raise ValueError("chassis.wheelRadius must be positive")
        if self.chassis.wheelBase <= 0:
            raise ValueError("chassis.wheelBase must be positive")
        if self.chassis.bodyMass <= 0:
            raise ValueError("chassis.bodyMass must be positive")
        if self.chassis.bodyHeightCoM <= 0:
            raise ValueError("chassis.bodyHeightCoM must be positive")
        if self.chassis.bodyInertia < 0:
            raise ValueError("chassis.bodyInertia must be non negative")
        if self.chassis.bodyLength < 0:
            raise ValueError("chassis.bodyLength must be non negative")
        if self.chassis.bodyHeight < 0:
            raise ValueError("chassis.bodyHeight must be non negative")
        self.motors.leftPort = _NormalizeRequiredMotorPort(
            self.motors.leftPort,
            "motors.leftPort",
        )
        self.motors.rightPort = _NormalizeRequiredMotorPort(
            self.motors.rightPort,
            "motors.rightPort",
        )
        self.motors.rightAuxPort = _NormalizeOptionalMotorPort(
            self.motors.rightAuxPort,
            "motors.rightAuxPort",
        )
        usedPorts = [self.motors.leftPort, self.motors.rightPort]
        if self.motors.rightAuxPort:
            usedPorts.append(self.motors.rightAuxPort)
        if len(set(usedPorts)) != len(usedPorts):
            raise ValueError("motor ports must be unique across left/right/rightAux")
        if self.motors.maxAngularRate <= 0:
            raise ValueError("motors.maxAngularRate must be positive")
        if self.motors.forwardSign not in (-1, 1):
            raise ValueError("motors.forwardSign must be +1 or -1")
        if self.motors.leftEncoderSign not in (-1, 1):
            raise ValueError("motors.leftEncoderSign must be +1 or -1")
        if self.motors.rightEncoderSign not in (-1, 1):
            raise ValueError("motors.rightEncoderSign must be +1 or -1")
        if self.motors.rightAuxEncoderSign not in (-1, 1):
            raise ValueError("motors.rightAuxEncoderSign must be +1 or -1")
        if not (0.0 < self.estimator.alpha < 1.0):
            raise ValueError("estimator.alpha must lie strictly between 0 and 1")
        if self.control.loopRate <= 0:
            raise ValueError("control.loopRate must be positive")
        if self.control.maxTilt <= 0:
            raise ValueError("control.maxTilt must be positive")
        if str(self.imu.tiltAxis).strip().lower() not in ("pitch", "roll"):
            raise ValueError("imu.tiltAxis must be 'pitch' or 'roll'")
        if self.imu.tiltSign not in (-1, 1):
            raise ValueError("imu.tiltSign must be +1 or -1")
        if self.controller.gravityCompGain < 0:
            raise ValueError("controller.gravityCompGain must be non negative")
        if str(self.controller.algorithm).strip().lower() not in ("tanh", "nonlinear", "pid"):
            raise ValueError("controller.algorithm must be 'tanh', 'nonlinear', or 'pid'")
        if self.controller.kTheta < 0:
            raise ValueError("controller.kTheta must be non negative")
        if self.controller.kThetaDot < 0:
            raise ValueError("controller.kThetaDot must be non negative")
        if self.controller.kPhi < 0:
            raise ValueError("controller.kPhi must be non negative")
        if self.controller.kPhiDot < 0:
            raise ValueError("controller.kPhiDot must be non negative")
        if self.controller.thetaDeadband < 0:
            raise ValueError("controller.thetaDeadband must be non negative")
        if self.controller.thetaDotDeadband < 0:
            raise ValueError("controller.thetaDotDeadband must be non negative")
        if self.controller.sScale <= 0:
            raise ValueError("controller.sScale must be positive")
        if self.controller.actuatorTau < 0:
            raise ValueError("controller.actuatorTau must be non negative")
        if not (0.0 <= self.controller.thetaDotFilterAlpha < 1.0):
            raise ValueError("controller.thetaDotFilterAlpha must be in [0.0, 1.0)")
        if self.controller.innerNaturalFrequency <= 0:
            raise ValueError("controller.innerNaturalFrequency must be positive")
        if self.controller.innerDampingRatio < 0:
            raise ValueError("controller.innerDampingRatio must be non negative")
        if self.controller.surfaceGain < 0:
            raise ValueError("controller.surfaceGain must be non negative")
        if self.controller.robustGain < 0:
            raise ValueError("controller.robustGain must be non negative")
        if self.controller.boundaryLayerWidth <= 0:
            raise ValueError("controller.boundaryLayerWidth must be strictly positive")
        if self.controller.outerPositionGain < 0:
            raise ValueError("controller.outerPositionGain must be non negative")
        if self.controller.outerVelocityGain < 0:
            raise ValueError("controller.outerVelocityGain must be non negative")
        if self.controller.maxReferenceTiltOffset < 0:
            raise ValueError("controller.maxReferenceTiltOffset must be non negative")
        if self.controller.pidKp < 0:
            raise ValueError("controller.pidKp must be non negative")
        if self.controller.pidKi < 0:
            raise ValueError("controller.pidKi must be non negative")
        if self.controller.pidKd < 0:
            raise ValueError("controller.pidKd must be non negative")
        if self.controller.pidIntegralStep < 0:
            raise ValueError("controller.pidIntegralStep must be non negative")
        if self.controller.pidIntegralLimit < 0:
            raise ValueError("controller.pidIntegralLimit must be non negative")
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
        controller=ControllerConfig(**(data.get("controller", {}) or {})),
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
