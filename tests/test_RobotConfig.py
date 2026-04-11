"""Tests for LegoBalance.RobotConfig."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from LegoBalance.RobotConfig import (
    DEFAULT_CONFIG_PATH,
    DeepMerge,
    LoadConfig,
    RobotConfig,
)


def test_DefaultConfigLoadsAndValidates():
    config = LoadConfig(applyLocalOverride=False)
    assert isinstance(config, RobotConfig)
    assert config.chassis.wheelRadius == pytest.approx(0.0285)
    assert config.chassis.wheelBase == pytest.approx(0.1275)
    assert config.motors.leftPort == "B"
    assert config.motors.rightPort == "F"
    assert config.motors.rightAuxPort == "D"
    assert config.motors.forwardSign == -1
    assert config.motors.leftEncoderSign == 1
    assert config.motors.rightEncoderSign == -1
    assert config.motors.rightAuxEncoderSign == -1
    assert config.motors.maxAngularRate == pytest.approx(17.4532925199)
    assert config.imu.zeroOffset == pytest.approx(-0.025220008)
    assert 0.0 < config.estimator.alpha < 1.0
    assert config.control.maxTilt > 0
    assert config.control.maxTilt == pytest.approx(2.0)
    assert config.control.maxWheelRate == pytest.approx(17.44)
    assert abs(config.control.targetTilt) < config.control.maxTilt
    assert config.controller.algorithm in {"pid", "tanh", "nonlinear"}
    assert config.controller.gravityCompGain >= 0.0
    assert config.controller.kTheta >= 0.0
    assert config.controller.kThetaDot >= 0.0
    assert config.controller.kPhi >= 0.0
    assert config.controller.kPhiDot >= 0.0
    assert config.controller.sScale > 0.0
    assert config.controller.actuatorTau >= 0.0
    assert 0.0 <= config.controller.thetaDotFilterAlpha < 1.0
    assert config.controller.thetaDeadband >= 0.0
    assert config.controller.thetaDotDeadband >= 0.0
    assert config.controller.pidKp >= 0.0
    assert config.controller.pidKi >= 0.0
    assert config.controller.pidKd >= 0.0
    assert config.controller.pidIntegralStep >= 0.0
    assert config.controller.pidIntegralLimit >= 0.0
    assert config.drive.loopPeriodMs == 20
    assert config.drive.printEveryN > 0
    assert config.drive.stopDurationMs == 50
    assert config.drive.driveDurationMs == 5000
    assert config.drive.testSpeed == pytest.approx(17.44)
    assert config.drive.maxTiltForMotion == pytest.approx(0.8726646259971648)


def test_TypedConfigDefaultsValidate():
    config = RobotConfig()
    config.Validate()
    assert config.controller.algorithm in {"pid", "tanh", "nonlinear"}


def test_DefaultPathExists():
    assert DEFAULT_CONFIG_PATH.exists(), "shipped default config must exist"


def test_DeepMergeBasic():
    base = {"a": 1, "b": {"c": 2, "d": 3}}
    override = {"b": {"d": 30, "e": 40}, "f": 5}
    merged = DeepMerge(base, override)
    assert merged == {"a": 1, "b": {"c": 2, "d": 30, "e": 40}, "f": 5}
    # original dicts should be untouched
    assert base == {"a": 1, "b": {"c": 2, "d": 3}}


def test_LoadConfigAcceptsExplicitPath(tmp_path: Path):
    custom = {
        "robot": {"name": "TestBot"},
        "chassis": {"wheelRadius": 0.05, "wheelBase": 0.20},
        "motors": {"leftPort": "C", "rightPort": "D"},
        "estimator": {"alpha": 0.5},
        "control": {"maxTilt": 1.0},
        "controller": {"kTheta": 12.5, "gravityCompGain": 0.4},
        "drive": {"maxTiltForMotion": 0.5},
    }
    customPath = tmp_path / "custom.yaml"
    customPath.write_text(yaml.safe_dump(custom))
    config = LoadConfig(path=customPath)
    assert config.name == "TestBot"
    assert config.chassis.wheelRadius == pytest.approx(0.05)
    assert config.motors.leftPort == "C"
    assert config.estimator.alpha == pytest.approx(0.5)
    assert config.controller.kTheta == pytest.approx(12.5)
    assert config.controller.gravityCompGain == pytest.approx(0.4)


def test_LoadConfigRejectsBadAlpha(tmp_path: Path):
    bad = {
        "estimator": {"alpha": 1.5},
    }
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigRejectsNegativeWheelRadius(tmp_path: Path):
    bad = {"chassis": {"wheelRadius": -0.01}}
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigRejectsBadEncoderSign(tmp_path: Path):
    bad = {"motors": {"rightEncoderSign": 0}}
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigRejectsBadRightAuxEncoderSign(tmp_path: Path):
    bad = {"motors": {"rightAuxEncoderSign": 0}}
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigRejectsDuplicateMotorPorts(tmp_path: Path):
    bad = {"motors": {"leftPort": "B", "rightPort": "F", "rightAuxPort": "F"}}
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigRejectsBadTiltAxis(tmp_path: Path):
    bad = {"imu": {"tiltAxis": "yaw"}}
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigRejectsNegativeGravityCompGain(tmp_path: Path):
    bad = {"controller": {"gravityCompGain": -0.1}}
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigRejectsBadControllerDeadband(tmp_path: Path):
    bad = {"controller": {"thetaDotDeadband": -0.1}}
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigRejectsNegativeActuatorTau(tmp_path: Path):
    bad = {"controller": {"actuatorTau": -0.1}}
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigRejectsUnsupportedControllerAlgorithm(tmp_path: Path):
    bad = {"controller": {"algorithm": "mystery"}}
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigRejectsNegativePidGain(tmp_path: Path):
    bad = {"controller": {"pidKd": -1.0}}
    badPath = tmp_path / "bad.yaml"
    badPath.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        LoadConfig(path=badPath)


def test_LoadConfigMissingFileRaises(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        LoadConfig(path=tmp_path / "does_not_exist.yaml")
