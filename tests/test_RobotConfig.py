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
    assert config.motors.leftPort == "B"
    assert config.motors.rightPort == "A"
    assert config.motors.forwardSign == -1
    assert config.motors.leftEncoderSign == 1
    assert config.motors.rightEncoderSign == -1
    assert config.motors.maxAngularRate == pytest.approx(17.4532925199)
    assert config.imu.zeroOffset == pytest.approx(-0.872665)
    assert 0.0 < config.estimator.alpha < 1.0
    assert config.control.maxTilt > 0
    assert config.control.maxTilt == pytest.approx(2.0)
    assert config.control.maxWheelRate == pytest.approx(17.44)
    assert config.control.targetTilt == pytest.approx(0.0)
    assert config.drive.loopPeriodMs == 20
    assert config.drive.printEveryN == 1
    assert config.drive.stopDurationMs == 50
    assert config.drive.driveDurationMs == 5000
    assert config.drive.testSpeed == pytest.approx(17.44)
    assert config.drive.maxTiltForMotion == pytest.approx(0.8726646259971648)


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
        "drive": {"maxTiltForMotion": 0.5},
    }
    customPath = tmp_path / "custom.yaml"
    customPath.write_text(yaml.safe_dump(custom))
    config = LoadConfig(path=customPath)
    assert config.name == "TestBot"
    assert config.chassis.wheelRadius == pytest.approx(0.05)
    assert config.motors.leftPort == "C"
    assert config.estimator.alpha == pytest.approx(0.5)


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


def test_LoadConfigMissingFileRaises(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        LoadConfig(path=tmp_path / "does_not_exist.yaml")
