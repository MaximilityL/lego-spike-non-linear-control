"""Hub-safe defaults for the package-backed drive smoke test.

The real estimator, controller, safety monitor, state, and unit conversions
come from the normal ``LegoBalance`` modules. This module only provides a
small Pybricks-friendly config object because the normal ``RobotConfig`` loader
reads YAML from the desktop filesystem.
"""


class ChassisConfig:
    def __init__(self):
        self.wheelRadius = 0.0285
        self.wheelBase = 0.12
        self.bodyMass = 0.5
        self.bodyHeightCoM = 0.08
        self.bodyInertia = 0.002


class MotorsConfig:
    def __init__(self):
        self.leftPort = "B"
        self.rightPort = "A"
        self.maxAngularRate = 17.453292519943295
        self.maxDuty = 80.0
        self.encoderCountsPerRev = 360
        self.forwardSign = -1
        self.leftEncoderSign = 1
        self.rightEncoderSign = -1


class ImuConfig:
    def __init__(self):
        self.tiltAxis = "pitch"
        self.tiltSign = -1
        self.zeroOffset = -1.0471975511965976
        self.gyroBias = 0.0


class EstimatorConfig:
    def __init__(self):
        self.filterType = "complementary"
        self.alpha = 0.98
        self.loopRate = 100.0


class ControlConfig:
    def __init__(self):
        self.loopRate = 100.0
        self.maxTilt = 1.0
        self.maxTiltRate = 10.0
        self.maxWheelRate = 17.453292519943295
        self.watchdogTimeout = 0.2


class DriveConfig:
    def __init__(self):
        self.testSpeed = 17.453292519943295
        self.maxTiltForMotion = 0.8726646259971648


class RobotConfig:
    def __init__(self):
        self.name = "LegoBalance Mk0"
        self.description = "Hub-safe package drive smoke config"
        self.chassis = ChassisConfig()
        self.motors = MotorsConfig()
        self.imu = ImuConfig()
        self.estimator = EstimatorConfig()
        self.control = ControlConfig()
        self.drive = DriveConfig()


def DefaultConfig():
    return RobotConfig()
