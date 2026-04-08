"""Hub-safe package runtime for the drive smoke test.

This module is intentionally small and MicroPython friendly: no dataclasses,
no pathlib, no yaml, no enum, and no desktop-only imports. It exists so a
Pybricks hub script can import real logic from ``LegoBalance`` instead of
copying the whole estimator/controller/safety path into a standalone file.

The constants mirror ``configs/Default.yaml`` at the hardware-validated
pre-balancing point. The desktop tests compare this module against the normal
``StateEstimator`` / ``DriveCommandController`` / ``SafetyMonitor`` flow so the
hub-safe package path cannot silently drift away.
"""

DEG_TO_RAD = 0.017453292519943295
RAD_TO_DEG = 57.29577951308232


def DegToRad(deg):
    return deg * DEG_TO_RAD


def RadToDeg(rad):
    return rad * RAD_TO_DEG


def DegPerSecToRadPerSec(degPerSec):
    return degPerSec * DEG_TO_RAD


def RadPerSecToDegPerSec(radPerSec):
    return radPerSec * RAD_TO_DEG


def SaturateSymmetric(value, magnitudeBound):
    if magnitudeBound < 0.0:
        raise ValueError("magnitudeBound must be non negative")
    if value > magnitudeBound:
        return magnitudeBound
    if value < -magnitudeBound:
        return -magnitudeBound
    return value


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


class BalanceState:
    def __init__(
        self,
        tilt=0.0,
        tiltRate=0.0,
        phi=0.0,
        phiDot=0.0,
        timestamp=0.0,
        valid=False,
    ):
        self.tilt = tilt
        self.tiltRate = tiltRate
        self.phi = phi
        self.phiDot = phiDot
        self.timestamp = timestamp
        self.valid = valid

    @property
    def theta(self):
        return self.tilt

    @property
    def thetaDot(self):
        return self.tiltRate

    def AsList(self):
        return [self.tilt, self.tiltRate, self.phi, self.phiDot]


class Measurement:
    def __init__(
        self,
        tiltAngle,
        tiltRate,
        leftWheelAngle,
        rightWheelAngle,
        leftWheelRate,
        rightWheelRate,
        timestamp,
        valid=True,
    ):
        self.tiltAngle = tiltAngle
        self.tiltRate = tiltRate
        self.leftWheelAngle = leftWheelAngle
        self.rightWheelAngle = rightWheelAngle
        self.leftWheelRate = leftWheelRate
        self.rightWheelRate = rightWheelRate
        self.timestamp = timestamp
        self.valid = valid


class ControlMode:
    Velocity = "velocity"
    Torque = "torque"
    DutyCycle = "duty"


class ControlOutput:
    def __init__(self, leftCommand, rightCommand, mode, timestamp=0.0):
        self.leftCommand = leftCommand
        self.rightCommand = rightCommand
        self.mode = mode
        self.timestamp = timestamp

    @staticmethod
    def Stop(mode=ControlMode.Velocity, timestamp=0.0):
        return ControlOutput(0.0, 0.0, mode, timestamp)


class DriveCommand:
    Stop = "stop"
    Forward = "forward"
    Backward = "backward"


class StateEstimator:
    def __init__(self, config):
        self.config = config
        self.alpha = config.estimator.alpha
        self.Reset()

    def Reset(self):
        self._lastTimestamp = None
        self._tiltEstimate = 0.0
        self._tiltRateEstimate = 0.0
        self._phiEstimate = 0.0
        self._phiDotEstimate = 0.0

    def Update(self, measurement):
        self._tiltEstimate = (
            self.config.imu.tiltSign * measurement.tiltAngle + self.config.imu.zeroOffset
        )
        self._tiltRateEstimate = (
            self.config.imu.tiltSign * measurement.tiltRate - self.config.imu.gyroBias
        )

        signedLeftAngle = self.config.motors.leftEncoderSign * measurement.leftWheelAngle
        signedRightAngle = self.config.motors.rightEncoderSign * measurement.rightWheelAngle
        signedLeftRate = self.config.motors.leftEncoderSign * measurement.leftWheelRate
        signedRightRate = self.config.motors.rightEncoderSign * measurement.rightWheelRate
        meanWheelAngle = 0.5 * (signedLeftAngle + signedRightAngle)
        meanWheelRate = 0.5 * (signedLeftRate + signedRightRate)
        forwardSign = self.config.motors.forwardSign

        self._phiEstimate = forwardSign * meanWheelAngle
        self._phiDotEstimate = forwardSign * meanWheelRate
        self._lastTimestamp = measurement.timestamp

        return BalanceState(
            tilt=self._tiltEstimate,
            tiltRate=self._tiltRateEstimate,
            phi=self._phiEstimate,
            phiDot=self._phiDotEstimate,
            timestamp=measurement.timestamp,
            valid=True,
        )


class DriveCommandController:
    def __init__(self, config):
        self.config = config
        self._command = DriveCommand.Stop
        self._driveSpeed = float(config.drive.testSpeed)
        if self._driveSpeed < 0.0:
            raise ValueError("drive.testSpeed must be non negative")
        if self._driveSpeed > config.motors.maxAngularRate:
            self._driveSpeed = float(config.motors.maxAngularRate)

    def SetCommand(self, command):
        if command not in (DriveCommand.Stop, DriveCommand.Forward, DriveCommand.Backward):
            raise TypeError("command must be a DriveCommand value")
        self._command = command

    def Forward(self):
        self.SetCommand(DriveCommand.Forward)

    def Backward(self):
        self.SetCommand(DriveCommand.Backward)

    def Stop(self):
        self.SetCommand(DriveCommand.Stop)

    @property
    def command(self):
        return self._command

    @property
    def driveSpeed(self):
        return self._driveSpeed

    def Compute(self, state):
        if not state.valid:
            return ControlOutput.Stop(mode=ControlMode.Velocity, timestamp=state.timestamp)

        if self._command == DriveCommand.Forward:
            magnitude = self._driveSpeed
        elif self._command == DriveCommand.Backward:
            magnitude = -self._driveSpeed
        else:
            magnitude = 0.0

        return ControlOutput(
            leftCommand=magnitude,
            rightCommand=magnitude,
            mode=ControlMode.Velocity,
            timestamp=state.timestamp,
        )

    def Reset(self):
        self._command = DriveCommand.Stop


class SafetyStatus:
    def __init__(self):
        self.armed = False
        self.tripped = False
        self.reasons = []
        self.lastUpdateTime = 0.0


class SafetyMonitor:
    def __init__(self, config):
        self.config = config
        self._status = SafetyStatus()
        self._maxTilt = config.control.maxTilt
        self._maxTiltRate = config.control.maxTiltRate
        self._maxWheelRate = config.control.maxWheelRate
        self._watchdogTimeout = config.control.watchdogTimeout
        self._maxTiltForMotion = config.drive.maxTiltForMotion

    @property
    def status(self):
        return self._status

    def Arm(self, currentTime=0.0):
        self._status.armed = True
        self._status.tripped = False
        self._status.reasons = []
        self._status.lastUpdateTime = currentTime

    def Disarm(self):
        self._status.armed = False

    def Trip(self, reason):
        self._status.tripped = True
        self._status.armed = False
        self._status.reasons.append(reason)

    def Check(self, state, controlOutput, currentTime=None):
        if currentTime is None:
            currentTime = controlOutput.timestamp

        if self._status.armed and self._watchdogTimeout > 0.0:
            elapsed = currentTime - self._status.lastUpdateTime
            if elapsed > self._watchdogTimeout:
                self.Trip("watchdog timeout")

        if not state.valid:
            self.Trip("state estimate not valid")
        if abs(state.tilt) > self._maxTilt:
            self.Trip("tilt exceeds max")
        if abs(state.tiltRate) > self._maxTiltRate:
            self.Trip("tiltRate exceeds max")

        leftClipped = SaturateSymmetric(controlOutput.leftCommand, self._maxWheelRate)
        rightClipped = SaturateSymmetric(controlOutput.rightCommand, self._maxWheelRate)

        if not self._status.armed or self._status.tripped:
            return ControlOutput.Stop(mode=controlOutput.mode, timestamp=currentTime)

        self._status.lastUpdateTime = currentTime
        return ControlOutput(
            leftCommand=leftClipped,
            rightCommand=rightClipped,
            mode=controlOutput.mode,
            timestamp=currentTime,
        )

    def StopCommand(self, mode=ControlMode.Velocity, timestamp=0.0):
        return ControlOutput.Stop(mode=mode, timestamp=timestamp)

    def IsTiltSafeForDriveMotion(self, state):
        if not state.valid:
            return False
        return abs(state.tilt) <= self._maxTiltForMotion
