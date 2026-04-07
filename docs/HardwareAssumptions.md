# Hardware Assumptions

This document captures every assumption the code makes about the physical robot.
Whenever you change the chassis or the firmware, re read this file and re verify the
assumptions before running motors at non trivial duty.

## 1. The Hub

- **Hub.** LEGO SPIKE Prime hub (also marketed as the LEGO Education hub).
- **Firmware.** Pybricks firmware, installed once via
  [code.pybricks.com](https://code.pybricks.com).
- **Built in IMU.** The hub has a 6 DOF IMU exposed through `pybricks.hubs.PrimeHub.imu`
  with at least the following methods:
  - `imu.tilt()` returns pitch and roll in degrees.
  - `imu.acceleration()` returns three axis acceleration in mm per s squared.
  - `imu.angular_velocity()` returns three axis gyro rates in degrees per second.
  Sign and axis conventions depend on how the hub is mounted on the chassis. See section
  3 below.

## 2. Motors

- **Type.** Two SPIKE Prime medium or large angular motors.
- **Ports.** Defaults are A for the left wheel and B for the right wheel. Override in
  `configs/Default.yaml`.
- **API.** `pybricks.pupdevices.Motor` exposing at least:
  - `Motor(port).angle()` returns the encoder position in degrees.
  - `Motor(port).speed()` returns the angular speed in deg per second.
  - `Motor(port).run(speed_deg_per_s)` issues a velocity command.
  - `Motor(port).dc(percent)` issues a raw duty command in -100..100.
  - `Motor(port).stop()` releases the motor.
  - `Motor(port).brake()` brakes the motor.

The desktop side `Units` module converts between Pybricks degrees and the SI radians
that the rest of the code uses.

## 3. Mounting And Sign Conventions

The single biggest source of bugs in a balancing robot is a wrong sign. Be explicit.

- **Body coordinate frame.** Right handed. x points forward, y points to the robot's
  left, z points up.
- **Tilt.** Positive tilt means the body is leaning forward. The wheels must roll
  forward (positive wheel angular velocity in the body frame) to recover.
- **IMU axis used for tilt.** The pitch axis returned by `imu.tilt()`. The mapping from
  hub orientation to body pitch depends on whether the hub is mounted upright, on its
  side, or upside down on the chassis. The `imu.tiltAxis` and `imu.tiltSign` fields in
  the config let you fix the mapping without editing code.
- **Wheel encoder direction.** If the motor is mounted facing outward on one side and
  inward on the other, one of the two will report negative angles for forward motion.
  Set this in code in the motor adapter, not in the controller.

## 4. Power And Safety

- The hub batteries can deliver enough current to spin the wheels fast. Always block the
  wheels for the first few runs.
- Pybricks programs can be stopped immediately by pressing the center button on the hub
  or the stop button in the browser editor. Use this. It is your kill switch.
- The desktop side `SafetyMonitor` enforces software level limits but cannot help if the
  hub firmware is unresponsive. Hardware safety is your responsibility.

## 5. What This Scaffold Does Not Assume

- It does not assume any particular wheel diameter. You set this in the config.
- It does not assume any particular center of mass height. You set this in the config.
- It does not assume any particular battery state. The Pybricks API exposes battery
  voltage if you want to log it.
- It does not assume that any specific mounting orientation is correct. You verify it.
