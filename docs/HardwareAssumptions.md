# Hardware Assumptions

This document records the physical assumptions the code makes about the robot.
Whenever the chassis, motor mounting, or hub orientation changes, verify these
assumptions again before running aggressive balance tests.

## 1. Hub

- **Hub**: LEGO SPIKE Prime hub.
- **Firmware**: Pybricks, installed through [code.pybricks.com](https://code.pybricks.com).
- **IMU API**: `PrimeHub.imu` provides tilt and angular velocity data used by the
  estimator.

The code assumes the hub is mounted consistently enough that one IMU tilt axis
tracks body lean and can be corrected through:

- `imu.tiltAxis`
- `imu.tiltSign`
- `imu.zeroOffset`
- `imu.gyroBias`

## 2. Motors

- **Type**: two SPIKE angular motors.
- **Default ports**: left motor on `B`, right motor on `F`.
- **Command path**: wheel velocity commands are typically applied through
  `Motor.run(...)` after converting rad/s to deg/s.

The balance controllers do not contain motor-mounting sign logic. That is kept
in config through:

- `motors.forwardSign`
- `motors.leftEncoderSign`
- `motors.rightEncoderSign`

## 3. Sign Conventions

These conventions are used throughout the repository:

- positive `theta`: body leans forward,
- positive `thetaDot`: forward lean is increasing,
- positive `phi`: wheel base has rolled forward,
- positive controller command: wheels should drive forward.

Keeping these conventions consistent is more important than any specific choice
of sign. The code assumes the config has already been adjusted so these meanings
are true.

## 4. State Definition

The implemented estimator state is:

```text
x = [theta, thetaDot, phi, phiDot]
```

The project intentionally uses wheel rotation as the primary wheel-motion state.
Linear travel is derived only when needed through the wheel radius.

## 5. Safety And Power

- the hub and motors can move the wheels fast enough to be dangerous on a bench,
- first runs should be done with the robot restrained or the wheels lifted,
- the hub center button and the Pybricks stop button are treated as kill switches,
- software safety is helpful but cannot replace careful bench procedure.

## 6. Configuration Is The Source Of Truth

- `configs/Default.yaml` is the desktop-side source of truth,
- `LegoBalance.HubDriveSmokeRuntime` is the generated hub-safe mirror used by the
  package-backed hub entrypoints,
- self-contained scripts under `hub/` should be kept aligned with those values.

## 7. What The Project Does Not Assume

- It does not assume perfect wheel-radius accuracy.
- It does not assume perfect center-of-mass estimates.
- It does not assume the hub orientation is universally correct across builds.
- It does not assume battery condition is constant between runs.
