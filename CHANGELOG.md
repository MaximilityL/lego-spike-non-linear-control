# Changelog

All notable changes to this project will be documented in this file.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.3] - 2026-04-08

### Added
- Added a package-backed hardware smoke path that exercises the shared
  `LegoBalance.StateEstimator`, `DriveCommandController`, and `SafetyMonitor` modules on
  the SPIKE hub instead of duplicating that logic inside a standalone hub script.
- Added a laptop plotter for the package-backed smoke run that reuses the existing
  drive-smoke telemetry parser while loading the plotted drive gate from the desktop
  `LegoBalance` config.
- Added a hub-safe default config helper and parity tests that keep its hardware-validated
  signs/constants aligned with the desktop config while proving the shared estimator,
  controller, and safety monitor accept it.

### Changed
- Kept the standalone `hub/HubDriveSmoke.py` as the hardware-verified reference path,
  while documenting the new package-backed path as the deliberate multi-file Pybricks
  exception for testing reusable package logic on the real robot.
- Generalized the drive-smoke plotting title so both standalone and package-backed runs
  can share the same diagnostic plot logic without changing the telemetry contract.
- Removed desktop-only dataclass, enum, and abstract-base-class runtime dependencies from
  the shared state/controller boundary modules so Pybricks can import the same estimator
  and drive controller used on the desktop.

## [1.0.2] - 2026-04-07

### Changed
- Promoted the hardware-verified drive smoke convention into the desktop defaults: left motor on port `B`, right motor on port `A`, `motors.forwardSign = -1`, `1000 deg/s` as `17.4532925199 rad/s`, and a `50 deg` pre-balancing drive gate.
- Locked the package contract around the implemented estimator state `x = [theta, thetaDot, phi, phiDot]`, where `phi` and `phiDot` are mean wheel rotation states and the linear `p` / `pDot` view is derived only when a wheel radius is needed.
- Defined `ControlOutput` in the controller/chassis frame: positive velocity means forward wheel-base motion and increasing `phi`; raw motor sign flips remain at the hardware adapter or hub-script boundary.
- Updated mock measurement conversion so desktop simulations and smoke flows undo the hardware signs before feeding `StateEstimator`, preserving the same state direction as the real hub run.
- Aligned `HubMain.py`, `HubDriveSmoke.py`, `PlotHubDriveSmoke.py`, config docs, and tests with the same hardware-validated signs, limits, and state notation.

### Added
- Added `DriveCommandController` as the non-balancing forward / backward / stop command path used to exercise the estimator + safety + motor pipeline before the upright controller is implemented.
- Added desktop and hub drive-smoke flows plus a post-run diagnostic plot for `[theta, thetaDot, phi, phiDot]`, wheel command, and drive-gate status.
- Added pre-balancing flow tests that prove forward commands produce increasing `phi`, drive-gate violations stop motion without tripping the absolute safety monitor, and the future controller boundary remains shape-compatible.

## [1.0.1] - 2026-04-07

### Changed
- Updated the state estimator to apply `imu.tiltSign`, `imu.zeroOffset`, and `imu.gyroBias` so tilt and tilt rate are corrected by configuration.
- Wheel encoder measurements now use `motors.leftEncoderSign` and `motors.rightEncoderSign`, average the signed motor readings, and convert from wheel angle to linear forward position/velocity using `chassis.wheelRadius` and `motors.forwardSign`.
- Added configuration validation for `motors.forwardSign`, `motors.leftEncoderSign`, and `motors.rightEncoderSign` values.
- Clarified the balance state and controller documentation to use the `[theta, thetaDot, p, pDot]` state representation.

## [1.0.0] - 2026-04-07

### Added
- Initial scaffold for the LEGO SPIKE inverted pendulum project targeting Pybricks.
- Top level project files: `README.md`, `VERSION`, `CHANGELOG.md`, `.gitignore`, `pyproject.toml`.
- Configuration system with a default YAML config under `configs/Default.yaml` and a `RobotConfig` loader.
- Desktop side Python package `LegoBalance` containing modular interfaces:
  - `HubInterface`, `MotorInterface`, `ImuInterface` abstract base classes.
  - `RobotConfig` dataclass and loader.
  - `DataLogger` for in memory and CSV friendly logging.
  - `SafetyMonitor` with tilt, current, and watchdog checks.
  - `StateEstimator` complementary filter scaffold.
  - `BalanceState` dataclass for the future control vector.
  - `ControlInterfaces` and `ControllerBase`.
  - `LyapunovController` placeholder with documented control objective.
  - `ConnectionDiagnostics` and `HardwareTestRunner` skeletons.
  - `Saturation` and `Units` utilities.
  - Mock hub, mock motor, and mock IMU adapters used by tests and the simulation stub.
- Hub side Pybricks programs in `hub/`:
  - `HubMain.py` minimal hub bring up program.
  - `HubMotorTest.py` safe motor smoke test.
  - `HubImuTest.py` IMU streaming program.
  - `HubEncoderTest.py` encoder echo program.
- Diagnostics and bootstrap scripts in `scripts/`:
  - `BootstrapEnv.sh` desktop environment setup helper.
  - `DetectHub.py` USB and Bluetooth presence check helper.
  - `RunDiagnostics.py` desktop side connection diagnostics runner.
- Examples in `examples/`:
  - `ClosedLoopSimulation.py` simulation stub that exercises the controller API without hardware.
  - `MotorSmokeTest.py` desktop side dry run of the motor smoke routine using mocks.
  - `ReadSensors.py` mock sensor read example.
- Test suite in `tests/` covering pure Python logic with `pytest`:
  - Config loading, saturation, units, state estimator shapes, controller interface,
    safety monitor, balance state, and connection diagnostics with mocks.
- Documentation in `docs/`:
  - `ArchitectureOverview.md`, `HardwareAssumptions.md`, `FutureControlRoadmap.md`,
    `TestStrategy.md`, `PybricksNotes.md`.

### Notes
- This release is a scaffold. The Lyapunov controller is intentionally a placeholder.
- No hardware behavior is claimed beyond what the listed Pybricks APIs document.
