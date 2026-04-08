# Changelog

All notable changes to this project will be documented in this file.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.2.1] - 2026-04-08

### Changed
- Aligned the standalone `hub/HubMain.py` IMU zero-offset with the current
  hardware calibration so the direct hub telemetry/readout path matches the
  latest measured upright reference instead of reporting against the older
  `-60 deg` offset.
- Kept this as a small follow-up release on top of `1.2.0`: the package-backed
  real balance path remains runnable and observable, while the standalone
  `HubMain.py` sensor bring-up path now reflects the same current calibration
  intent used during ongoing balance tuning.

## [1.2.0] - 2026-04-08

### Added
- Added a real package-backed hub balancing entrypoint in
  `src/HubPackageBalance.py` that runs the shared `StateEstimator`,
  `NonLinearController`, and `SafetyMonitor` on the SPIKE hub instead of the
  desktop mock plant.
- Added a laptop-side post-run telemetry plotter for the real balance run and
  a live plotting wrapper that regenerates the hub-safe runtime config from
  YAML, launches the hub program, and streams real telemetry into live plots
  of reference vs tilt, the implemented state, and wheel command.
- Added a desktop closed-loop balance plotter so the same shared controller
  path can be exercised and visualized off-hardware when needed.

### Changed
- Extended the control config with a YAML-driven `targetTilt` reference so the
  package-backed balance run can request an explicit tilt target without
  moving the controller boundary or hard-coding the reference inside the hub
  script.
- Made the shared `NonLinearController` import path friendlier to the
  package-backed Pybricks runtime by removing the desktop-only `__future__`
  import and other avoidable runtime dependencies that blocked hub execution.
- Reached the first real hardware integration milestone for balancing: the
  package-backed hub path now runs the real estimator, controller, safety
  monitor, telemetry, and plotting flow end to end, but the current control
  law is not yet physically stabilizing the robot on hardware. This release is
  about making the real loop observable and tunable, not claiming closed-loop
  stability.

## [1.1.1] - 2026-04-08

### Changed
- Replaced the balancing-controller placeholder with a real sagittal
  balancing law in `NonLinearController`: a velocity-mode,
  sliding-mode-inspired outer-loop controller acting on the implemented state
  `[theta, thetaDot, phi, phiDot]`.
- Defined the controller logic around a composite sliding variable
  `sigma = thetaDot + lambdaTheta*theta + lambdaPhiDot*phiDot + lambdaPhi*phi`
  and a bounded command that combines smooth state feedback with a
  boundary-layer saturation term instead of a discontinuous `sign(...)`
  switch, reducing jitter on sampled LEGO hardware.
- Locked the controller contract around practical runtime behavior: invalid
  states stop immediately, valid commands preserve timestamps, left and right
  wheel commands remain symmetric, positive forward lean commands positive
  forward wheel motion, and controller output is intentionally clamped to
  `control.maxWheelRate` before reaching the safety monitor.
- Upgraded the controller tests from placeholder checks to real logic checks,
  covering invalid-state stop behavior, symmetry, output saturation, reset
  semantics, placeholder status, and forward-lean sign sanity.
- Brought the typed config defaults, shipped YAML, generated
  `HubDriveSmokeRuntime`, and related tests back into agreement on the
  hardware-validated chassis values, wheel-rate limit, drive test speed, and
  pre-balancing motion gate so the full desktop test suite reflects the
  current repo truth.

## [1.1.0] - 2026-04-08

### Added
- Added `NonLinearController` as the canonical balancing-controller module and
  preserved `LyapunovController` as a compatibility alias so existing imports
  still work during the transition.
- Added `docs/NonLinearControllerDesignGuide.md` as a controller handoff
  document describing the control boundary, the expected state estimate,
  command conventions, and how the controller is meant to interact with the
  estimator, safety monitor, and motor path.

### Changed
- Reframed the balancing-controller API around the implemented runtime logic:
  `Measurement -> StateEstimator.Update -> NonLinearController.Compute ->
  SafetyMonitor.Check -> motor command`.
- Updated the architecture, roadmap, test-strategy, hardware, Pybricks, and
  top-level docs so they describe the actual implemented state
  `[theta, thetaDot, phi, phiDot]`, the derived `p` / `pDot` view, and the
  controller/chassis sign conventions instead of leaving the controller as an
  abstract future placeholder.
- Switched the desktop closed-loop simulation and controller-facing tests to
  the new `NonLinearController` name while keeping behavior unchanged: the
  controller remains an explicit placeholder that returns zero commands until a
  real balancing law is implemented.

## [1.0.4] - 2026-04-08

### Changed
- Made the package-backed hub smoke flow configuration-driven end to end. The
  package runtime generator now copies the values from `configs/Default.yaml`
  directly into `LegoBalance.HubDriveSmokeRuntime`, instead of rejecting local
  edits that differ from a previously hard-coded smoke-test profile.
- Moved the package smoke timing knobs into the `drive` config section so the
  YAML now controls the hub loop period, telemetry cadence, stop leg duration,
  forward/backward leg duration, test speed, and motion gate without editing
  `src/HubPackageDriveSmoke.py`.
- Updated the typed config defaults and parity tests to follow the current YAML
  values, keeping the desktop loader, generated hub runtime, and package smoke
  path aligned around the same source of truth.
- Increased the shared drive-smoke plot typography and figure size so the popup
  plots are easier to read for both the standalone and package-backed smoke
  plotters.

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
- Added a generator for the hub-safe drive-smoke config so `configs/Default.yaml` remains
  the source of truth even though the SPIKE hub cannot parse YAML directly.

### Changed
- Kept the standalone `hub/HubDriveSmoke.py` as the hardware-verified reference path,
  while documenting the new package-backed path as the deliberate multi-file Pybricks
  exception for testing reusable package logic on the real robot.
- Generalized the drive-smoke plotting title so both standalone and package-backed runs
  can share the same diagnostic plot logic without changing the telemetry contract.
- Removed desktop-only dataclass, enum, and abstract-base-class runtime dependencies from
  the shared state/controller boundary modules so Pybricks can import the same estimator
  and drive controller used on the desktop.
- Made the package-backed plotter regenerate `LegoBalance.HubDriveSmokeRuntime` from
  `configs/Default.yaml` before live upload, and fail early if the config no longer
  matches the hardware-verified smoke constants.
- Kept the hub import path MicroPython-safe by removing unsupported `__future__`,
  `property`, `staticmethod`, relative import, and `math` dependencies from the modules
  uploaded by `src/HubPackageDriveSmoke.py`.

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
