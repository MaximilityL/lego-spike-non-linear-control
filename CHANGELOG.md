# Changelog

All notable changes to this project will be documented in this file.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
