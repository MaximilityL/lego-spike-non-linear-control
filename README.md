# LEGO SPIKE Inverted Pendulum Control Stack

This repository contains the control stack, hub entrypoints, validation tools,
and experiment notes for a two-wheel self-balancing robot built on the LEGO
SPIKE Prime hub with Pybricks.

The project is no longer just scaffolding. The current codebase includes:

- a measured sign convention recorded in configuration,
- a minimal estimator that produces the implemented state
  `x = [theta, thetaDot, phi, phiDot]`,
- a real tanh-based nonlinear balance controller,
- an alternative PID balance controller for comparison,
- a safety monitor that gates every motor command,
- desktop simulation and plotting tools,
- package-backed and self-contained Pybricks runs on the real hub,
- automated tests around config, estimator, controllers, safety, and runtime flow.

If you are turning this project into a LaTeX report, start with these files:

- [docs/ImplementationReportGuide.md](docs/ImplementationReportGuide.md)
- [docs/ArchitectureOverview.md](docs/ArchitectureOverview.md)
- [docs/NonLinearControllerDesignGuide.md](docs/NonLinearControllerDesignGuide.md)
- [docs/TestStrategy.md](docs/TestStrategy.md)

## 1. What We Built

The implemented system has one clear runtime shape:

```python
measurement = MakeMeasurement(...)
state = estimator.Update(measurement)
rawCommand = controller.Compute(state)
safeCommand = safety.Check(state, rawCommand, currentTime=state.timestamp)
motors.Apply(safeCommand)
logger.Record(state, safeCommand)
```

That loop runs in three useful environments:

- **Desktop closed-loop simulation** through `examples/ClosedLoopSimulation.py`.
- **Package-backed balance run on the real hub** through `src/HubPackageBalance.py`.
- **Self-contained hub smoke and bring-up scripts** under `hub/`.

The main architectural decision is the split between:

- **desktop-side shared logic** in `src/LegoBalance/`, where we keep typed config,
  estimator logic, controllers, safety, tests, and examples, and
- **hub-side Pybricks entrypoints** in `hub/` and `src/HubPackage*.py`, where we
  read real sensors and command real motors.

That split let us iterate on controller logic and tests on a laptop while still
running the same estimator/controller/safety path on the SPIKE hub when needed.

## 2. Implemented State And Sign Convention

The canonical state is:

```text
x = [theta, thetaDot, phi, phiDot]
```

where:

- `theta` is body tilt in radians, with positive meaning the robot leans forward.
- `thetaDot` is body tilt rate in rad/s.
- `phi` is the mean wheel rotation angle in radians after encoder sign correction.
- `phiDot` is the mean wheel rotation rate in rad/s.

The estimator intentionally stores wheel rotation rather than linear travel as the
primary wheel state. Linear position and velocity are derived only when needed:

```text
p = r * phi
pDot = r * phiDot
```

This keeps the core state close to the raw encoder measurements and avoids making
the estimator depend on wheel-radius calibration more than necessary.

The sign corrections are applied in configuration rather than inside the controller:

```text
theta    = tiltSign * rawTilt + zeroOffset
thetaDot = tiltSign * rawTiltRate - gyroBias
phi      = forwardSign * 0.5 * (leftEncoderSign * leftAngle + rightEncoderSign * rightAngle)
phiDot   = forwardSign * 0.5 * (leftEncoderSign * leftRate  + rightEncoderSign * rightRate)
```

That separation matters because it lets the balance controller work only with
sign-corrected SI-unit states.

## 3. Controllers

The controller is selected through `config.controller.algorithm` and built by
`LegoBalance.BalanceControllerFactory.BuildBalanceController`.

### Default controller: `NonLinearController`

The default balance controller is a model-light tanh law:

```text
thetaError = theta - targetTilt
s = kTheta * db(thetaError)
  + kThetaDot * db(thetaDotFiltered)
  - kPhi * phi
  - kPhiDot * phiDot
u = maxWheelRate * tanh(s / sScale)
```

Key design choices:

- `theta` and `thetaDot` dominate because tilt stabilization is the primary job.
- `phi` and `phiDot` are weaker drift-suppression terms.
- `targetTilt` shifts the balance point to compensate for bias and reduce walk-off.
- `thetaDeadband` and `thetaDotDeadband` suppress jitter near upright.
- `thetaDotFilterAlpha` optionally smooths gyro noise.
- `tanh` gives a smooth bounded command without abrupt mode switching.
- hard saturation is still kept as a backstop through `SaturateSymmetric`.

The controller returns symmetric wheel-velocity commands in rad/s, which map
cleanly to Pybricks `Motor.run(...)` after unit conversion.

### Alternative controller: `PidController`

The repo also includes a discrete PID controller modeled on the common LEGO
self-balancing example structure. It is useful as a comparison baseline and can
be selected by setting:

```yaml
controller:
  algorithm: "pid"
```

Both balance controllers obey the same `ControlOutput` contract, so swapping
between them does not change the rest of the runtime loop.

## 4. Safety Layer

`LegoBalance.SafetyMonitor` is the final gate before a command reaches the motors.
It is disarmed by default and must be armed explicitly by the application.

The safety monitor:

- trips on invalid or non-finite state,
- trips on excessive tilt or tilt rate,
- trips on watchdog timeout,
- clips commands to the configured wheel-rate ceiling,
- substitutes a stop command whenever the system is disarmed or tripped.

For pre-balancing drive-smoke tests it also exposes a tighter soft gate through
`IsTiltSafeForDriveMotion(...)` so we can refuse motion before reaching the full
trip threshold.

## 5. Repository Layout

```text
lego-spike-non-linear-control/
├── README.md
├── CHANGELOG.md
├── VERSION
├── configs/
│   ├── Default.yaml
│   └── README.md
├── docs/
│   ├── ArchitectureOverview.md
│   ├── FutureControlRoadmap.md
│   ├── HardwareAssumptions.md
│   ├── ImplementationReportGuide.md
│   ├── NonLinearControllerDesignGuide.md
│   ├── PybricksNotes.md
│   └── TestStrategy.md
├── examples/
│   ├── ClosedLoopSimulation.py
│   ├── DriveCommandSmoke.py
│   ├── EstimatorReadout.py
│   ├── MotorSmokeTest.py
│   └── ReadSensors.py
├── hub/
│   ├── HubMain.py
│   ├── HubDriveSmoke.py
│   ├── HubImuTest.py
│   ├── HubEncoderTest.py
│   ├── HubMotorTest.py
│   ├── HubSingleMotorStepResponseF.py
│   └── HubBluetoothTest.py
├── scripts/
│   ├── BootstrapEnv.sh
│   ├── DeployToHub.md
│   ├── GenerateHubDriveSmokeRuntime.py
│   ├── PlotClosedLoopBalance.py
│   ├── PlotHubMainLive.py
│   ├── PlotHubPackageBalance.py
│   ├── PlotHubPackageDriveSmoke.py
│   ├── PlotHubSingleMotorStepResponse.py
│   ├── PlotHubDriveSmoke.py
│   └── RunDiagnostics.py
├── src/
│   ├── HubPackageBalance.py
│   ├── HubPackageDriveSmoke.py
│   └── LegoBalance/
└── tests/
```

## 6. Quick Start

### Desktop setup

```bash
bash scripts/BootstrapEnv.sh
source .venv/bin/activate
pip install -e .[dev]
```

### Run the test suite

```bash
pytest
```

### Run the desktop closed-loop balance simulation

```bash
python examples/ClosedLoopSimulation.py
python scripts/PlotClosedLoopBalance.py
```

### Run the package-backed balance controller on the real hub

```bash
python scripts/GenerateHubDriveSmokeRuntime.py
python scripts/PlotHubPackageBalance.py
```

The plotter launches `src/HubPackageBalance.py`, captures its telemetry rows,
and saves a diagnostic PNG under `plots/`.

## 7. Running The Main Workflows

### Sensor bring-up and sign checks

```bash
pybricksdev run ble hub/HubMain.py
python scripts/PlotHubMainLive.py
```

### Package-backed real balance run

```bash
python scripts/GenerateHubDriveSmokeRuntime.py
pybricksdev run ble src/HubPackageBalance.py
```

or, with automatic capture and plotting:

```bash
python scripts/PlotHubPackageBalance.py
```

### Self-contained hub drive smoke

```bash
pybricksdev run ble hub/HubDriveSmoke.py
python scripts/PlotHubDriveSmoke.py
```

### Single-motor step response on Port F

```bash
pybricksdev run ble hub/HubSingleMotorStepResponseF.py
python scripts/PlotHubSingleMotorStepResponse.py
```

### Package-backed drive smoke

```bash
python scripts/PlotHubPackageDriveSmoke.py
```

## 8. Validation

The codebase is validated in layers:

- pure Python tests for config loading, units, saturation, and state containers,
- estimator tests for sign handling and the `phi`/`phiDot` state definition,
- controller tests for both nonlinear and PID controllers,
- safety tests for arming, limits, watchdog behavior, and command clipping,
- end-to-end mock tests for the estimator -> controller -> safety -> logger path,
- runtime-generation tests that keep the hub-safe config helper consistent with
  `configs/Default.yaml`.

For the practical test workflow, see [docs/TestStrategy.md](docs/TestStrategy.md).

## 9. Current Limits

This repository has a real balance controller, but it is still an experimental
robotics project rather than a finished product.

Current limits to keep in mind:

- hardware tuning is still empirical,
- the estimator is deliberately minimal and does not yet fuse sensors,
- yaw control and trajectory tracking are out of scope,
- wheel slip, battery variation, and timing jitter still matter on the real robot,
- the generated hub-safe config helper still has the historical filename
  `HubDriveSmokeRuntime.py` even though it now feeds package-backed balance runs too.

## 10. Docs Map

- [docs/ImplementationReportGuide.md](docs/ImplementationReportGuide.md): report-friendly
  summary and suggested LaTeX structure.
- [docs/ArchitectureOverview.md](docs/ArchitectureOverview.md): module boundaries,
  data flow, and execution paths.
- [docs/NonLinearControllerDesignGuide.md](docs/NonLinearControllerDesignGuide.md):
  implemented tanh controller design and tuning logic.
- [docs/TestStrategy.md](docs/TestStrategy.md): automated and manual validation plan.
- [docs/HardwareAssumptions.md](docs/HardwareAssumptions.md): robot-specific physical
  assumptions and sign conventions.
- [docs/PybricksNotes.md](docs/PybricksNotes.md): Pybricks-specific operational notes.
- [scripts/DeployToHub.md](scripts/DeployToHub.md): practical hub deployment reference.

## 11. License

MIT.
