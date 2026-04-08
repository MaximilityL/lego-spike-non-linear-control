# Architecture Overview

This repo is structured so that the balancing controller is the only hard part left to
implement. The configuration loader, state boundary objects, estimator, safety layer,
mock hardware, and hub smoke paths already exist.

## 1. Two Execution Environments

- **Desktop side.** Standard CPython. Most design work happens under
  `src/LegoBalance/`, `tests/`, `examples/`, and `scripts/`. This is where you model,
  test, document, and iterate quickly.
- **Hub side.** Pybricks MicroPython on the SPIKE Prime hub. Most files under `hub/`
  are intentionally self contained. The one package-backed exception is
  `src/HubPackageDriveSmoke.py`, which imports a MicroPython-safe subset of the
  `LegoBalance` package plus a generated config helper.

The key architectural rule is simple: desktop logic must stay decoupled from
`pybricks.*`, and hub entrypoints must stay honest about what can actually run on the
device.

## 2. Core Modules

| Layer | Main modules | Responsibility |
| --- | --- | --- |
| Config and units | `RobotConfig`, `Units`, `Saturation` | Typed config, unit conversion, and command clamping helpers |
| Estimation boundary | `ControlInterfaces.Measurement`, `StateEstimator`, `BalanceState` | Convert raw sensor readings into the canonical state |
| Controllers | `ControllerBase`, `DriveCommandController`, `NonLinearController` | Map state to left/right wheel commands |
| Safety | `SafetyMonitor` | Arm, gate, and trip before commands reach the motors |
| Adapters and logging | `MockAdapters`, `HubInterface`, `MotorInterface`, `ImuInterface`, `DataLogger` | Mock hardware, abstract interfaces, and offline analysis |

`src/LegoBalance/LyapunovController.py` still exists only as a backward-compatible alias.
The canonical balancing-controller module is now `src/LegoBalance/NonLinearController.py`.

## 3. State And Command Boundary

The implemented control state is:

```text
x = [theta, thetaDot, phi, phiDot]
```

where:

- `theta` / `tilt` is body pitch in radians.
- `thetaDot` / `tiltRate` is body pitch rate in rad/s.
- `phi` is mean wheel rotation in radians after encoder sign correction.
- `phiDot` is mean wheel rotation rate in rad/s.

The translational pair `p` and `pDot` is intentionally derived, not stored:

```text
p = r * phi
pDot = r * phiDot
```

That keeps the core estimator radius-independent until the wheel radius is trusted.

Commands cross the controller boundary as `ControlOutput`. Positive command means forward
chassis motion / increasing `phi`. The controller does not apply motor mounting signs;
that is handled by config and adapters below it.

## 4. Single Control Step

The runtime loop is the same on desktop and on the hub-safe package path:

1. Read sensors and build a `Measurement` in SI units.
2. Call `StateEstimator.Update(measurement)` to get a `BalanceState`.
3. Call `controller.Compute(state)` to get a `ControlOutput`.
4. Pass the result through `SafetyMonitor.Check(...)`.
5. Apply the safe command to the motors.
6. Log the state and command for offline analysis.

In code, that shape looks like this:

```python
measurement = MakeMeasurement(...)
state = estimator.Update(measurement)
rawCommand = controller.Compute(state)
safeCommand = safety.Check(state, rawCommand, currentTime=state.timestamp)
motors.Apply(safeCommand)
logger.Record(state, safeCommand)
```

That loop is already exercised by `examples/ClosedLoopSimulation.py` and by the
package-backed drive smoke runtime.

## 5. Current Entry Points

- `examples/EstimatorReadout.py`: prints the implemented state without involving a
  controller.
- `examples/DriveCommandSmoke.py`: desktop mock run of the forward/stop/backward smoke
  flow.
- `examples/ClosedLoopSimulation.py`: mock closed loop using `NonLinearController`,
  `StateEstimator`, `SafetyMonitor`, and `MockHub`.
- `hub/HubMain.py`: self-contained Pybricks sensor bring-up and telemetry.
- `hub/HubDriveSmoke.py`: self-contained Pybricks drive smoke script.
- `src/HubPackageDriveSmoke.py`: package-backed hub smoke path that imports shared
  estimator, drive controller, and safety code.
- `scripts/GenerateHubDriveSmokeRuntime.py`: regenerates
  `src/LegoBalance/HubDriveSmokeRuntime.py` from `configs/Default.yaml`.

## 6. Controller Integration Boundary

If someone is implementing the balancing law, the file to edit is
`src/LegoBalance/NonLinearController.py`.

What the rest of the repo guarantees to that class:

- It receives a sign-corrected, SI-unit `BalanceState`.
- It can read physical parameters and limits through `self.config`.
- It can return either velocity, torque, or duty commands via `ControlOutput.mode`.
- It does not need to talk to motors directly.
- It does not need to enforce the final stop/trip logic; `SafetyMonitor` already exists.

For the implementation handoff and interaction contract, see
`docs/NonLinearControllerDesignGuide.md`.
