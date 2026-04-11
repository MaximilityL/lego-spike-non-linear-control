# Architecture Overview

This project is organized around one idea: the estimator, controller, safety
logic, and validation tools should be easy to develop on a laptop, but the same
runtime shape should still execute honestly on the LEGO SPIKE Prime hub.

## 1. Control Objective

The system controls a two-wheel inverted pendulum. The immediate control goal is:

- stabilize the body around upright,
- keep the wheels from drifting away indefinitely,
- enforce software safety limits before motor commands are applied.

The canonical state is:

```text
x = [theta, thetaDot, phi, phiDot]
```

with:

- `theta`: body tilt in radians,
- `thetaDot`: body tilt rate in rad/s,
- `phi`: mean wheel rotation in radians,
- `phiDot`: mean wheel rotation rate in rad/s.

Linear travel is derived only when needed:

```text
p = r * phi
pDot = r * phiDot
```

## 2. Two Execution Environments

### Desktop side

Most development happens under normal CPython:

- `src/LegoBalance/`
- `tests/`
- `examples/`
- `scripts/`

This side contains the typed config, estimator, controllers, safety layer,
logging, plotting, and mock hardware used for regression testing.

### Hub side

Real hardware runs under Pybricks MicroPython on the SPIKE hub:

- self-contained bring-up and smoke scripts live under `hub/`,
- package-backed balance and drive runs live under `src/HubPackageBalance.py`
  and `src/HubPackageDriveSmoke.py`.

The package-backed entrypoints import a MicroPython-safe subset of the shared
`LegoBalance` package, plus a generated config helper from
`LegoBalance.HubDriveSmokeRuntime`.

## 3. Why The Split Matters

This split gave the project three practical advantages:

1. We can test most logic without a hub connected.
2. We can keep hardware-specific sign handling out of the controller itself.
3. We can reuse the same estimator/controller/safety pipeline in desktop and
   hub runs instead of maintaining two different control stacks.

## 4. Main Modules

| Layer | Main modules | Responsibility |
| --- | --- | --- |
| Configuration | `RobotConfig`, `configs/Default.yaml`, `HubDriveSmokeRuntime` | Load, validate, and mirror robot parameters |
| State boundary | `ControlInterfaces.Measurement`, `BalanceState`, `StateEstimator` | Convert raw measurements into the canonical state |
| Balance control | `NonLinearController`, `PidController`, `BalanceControllerFactory` | Compute symmetric wheel commands from the estimated state |
| Pre-balance motion | `DriveCommandController` | Run forward/backward/stop smoke paths through the same runtime loop |
| Safety | `SafetyMonitor` | Arm, trip, watchdog, and clamp commands |
| Hardware abstraction | `MockAdapters`, `HubInterface`, `MotorInterface`, `ImuInterface` | Isolate the hardware boundary from control logic |
| Logging and analysis | `DataLogger`, plotting scripts under `scripts/` | Record runs and produce offline plots |

## 5. State And Command Boundaries

The estimator applies all sign and calibration corrections before the controller
sees the state:

```text
theta    = tiltSign * rawTilt + zeroOffset
thetaDot = tiltSign * rawTiltRate - gyroBias
phi      = forwardSign * mean(sign-corrected wheel angles)
phiDot   = forwardSign * mean(sign-corrected wheel rates)
```

That means every balance controller receives:

- SI units only,
- corrected signs only,
- no direct dependency on Pybricks APIs,
- no raw degree-based sensor conventions.

Commands leave the controller as a `ControlOutput`:

```text
u = [leftCommand, rightCommand, mode, timestamp]
```

For pure sagittal balance the left and right commands are equal. Positive
command always means forward chassis motion.

## 6. One Control Step

The common control loop is:

```python
measurement = MakeMeasurement(...)
state = estimator.Update(measurement)
rawCommand = controller.Compute(state)
safeCommand = safety.Check(state, rawCommand, currentTime=state.timestamp)
motors.Apply(safeCommand)
logger.Record(state, safeCommand)
```

This same shape appears in:

- `examples/ClosedLoopSimulation.py`,
- `src/HubPackageBalance.py`,
- `src/HubPackageDriveSmoke.py`.

That reuse is one of the strongest architectural features in the repo. It means
the project tests the same boundaries on desktop and hardware instead of only
testing isolated functions.

## 7. Controller Selection

`LegoBalance.BalanceControllerFactory.BuildBalanceController(config)` selects the
active balance controller from `config.controller.algorithm`.

Supported options are:

- `nonlinear` (default): the geometry aware robust nonlinear controller
  with a slow outer recentering loop, a Lyapunov style inner loop with a
  smooth sliding mode robust correction, and a first order actuator lag
  inversion,
- `tanh`: legacy alias kept for backward compatibility; it resolves to the
  same geometry aware law as `nonlinear`,
- `pid`: the discrete PID baseline controller.

Because all options return the same `ControlOutput`, the runtime code does
not change when switching controllers.

## 8. Runtime Paths

### Desktop balance simulation

`examples/ClosedLoopSimulation.py` uses `MockHub` as a toy plant and runs the
same estimator/controller/safety flow as the hardware path. It is useful for
API validation and coarse sanity checks.

### Package-backed real balance run

Two hub entrypoints share the same estimator, selected balance controller,
and safety monitor:

- `src/HubPackageBalance.py`: live path. Telemetry is printed every few
  control iterations so the laptop side plotter can render the run as it
  happens. Per iteration BLE prints couple the control loop to the print
  rate, which is why decimation is used on this path.
- `src/HubPackageBalanceBuffered.py`: buffered path. Every control
  iteration is captured into preallocated packed float32 buffers on the
  hub, and the entire trace is dumped over BLE only after the run ends.
  This removes the print blocking from the 100 Hz control loop and gives
  post run analysis the full unthinned state and command trace. The run
  duration is trimmed (currently 25 s) so the full capture fits in hub
  RAM without decimation.

Companion laptop plotters (`scripts/PlotHubPackageBalance.py` and
`scripts/PlotHubPackageBalanceBuffered.py`) regenerate the hub runtime
mirror from `configs/Default.yaml`, launch the appropriate hub entrypoint
through `pybricksdev`, collect telemetry, and render tilt reference, full
state, and raw versus applied command in a shared layout so live and
buffered runs remain visually comparable.

### Self-contained hub bring-up

Files under `hub/` are minimal Pybricks programs used for fast sensor checks,
encoder checks, and smoke tests without relying on shared package imports.

## 9. Design Tradeoffs

Several design choices are deliberate:

- The estimator is minimal because sign correctness and unit consistency were
  more important than estimator sophistication in the first balancing phase.
- The default nonlinear controller uses chassis geometry for its nominal
  Lyapunov style law but refuses to rely on exact cancellation: a smooth
  sliding mode robust correction absorbs the remaining uncertainty in mass,
  pitch inertia, contact, and the actuator lag time constant. This is the
  deliberate middle ground between "model light, no physics" and "full
  feedback linearization."
- Safety is its own module because safety logic needs a different failure model
  than control logic: it should fail to "stop", not fail silently.
- Package-backed hub runs exist because they test the real shared code path,
  not just a manually copied standalone script.

## 10. What To Cite In A Report

If you are writing a report from this repository, the clean narrative is:

1. configuration and sign verification define a trustworthy state convention,
2. `StateEstimator` maps raw IMU and encoder data into
   `[theta, thetaDot, phi, phiDot]`,
3. the selected controller maps that state into a symmetric wheel-velocity command,
4. `SafetyMonitor` is the final gate before motor actuation,
5. desktop and hub runs share the same logical pipeline.

For a report-friendly outline and equations, see
`docs/ImplementationReportGuide.md`.
