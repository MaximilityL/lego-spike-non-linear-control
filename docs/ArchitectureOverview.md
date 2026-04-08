# Architecture Overview

This document explains how the pieces of the project fit together. Read it after the
top level `README.md`.

## 1. Two Halves

There are two distinct execution environments.

- **Desktop side.** Standard CPython 3.10 or newer. Lives entirely under
  `src/LegoBalance/`, `tests/`, `examples/`, and `scripts/`. Has access to the full
  Python ecosystem, type hints, dataclasses, pytest, ruff, mypy.
- **Hub side.** MicroPython under Pybricks, running on the SPIKE Prime hub. Most hub
  scripts live under `hub/` and are self contained. The package-backed drive smoke
  entrypoint lives at `src/HubPackageDriveSmoke.py` so `pybricksdev` can upload the
  hub-safe `LegoBalance.HubDriveSmokeRuntime` module beside it.

The desktop side is where you design, test, and document. The hub side is where you
actually run code. The normal path keeps the two in sync by convention; the package smoke
path imports only the MicroPython-safe runtime subset.

## 2. The Module Layers

```
                +---------------------------------------------------+
                |              Application or Example                |
                |  examples/ClosedLoopSimulation.py, scripts/...      |
                +---------------------------------------------------+
                                       |
                                       v
+------------------+   +----------------------+   +-------------------+
|  RobotConfig     |   |  Controller          |   |  StateEstimator   |
|  Saturation      |-->|  ControllerBase      |<--|  BalanceState     |
|  Units           |   |  LyapunovController  |   |                   |
+------------------+   +----------------------+   +-------------------+
                                       |
                                       v
                       +------------------------------+
                       |  ControlInterfaces           |
                       |  ControlOutput, Measurement  |
                       +------------------------------+
                                       |
                                       v
+----------------+    +------------------+    +-----------------+
|  HubInterface  |--->|  MotorInterface  |    |  ImuInterface   |
+----------------+    +------------------+    +-----------------+
        |                     |                       |
        v                     v                       v
   MockHub /             MockMotor /             MockImu /
   PybricksHub*         PybricksMotor*           PybricksImu*

(* lives in `hub/` because it imports from `pybricks.*`)
```

## 3. Data Flow For A Single Control Step

1. Each subsystem polls the `HubInterface` for fresh measurements.
2. Measurements are wrapped in a `Measurement` dataclass and handed to the
   `StateEstimator`.
3. The estimator returns a `BalanceState` dataclass containing tilt, tilt rate, wheel
   position, and wheel velocity.
4. The `Controller` consumes the `BalanceState` and returns a `ControlOutput` containing
   the wheel torque or wheel rate command.
5. `SafetyMonitor.Check(state, control)` either passes the command through or replaces
   it with a stop command.
6. The `MotorInterface` applies the (possibly safety filtered) command.
7. The `DataLogger` records everything for offline analysis.

## 4. Why Abstract Interfaces

Inverted pendulum balancing depends on getting many small things right at the same
time. To debug it you need to be able to:

- Replay a logged state into the controller without hardware.
- Swap a real IMU for a fake one that produces a known signal.
- Test the safety monitor with synthetic dangerous inputs.
- Run the entire control loop in simulation before deploying.

Abstract interfaces make all of this trivial. The desktop code never imports from
`pybricks.*`. It always talks through `HubInterface`, `MotorInterface`, and
`ImuInterface`. In tests we inject mocks. On the hub, the equivalent functions are
usually implemented inline against the real Pybricks API in scripts under `hub/`; the
package-backed smoke test imports the hub-safe runtime subset instead.

## 5. Why Not Use One Big Class

Two reasons.

- **Testability.** A monolithic balance class would be impossible to test without
  hardware.
- **Future extensibility.** The user has stated that the long term target is a Lyapunov
  based controller. That controller needs a clean place to plug in. The `ControllerBase`
  abstraction is exactly that place.

## 6. Where The Lyapunov Controller Will Live

It lives in `src/LegoBalance/LyapunovController.py`. The class already exists with
documented method signatures and a fully described mathematical objective. The body of
`Compute` is intentionally a placeholder. See `docs/FutureControlRoadmap.md` for the
plan to fill it in.
