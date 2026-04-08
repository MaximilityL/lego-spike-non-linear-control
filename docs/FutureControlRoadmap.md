# Future Control Roadmap

This document is the bridge between the current scaffold and a real balancing controller.
The canonical controller module is now `src/LegoBalance/NonLinearController.py`. The old
`LyapunovController.py` name remains only as a compatibility alias.

The first intended implementation is still Lyapunov-based. The broader module name exists
so the repo can host other nonlinear designs later without another rename.

## 1. Implemented State And Plant Convention

The current estimator produces:

```text
x = [theta, thetaDot, phi, phiDot]^T
```

where:

- `theta` is body tilt around the wheel axis, in radians, with `theta = 0` upright.
- `thetaDot` is body tilt rate in rad/s.
- `phi` is mean wheel rotation in radians after encoder sign correction.
- `phiDot` is mean wheel rotation rate in rad/s.

The translational pair `p` and `pDot` is derived only when needed:

```text
p = r * phi
pDot = r * phiDot
```

That choice is deliberate. It keeps the core estimator close to the encoders and avoids
hard-wiring the control state to a wheel-radius calibration before that value is trusted.

## 2. Control Objective

The balancing controller should stabilize the upright equilibrium while prioritizing body
tilt over wheel motion:

- Drive `theta -> 0`.
- Drive `thetaDot -> 0`.
- Optionally regulate `phi` / `phiDot`, or the derived `p` / `pDot`, with weaker weight.
- Return symmetric left/right commands for pure sagittal balance. Yaw control is out of
  scope for now.

The output contract is already fixed by `ControlOutput`:

```text
u = wheel velocity command (rad/s), torque proxy, or duty command
```

`ControlMode.Velocity` is the easiest first target because Pybricks `Motor.run` already
provides the inner motor loop.

## 3. Recommended First Design Path

1. Verify the signs, units, and chassis parameters in `configs/Default.yaml`.
2. Derive or identify the planar inverted-pendulum model around the upright equilibrium.
3. Start with a baseline stabilizer on the implemented state.
4. If the design is Lyapunov-based, a common first pass is:

```text
V(x) = (1/2) x^T P x
u = -K x
```

with `P` positive definite and `K` chosen so the closed-loop derivative is negative in a
useful operating region.

5. Implement the law inside `NonLinearController.Compute`.
6. Add saturation explicitly before returning a command.
7. Validate on the desktop before considering hub deployment.

Nothing in the scaffold forces Lyapunov specifically, but the current placeholder
docstring and this roadmap both assume that is the most likely first implementation.

## 4. Where The Real Code Plugs In

- **Estimator.** `StateEstimator.Update(measurement)` already returns the controller input.
- **Controller.** `NonLinearController.Compute(state)` is the one method that needs the
  real balancing law.
- **Safety.** `SafetyMonitor.Check(state, control, currentTime=...)` already gates the
  result before it reaches the motors.
- **Simulation.** `examples/ClosedLoopSimulation.py` already wires estimator, controller,
  safety, logging, and mock hardware together.
- **Hub deployment.** The current package-backed hub smoke path still uses
  `DriveCommandController`. A real balancing deployment will need either a new self-
  contained Pybricks entrypoint or a shared package path that keeps the controller
  MicroPython-safe.

## 5. Validation Sequence

Use this order:

1. Unit-test the controller contract in `tests/test_NonLinearController.py`.
2. Run the desktop mock loop in `examples/ClosedLoopSimulation.py`.
3. Add or update offline plots and log analysis if the design needs them.
4. Only then move toward on-hub execution.

If the controller is meant to run from the shared package on the hub, keep the
implementation limited to MicroPython-safe features: plain `math`, small lists, and
small matrix code instead of desktop-only dependencies such as `numpy`.

## 6. Work Still Outside The Current Scaffold

- Estimator fusion beyond the current sign-corrected direct state calculation.
- System identification for torque constants and friction.
- Yaw control, steering, or trajectory tracking.
- Online bias estimation.

Those are all valid future steps, but they are not prerequisites for landing the first
balancing controller.
