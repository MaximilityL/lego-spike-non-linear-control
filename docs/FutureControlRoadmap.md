# Future Control Roadmap

This document is the bridge between the current scaffold and a working Lyapunov based
balancing controller. It is intentionally honest. The current `LyapunovController` is a
placeholder. Filling it in is a real piece of work and this file is the plan.

## 1. Robot Model (Intended)

A planar two wheel inverted pendulum on a flat floor. The body is a rigid pendulum free
to rotate about the wheel axis. The wheels are driven by motors with finite torque.

State vector

```
x = [theta, thetaDot, phi, phiDot]^T
```

where

- `theta` is the body tilt angle around the wheel axis, with theta = 0 being upright.
- `thetaDot` is the body tilt rate.
- `phi` is the (mean) wheel rotation angle.
- `phiDot` is the (mean) wheel rotation rate.

Control input

```
u = wheel torque (Nm) or wheel angular velocity command (rad/s)
```

The scaffold lets you choose either by setting `ControlOutput.mode` to `torque` or
`velocity`. The latter is easier to bring up because Pybricks `Motor.run` already
implements an inner velocity loop.

## 2. Linearization (For Sanity Checking Only)

Around the upright equilibrium `x = 0` the linearized model has the structure

```
xDot = A x + B u
```

with

- A nontrivial coupling between `theta` and `phi` because the body acts as a pendulum
  reaction load on the wheels.
- A `g/h` term in A coming from gravity, where `g` is gravitational acceleration and
  `h` is the height of the center of mass.

You should compute the symbolic A and B for your specific chassis using the geometry in
`configs/Default.yaml` before designing the controller. The scaffold does not do this
for you because the values depend on your build.

## 3. Lyapunov Approach Outline

A defensible candidate Lyapunov function for a two wheel balancer is

```
V(x) = (1/2) x^T P x
```

with `P` symmetric positive definite, optionally augmented with an integral term in
`phi` if you want zero steady state position drift. The control law is then chosen so
that

```
dV/dt = x^T P (A x + B u) <= -alpha V(x)
```

for some `alpha > 0` along trajectories of the closed loop system. The
`LyapunovController` placeholder documents this objective in its docstring and exposes
the right method shape for the future implementation.

A reasonable design pipeline:

1. Identify the linearized model from chassis parameters.
2. Solve a continuous time algebraic Lyapunov equation `A^T P + P A = -Q` for some
   symmetric positive definite `Q` chosen to penalize tilt strongly.
3. Pick `u = -K x` with K from any standard linear method (LQR is fine here).
4. Verify offline that with this `u`, `dV/dt < 0` along the closed loop trajectories
   in a reasonable region.
5. Optionally extend to a nonlinear Lyapunov design once the linear baseline works on
   real hardware.

The scaffold is structured so that step 1 through step 4 can happen entirely on the
desktop, against `examples/ClosedLoopSimulation.py`, before any hub deployment.

## 4. What Plugs In Where

- **State estimator.** `StateEstimator.Update(measurement, dt)` returns a
  `BalanceState`. Today it is a stub that just forwards measurements. Replace its body
  with a complementary filter (or a Kalman filter if you prefer).
- **Controller.** `LyapunovController.Compute(state)` returns a `ControlOutput`. Today
  it returns zero. Replace its body with the Lyapunov based control law from section 3.
- **Safety.** `SafetyMonitor.Check(state, control)` is already wired and will gate the
  command before it reaches the motors.
- **Deployment.** Once the controller works in `ClosedLoopSimulation.py`, copy the math
  into a single self contained Pybricks program in `hub/`. The function bodies should be
  identical, with imports replaced by the Pybricks equivalents.

## 5. What Is Out Of Scope For This Repo

- Identifying motor torque constants from data (you can add a script later).
- Online parameter estimation.
- Path following or trajectory tracking.

These can be built on top of this scaffold. They should not block the bring up.
