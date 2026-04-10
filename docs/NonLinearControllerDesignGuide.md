# NonLinearController Design Guide

This document explains the implemented balance controller in
`src/LegoBalance/NonLinearController.py`.

The controller is already real code, not a placeholder. The goal of this note
is to make its design choices, equations, and tuning knobs easy to explain in a
report and easy to revisit during later tuning.

## 1. Design Goal

The controller stabilizes the robot around upright using the sign-corrected
state:

```text
x = [theta, thetaDot, phi, phiDot]
```

The main priority is tilt stabilization. Wheel position and wheel speed appear
only as weaker terms to suppress long-term walk-off.

The controller outputs symmetric wheel-velocity commands:

```text
u_left = u_right = u
```

That matches pure sagittal-plane balancing and maps directly to Pybricks
`Motor.run(...)`.

## 2. Why A Model-Light Nonlinear Controller

We deliberately avoided a controller that depends heavily on precise model
parameters such as:

- body mass,
- body inertia,
- center-of-mass height,
- motor torque constants.

Those parameters are hard to estimate accurately on a LEGO build, and a bad
model-based compensation term can hurt more than it helps.

Instead, the implemented controller trusts:

- measured tilt,
- measured tilt rate,
- measured wheel rotation,
- configured command limits.

It does not try to cancel gravity explicitly through a fragile feedforward term.

## 3. Implemented Control Law

The controller computes a balance-point error first:

```text
thetaError = theta - targetTilt
```

`targetTilt` lets the controller balance around a small configured lean instead
of forcing perfect geometric verticality. That is useful when the IMU zero,
center of mass, or wheel bias would otherwise create steady drift.

After optional filtering and deadbanding, the controller forms a composite
stabilizing variable:

```text
s = kTheta    * db(thetaError)
  + kThetaDot * db(thetaDotFiltered)
  - kPhi      * phi
  - kPhiDot   * phiDot
```

The wheel command is then:

```text
uSoft = maxWheelRate * tanh(s / sScale)
u     = sat(uSoft, maxWheelRate)
```

where:

- `db(...)` is a continuous deadband,
- `tanh(...)` gives a smooth bounded nonlinearity,
- `sat(...)` is a hard symmetric saturation backstop.

The controller returns:

```python
ControlOutput(
    leftCommand=u,
    rightCommand=u,
    mode=ControlMode.Velocity,
    timestamp=state.timestamp,
)
```

## 4. Meaning Of Each Term

### `kTheta`

The dominant restoring gain. If the robot falls before the wheels react
strongly enough, this is usually the first gain to increase.

### `kThetaDot`

The damping gain. This term reduces overshoot and oscillation around upright.

### `kPhi`

A weak centering term on wheel position. It is not needed for basic balance,
but it helps prevent slow walk-off over time.

### `kPhiDot`

A weak damping term on wheel speed. It supports drift suppression without
dominating the tilt response.

### `sScale`

The normalization factor before `tanh`. It controls how quickly the nonlinear
command approaches the wheel-rate limit.

- smaller `sScale`: more aggressive response, earlier saturation,
- larger `sScale`: softer response, wider quasi-linear region.

### `thetaDeadband` and `thetaDotDeadband`

These reduce sensor-noise-driven twitch near upright. The deadband is
continuous rather than stepwise so the command remains smooth.

### `thetaDotFilterAlpha`

An optional first-order IIR filter on `thetaDot`. This is meant for noise
reduction, not for stability by itself. Too much filtering adds phase lag and
can worsen oscillation.

## 5. Why `tanh`

`tanh` gives three useful properties at once:

1. It is approximately linear near zero, so small-error behavior still looks
   like familiar state feedback.
2. It is smooth, so command transitions are not abrupt.
3. It is bounded, so the controller naturally respects the motor-rate ceiling.

Near upright, the effective small-signal gains are approximately:

```text
kTheta_eff    = maxWheelRate * kTheta / sScale
kThetaDot_eff = maxWheelRate * kThetaDot / sScale
```

That makes `sScale` and `maxWheelRate` just as important as the raw gain values
when explaining the closed-loop behavior.

## 6. Why The Wheel Terms Have Negative Sign

Positive `phi` means the robot has rolled forward. If the robot keeps drifting
forward while nearly upright, we want the controller to bias the command
slightly backward. That is why `phi` and `phiDot` enter with negative sign in
the composite variable.

These terms are intentionally weaker than the tilt terms. If they are too
large, the controller starts fighting wheel drift more aggressively than body
tilt, which is usually the wrong priority for a balancing robot.

## 7. Implementation Details That Matter

The implementation includes a few practical details that are easy to miss when
only reading the equation:

- invalid states produce an immediate stop command,
- NaN values produce an immediate stop command,
- the command timestamp is preserved from the state,
- the controller uses only scalar math so it stays MicroPython-safe,
- `_Tanh(...)` falls back to a rational approximation if `math.tanh` is not
  available on the hub runtime,
- `Reset()` clears the optional `thetaDot` filter state.

These details are important because the same controller can run on desktop and
on the real hub through the shared package path.

## 8. Tuning Workflow

A practical tuning order is:

1. set `targetTilt` so the robot does not walk steadily in one direction,
2. increase `kTheta` until the controller reacts strongly enough to catch lean,
3. increase `kThetaDot` until oscillation is acceptably damped,
4. adjust `sScale` to set how quickly the command saturates,
5. add small `kPhi` and `kPhiDot` values only after tilt behavior is reasonable,
6. widen deadbands or add light `thetaDot` filtering only if sensor noise is
   causing chatter near upright.

Typical symptom-based adjustments:

- falls before correcting: increase `kTheta`,
- oscillates around upright: increase `kThetaDot`,
- too timid at moderate tilt: decrease `sScale`,
- slams to the limit too easily: increase `sScale`,
- balances but drifts: increase `kPhi` or `kPhiDot` slightly,
- twitches near zero: increase deadbands or use a light `thetaDot` filter.

## 9. Interaction With The Safety Layer

The controller is not the final safety authority. It computes the raw balance
command, then `SafetyMonitor.Check(...)` decides whether that command is allowed
to reach the motors.

That separation is intentional:

- the controller focuses on control quality,
- the safety monitor focuses on fault handling and stop behavior.

## 10. Alternative Balance Path: PID

The repository also includes `PidController` as a simpler comparison baseline.
It shares the same `ControlOutput` boundary and can be selected in config.

For the report, the clean way to describe the design is:

- the tanh nonlinear controller is the main controller,
- the PID controller is a baseline or comparison controller,
- both run inside the same estimator/safety/runtime architecture.

## 11. Recommended Report Language

If you want a concise report description, the controller can be summarized as:

> A model-light nonlinear wheel-velocity controller was implemented around a
> composite state variable built from tilt, tilt rate, wheel position, and
> wheel rate. The command is generated through a bounded `tanh` nonlinearity so
> that the controller remains smooth near upright and naturally saturates at the
> validated wheel-rate limit.
