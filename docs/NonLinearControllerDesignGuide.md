# NonLinearController Design Guide

This document explains the active balance controller in
`src/LegoBalance/NonLinearController.py`. It replaces the earlier model light
tanh composite variable guide. The current controller is a geometry aware
robust nonlinear law that uses the identified actuator lag model to map a
desired chassis acceleration into the wheel velocity command.

## 1. Design Goal

The controller stabilizes the two wheel inverted pendulum around upright
using the canonical state provided by the estimator:

```text
x = [theta, thetaDot, phi, phiDot]
```

- `theta`: body tilt angle in radians, positive means forward lean
- `thetaDot`: body tilt rate in rad/s
- `phi`: mean wheel rotation angle in radians, positive means rolled forward
- `phiDot`: mean wheel rotation rate in rad/s

Linear chassis position and velocity are derived with the wheel radius:

```text
p    = r * phi
pDot = r * phiDot
```

The controller outputs symmetric wheel velocity commands for pure sagittal
balance:

```text
uLeft = uRight = u
```

## 2. Why Not The Old Tanh Composite Variable Law

The earlier controller collapsed the full state into a single composite
variable `s = kTheta*theta + kThetaDot*thetaDot - kPhi*phi - kPhiDot*phiDot`
and ran it through `u = uMax * tanh(s/sScale)`. That design was deliberately
model light. It worked, but it had two structural issues that the new law
fixes:

1. It treated tilt gains and wheel gains as a flat scalar surface, so the
   controller could not cleanly separate the fast anti fall loop from the
   slow recentering loop.
2. It did not account for the identified first order actuator lag, so the
   wheel velocity command it issued was not the velocity the actuator
   actually delivered.

The new controller splits the problem into an inner tilt stabilizer, a slow
outer recentering loop, and an actuator lag inversion that maps a desired
chassis acceleration into a wheel velocity command the motor actually
tracks.

## 3. Reduced Plant Model

The sagittal tilt dynamics reduce to:

```text
(I + m*l^2) * thetaDDot = m*g*l*sin(theta) - m*l*a + dTheta
```

where:

- `m` is the body mass,
- `l` is the vertical offset from the wheel axle to the body center of mass,
- `I` is the pitch inertia of the body about its center of mass,
- `g = 9.81 m/s^2`,
- `a = pDDot` is the chassis linear acceleration,
- `dTheta` lumps every unmodeled effect.

Let `J = I + m*l^2`. Dividing through gives:

```text
thetaDDot = alphaHat * sin(theta) - betaHat * a + DeltaTheta
alphaHat  = (m*g*l) / J
betaHat   = (m*l)   / J
```

Both `alphaHat` and `betaHat` are computed from the configured geometry in
`chassis.bodyMass`, `chassis.bodyHeightCoM`, and `chassis.bodyInertia`.

The configured `bodyInertia` is the pitch inertia about the body center of
mass. The parallel axis term `m*l^2` is added internally. When
`bodyInertia` is zero or missing, the controller falls back to a
rectangular body approximation using `chassis.bodyLength` and
`chassis.bodyHeight`:

```text
I ~= m * (bodyLength^2 + bodyHeight^2) / 12
```

That fallback is an estimate; the controller documents it as such. If no
inertia source is available, construction fails loudly rather than
silently substituting a constant.

## 4. Actuator Model

Pybricks closes an internal velocity loop on the wheel, so the effective
plant is not a torque source. The Port F sweep identified a first order
response:

```text
phiDotActualDot = (u - phiDot) / tau
```

Converting to linear acceleration with `p = r*phi` gives:

```text
a = r * phiDDot = (r / tau) * (u - phiDot)
```

Inverted, the desired chassis acceleration maps to a wheel velocity
command:

```text
u = phiDot + (tau / r) * aDes
```

That mapping is mandatory. Without it, the controller would issue a
velocity command whose steady state the actuator never actually produces.

## 5. Slow Outer Loop

The outer loop produces a bounded tilt reference offset from the linear
chassis position and velocity. `targetTilt` stays the configured static
bias used to correct assembly asymmetry; the outer loop adds a slow
position centering lean around that base value:

```text
offsetRaw = -(kOuterP * p + kOuterD * pDot)
offset    = sat(offsetRaw, maxReferenceTiltOffset)
thetaRef  = targetTilt + offset
```

The saturation keeps the recentering loop from asking for an unreasonable
lean that the inner loop could not track. The outer loop is intentionally
slow so it does not dominate the fast anti fall loop.

### Reference derivative

The controller also needs `thetaRefDot` so the inner loop can form a
proper error rate. Taking the time derivative of the unclamped reference:

```text
thetaRefDot = -(kOuterP * pDot + kOuterD * a)
```

The current chassis acceleration `a` is estimated from the previous
bounded command through the actuator model, which breaks the algebraic
loop that would otherwise couple `thetaRefDot` and the new `aDes`:

```text
aEstPrev = (r / tau) * (uPrev - phiDot)
```

When `tau <= 0` the estimate falls back to zero.

## 6. Fast Inner Loop

The inner loop uses a Lyapunov style tilt energy function:

```text
V(e, eDot) = 0.5 * eDot^2 + lambda * (1 - cos(e))
```

and chooses a nominal desired chassis acceleration so the ideal small
signal error dynamics satisfy `eDDot + kDamping*eDot + lambda*e ~= 0`:

```text
aNom = ((alphaHat + lambda) * sin(e) + kDamping * eDot) / betaHat
```

Here `e = theta - thetaRef` and `eDot = thetaDot - thetaRefDot`.

### Robust correction

To absorb uncertainty in inertia, contact, friction and unmodeled terms,
a smooth sliding surface like robust term is added on top of the nominal
law:

```text
s       = eDot + cSurface * sin(e)
aRobust = (kRobust / betaHat) * tanh(s / epsilonBoundary)
aDes    = aNom + aRobust
```

Two invariants are enforced in the implementation:

- the robust term is always smooth (`tanh`, never `sign`),
- `epsilonBoundary` is strictly positive.

Together that guarantees the command is continuous even under pure sliding
behavior.

## 7. Full Control Step

```text
1. Read state: theta, thetaDot, phi, phiDot
2. Filter thetaDot lightly if thetaDotFilterAlpha > 0
3. Compute p = r*phi and pDot = r*phiDot
4. aEstPrev = (r/tau)*(uPrev - phiDot) if tau > 0 else 0
5. offsetRaw   = -(kOuterP*p + kOuterD*pDot)
   offset      = sat(offsetRaw, maxReferenceTiltOffset)
   thetaRef    = targetTilt + offset
6. thetaRefDot = -(kOuterP*pDot + kOuterD*aEstPrev)
7. e    = theta - thetaRef
   eDot = thetaDotUsed - thetaRefDot
8. J = I + m*l^2
   alphaHat = m*g*l / J
   betaHat  = m*l / J
9. aNom = ((alphaHat + lambda)*sin(e) + kDamping*eDot) / betaHat
10. s   = eDot + cSurface*sin(e)
11. aRobust = (kRobust/betaHat) * tanh(s / epsilonBoundary)
12. aDes    = aNom + aRobust
13. uRaw    = phiDot + (tau/r) * aDes
14. uBound  = sat(uRaw, maxWheelRate)
15. remember uBound as uPrev for the next step
16. return symmetric ControlOutput(uBound, uBound, Velocity, timestamp)
```

## 8. Config Fields

Geometry used by the controller:

- `chassis.wheelRadius`
- `chassis.bodyMass`
- `chassis.bodyHeightCoM`
- `chassis.bodyInertia` (pitch inertia about the body center of mass)
- `chassis.bodyLength` and `chassis.bodyHeight` (box fallback, optional)

Controller tuning:

- `controller.innerNaturalFrequency`: sets the nominal small signal
  natural frequency. `lambda = innerNaturalFrequency^2`. Bigger values
  give a stiffer inner loop.
- `controller.innerDampingRatio`: sets the nominal damping ratio.
  `kDamping = 2*innerDampingRatio*innerNaturalFrequency`. One is a good
  starting point.
- `controller.surfaceGain`: the contribution of `sin(e)` to the sliding
  surface variable `s`. Bigger values make the robust term react
  earlier at small tilt errors.
- `controller.robustGain`: magnitude of the smooth robust correction in
  `rad/s^2`. Bigger values reject disturbances and tolerate model
  mismatch but can feel aggressive near upright.
- `controller.boundaryLayerWidth`: strictly positive tanh boundary
  width. Very small values cause chatter; larger values soften the
  response.
- `controller.outerPositionGain`: rad per meter recentering gain on
  linear chassis position. Must be much slower than the inner loop.
- `controller.outerVelocityGain`: rad per (m/s) recentering gain on
  linear chassis velocity. Also slow by design.
- `controller.maxReferenceTiltOffset`: symmetric cap on the outer loop
  tilt offset, in radians. Prevents the recentering loop from asking
  for unreasonable lean.
- `controller.actuatorTau`: identified wheel velocity actuator time
  constant. Required to invert the actuator lag.
- `controller.thetaDotFilterAlpha`: optional IIR smoothing on the
  measured tilt rate. Use only for noise; heavy filtering adds phase
  lag and hurts stability.

Legacy fields such as `kTheta`, `kThetaDot`, `kPhi`, `kPhiDot`, `sScale`,
`thetaDeadband`, and `thetaDotDeadband` remain in the config schema so
older YAML and the hub runtime mirror keep loading. The current
controller does not read any of them.

## 9. Tuning Workflow

A practical tuning order is:

1. Verify the chassis geometry. Mass, CoM height, and bodyInertia drive
   `alphaHat` and `betaHat`. Measure or estimate them before touching
   any gain.
2. Set `targetTilt` so the robot does not walk steadily in one
   direction with `outerPositionGain = outerVelocityGain = 0`.
3. Start the inner loop with conservative values such as
   `innerNaturalFrequency = 6.0` and `innerDampingRatio = 1.0`. Increase
   the natural frequency until the robot catches lean reliably.
4. Adjust the damping ratio to remove oscillation. Stay near one.
5. Add a modest `robustGain` (for example `2.0`) and a reasonable
   `boundaryLayerWidth` (for example `0.5`). If the robot chatters
   near upright, widen the boundary layer.
6. Once the inner loop balances, enable the outer loop gradually. Start
   with small `outerPositionGain` such as `0.5 rad/m` and a matching
   `outerVelocityGain` that keeps the recentering critically damped.
7. Leave `maxReferenceTiltOffset` at a small value (for example
   `0.1 rad`, about six degrees) so the outer loop can never ask for
   an unreasonable lean.

Symptom guide:

- Falls before reacting: increase `innerNaturalFrequency`.
- Oscillates around upright: increase `innerDampingRatio` or widen
  `boundaryLayerWidth`.
- Too gentle against a push: increase `robustGain`.
- Chatters or buzzes near upright: widen `boundaryLayerWidth`.
- Balances but drifts: enable or raise `outerPositionGain` and
  `outerVelocityGain`.
- Outer loop demands unreasonable lean: lower
  `maxReferenceTiltOffset`.

## 10. Handling Geometry Uncertainty

The controller uses nominal configured geometry to form `alphaHat` and
`betaHat`. These nominal values are not meant to be exact: the robust
correction `aRobust = (kRobust/betaHat)*tanh(s/eps)` is the only thing
that has to tolerate moderate parameter mismatch. This is intentional
and documented in the controller source.

The controller is not performing exact feedback linearization. It is
performing Lyapunov style energy shaping with a smooth robust backup.

## 11. Interaction With The Safety Layer

The controller is not the final safety authority. It computes the raw
balance command, then `SafetyMonitor.Check(...)` decides whether that
command is allowed to reach the motors. The separation is intentional:

- the controller focuses on control quality,
- the safety monitor focuses on fault handling and stop behavior.

The controller itself still guards against invalid state and any NaN in
the state vector by returning a stop command immediately. That guard is
a local courtesy so the safety monitor never has to see garbage.

## 12. MicroPython Safety

Only scalar arithmetic is used. The module binds `math.tanh` and
`math.sin` at import time and falls back to tiny polynomial approximants
when either function is missing from the runtime. There are no list or
numpy allocations inside the control path, which keeps the controller
safe to run unchanged on the SPIKE hub under Pybricks.

## 13. Alternative Balance Path: PID

The repository still includes `PidController` as a simpler baseline. It
shares the same `ControlOutput` boundary and can be selected in config
via `controller.algorithm = "pid"`.

## 14. Recommended Report Language

If you need a concise report description, the controller can be
summarized as:

> A geometry aware robust nonlinear wheel velocity controller was
> implemented on top of the reduced tilt dynamics of the chassis. A slow
> outer loop generates a bounded tilt reference from the mean wheel
> position and velocity so the robot returns toward its starting point.
> The inner loop combines a Lyapunov style nominal acceleration law
> derived from the configured geometry with a smooth sliding surface
> like robust correction, and a first order actuator lag inversion maps
> the desired chassis acceleration into the wheel velocity command.
