# Implementation Report Guide

This document is a report-friendly summary of what the repository currently
implements. It is written so you can turn it into LaTeX sections with minimal
rewriting.

## 1. One-Paragraph Project Summary

This project implements the control stack for a two-wheel self-balancing robot
built on the LEGO SPIKE Prime hub with Pybricks. The software architecture
separates desktop-side development and testing from hub-side execution, while
preserving one common runtime pipeline: raw sensor measurements are converted
into a sign-corrected state estimate, a balance controller computes a symmetric
wheel-velocity command, and a safety monitor validates that command before it
reaches the motors. The current balance controller is a geometry aware robust
nonlinear law: it builds a reduced sagittal pendulum model from configured
chassis geometry, stabilizes the tilt error around a slow outer-loop reference
with a Lyapunov style nominal acceleration plus a smooth sliding mode robust
correction, and maps the resulting desired chassis acceleration through an
identified first-order actuator lag into the wheel-velocity command accepted by
the Pybricks motor API. A simpler discrete PID controller is retained behind the
same factory interface as a baseline. Real-hardware Pybricks entrypoints (live
telemetry and buffered post-run variants) and automated tests cover the
estimator, controllers, safety logic, and runtime integration.

## 2. Suggested Report Structure

A clean report structure for this repository is:

1. Introduction and project objective
2. Hardware platform and assumptions
3. Software architecture
4. State estimation
5. Control design
6. Safety and runtime supervision
7. Experimental workflow and tooling
8. Validation and testing
9. Limitations and future work

## 3. Core Technical Story

The simplest accurate story of the project is:

1. We first fixed the robot sign convention in configuration rather than inside
   the controller.
2. We implemented a minimal estimator that converts raw IMU and encoder data
   into a clean state vector.
3. We implemented a balance controller that consumes that state and outputs a
   symmetric wheel command.
4. We placed a safety monitor after the controller so that unsafe states or
   stale updates force a stop.
5. We validated the same logical pipeline on desktop and on the real hub.

## 4. Equations Worth Including

### 4.1. State definition

```text
x = [theta, thetaDot, phi, phiDot]
```

### 4.2. Estimator mapping

```text
theta    = tiltSign * rawTilt + zeroOffset
thetaDot = tiltSign * rawTiltRate - gyroBias
phi      = forwardSign * 0.5 * (leftEncoderSign * leftAngle + rightEncoderSign * rightAngle)
phiDot   = forwardSign * 0.5 * (leftEncoderSign * leftRate  + rightEncoderSign * rightRate)
```

### 4.3. Derived translation

```text
p = r * phi
pDot = r * phiDot
```

### 4.4. Reduced sagittal tilt dynamics

```text
(I + m*l^2) * thetaDDot = m*g*l*sin(theta) - m*l*a + dTheta
J         = I + m*l^2
alphaHat  = m*g*l / J
betaHat   = m*l   / J
thetaDDot = alphaHat*sin(theta) - betaHat*a + DeltaTheta
```

### 4.5. First-order wheel velocity actuator model

```text
d(phiDot)/dt = (u - phiDot) / tau
a            = r * phiDDot = (r/tau) * (u - phiDot)
u            = phiDot + (tau/r) * aDes          # inversion used by the controller
```

### 4.6. Geometry aware nonlinear balance controller

Slow outer loop (position recentering reference):

```text
p              = r * phi
pDot           = r * phiDot
offsetRaw      = -(kOuterP*p + kOuterD*pDot)
offset         = sat(offsetRaw, maxReferenceTiltOffset)
thetaRef       = targetTilt + offset
aEstPrev       = (r/tau) * (uPrev - phiDot)      # if tau > 0 else 0
thetaRefDot    = -(kOuterP*pDot + kOuterD*aEstPrev)
```

Fast inner loop (Lyapunov style nominal law plus smooth robust correction):

```text
e       = theta - thetaRef
eDot    = thetaDotUsed - thetaRefDot
aNom    = ((alphaHat + lambda)*sin(e) + kDamping*eDot) / betaHat
s       = eDot + cSurface*sin(e)
aRobust = (kRobust / betaHat) * tanh(s / epsilonBoundary)
aDes    = aNom + aRobust
```

Command mapping and saturation:

```text
uRaw    = phiDot + (tau/r)*aDes
u       = sat(uRaw, maxWheelRate)
```

Gain conventions: `lambda = innerNaturalFrequency^2`,
`kDamping = 2*innerDampingRatio*innerNaturalFrequency`,
`cSurface = surfaceGain`, `kRobust = robustGain`,
`epsilonBoundary = boundaryLayerWidth`.

### 4.7. Runtime pipeline

```text
measurement -> estimator -> controller -> safety monitor -> motors
```

## 5. Design Decisions To Explain

### Why use `[theta, thetaDot, phi, phiDot]`?

Because `phi` and `phiDot` come directly from wheel encoders and do not require
the estimator to trust wheel-radius calibration more than necessary.

### Why use configuration for sign conventions?

Because it keeps hardware-specific mounting corrections out of the controller,
so the controller sees one consistent physical meaning for each state variable.

### Why use a geometry aware nonlinear controller instead of a simple PID?

Because the plant is structurally unstable and the actuator is not torque-like.
The reduced tilt dynamics depend on mass, CoM height, and pitch inertia (partly
known from the config), while the wheel channel behaves like a first-order lag
with a measurable time constant. A law that reasons in chassis acceleration and
then maps that acceleration through the actuator model fits this physics better
than a generic PID on tilt error. A smooth sliding mode robust correction is
added on top of the nominal Lyapunov law so the controller tolerates the
remaining uncertainty in pitch inertia, contact, and the lag time constant.

### Why not full feedback linearization?

Because LEGO build parameters (mass, inertia, CoM height, wheel radius) are not
known accurately enough to justify exact cancellation of the nonlinearities. A
20% error in `alphaHat` would propagate as a persistent disturbance. The
controller therefore uses nominal geometry for the Lyapunov style law and lets
the smooth robust term absorb the mismatch, rather than trusting the geometry
to be correct.

### Why keep a separate safety monitor?

Because safety logic should fail toward "stop" independently of the control-law
implementation.

## 6. Suggested Figures

Good figures for the report would be:

- a block diagram of the runtime pipeline,
- a diagram of the state vector and sign conventions,
- a post-run balance plot from `scripts/PlotHubPackageBalance.py`,
- a plot showing tilt reference, measured tilt, and wheel command,
- optionally a comparison between nonlinear and PID runs.

## 7. Suggested Section Notes

### Introduction

State that the project goal was to build a reusable and testable balancing
control stack for a LEGO SPIKE two-wheel inverted pendulum rather than only a
single throwaway script.

### Architecture

Explain the desktop/hub split and emphasize that the estimator, controller, and
safety logic are shared across simulation and hardware runs.

### Estimator

Describe the estimator as intentionally minimal: it performs sign correction,
offset correction, and state assembly, but does not yet perform full sensor
fusion.

### Controller

Describe the active controller as a geometry aware robust nonlinear law with
three layers: a slow outer loop that biases the tilt reference from the linear
chassis position and velocity, a fast inner loop that combines a Lyapunov style
nominal acceleration with a smooth sliding mode robust correction, and an
actuator lag inversion that maps the desired chassis acceleration into the
wheel-velocity command the Pybricks motor API accepts. Emphasize that the
reduced tilt model is derived from chassis geometry (mass, CoM height, pitch
inertia) rather than a hand-tuned composite variable, and that the robust term
is the part responsible for tolerating the remaining parameter uncertainty.

### Safety

Describe the safety monitor as the last gate before actuation, with tilt limits,
tilt-rate limits, and a watchdog timeout.

### Validation

Explain that validation was layered: unit tests, integration tests, desktop
closed-loop simulation, and package-backed real-hub runs.

## 8. Honest Limitations To Mention

For a strong report, it is worth stating the limits clearly:

- the estimator is still minimal and does not yet perform sensor fusion,
- hardware tuning is empirical,
- yaw control is out of scope,
- the mock plant is useful for interface checks but not for proving hardware stability,
- real performance still depends on battery state, wheel slip, and chassis details,
- the pitch inertia in the config is an estimate, not a measured value; the
  smooth robust term is what tolerates that mismatch,
- the actuator time constant `tau` was read from a single motor step response
  plot and is a nominal engineering value, not a least-squares identification,
- there is no global Lyapunov proof; the nominal descent argument is only valid
  near upright and assumes an ideal actuator, so the controller is not claimed
  to be globally stabilizing.

## 9. Useful Source Files To Cite

- `src/LegoBalance/StateEstimator.py`
- `src/LegoBalance/NonLinearController.py`
- `src/LegoBalance/PidController.py`
- `src/LegoBalance/BalanceControllerFactory.py`
- `src/LegoBalance/SafetyMonitor.py`
- `src/LegoBalance/RobotConfig.py`
- `configs/Default.yaml`
- `src/HubPackageBalance.py`
- `src/HubPackageBalanceBuffered.py`
- `scripts/PlotHubPackageBalance.py`
- `scripts/PlotHubPackageBalanceBuffered.py`
- `examples/ClosedLoopSimulation.py`
- `tests/test_NonLinearController.py`
- `tests/test_PidController.py`
- `tests/test_StateEstimator.py`
- `tests/test_SafetyMonitor.py`
- `docs/NonLinearControllerDesignGuide.md`
- `docs/NonLinearControllerReview.md`
- `docs/MotorActuatorLagIdentification.md`
