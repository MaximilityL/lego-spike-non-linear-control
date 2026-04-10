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
reaches the motors. The current implementation includes a tanh-based nonlinear
balance controller, an alternative PID controller, real-hardware Pybricks
entrypoints, and automated tests for the estimator, controllers, safety logic,
and runtime integration.

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

### 4.4. Nonlinear balance controller

```text
thetaError = theta - targetTilt
s = kTheta * db(thetaError)
  + kThetaDot * db(thetaDotFiltered)
  - kPhi * phi
  - kPhiDot * phiDot
u = maxWheelRate * tanh(s / sScale)
```

### 4.5. Runtime pipeline

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

### Why use a tanh-based controller?

Because `tanh` gives a smooth bounded nonlinear command without depending on a
precise mechanical model of a LEGO build.

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

Describe the tanh controller as a model-light nonlinear velocity controller
centered on tilt stabilization, with weaker wheel-state terms for drift
suppression.

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
- real performance still depends on battery state, wheel slip, and chassis details.

## 9. Useful Source Files To Cite

- `src/LegoBalance/StateEstimator.py`
- `src/LegoBalance/NonLinearController.py`
- `src/LegoBalance/PidController.py`
- `src/LegoBalance/SafetyMonitor.py`
- `src/HubPackageBalance.py`
- `examples/ClosedLoopSimulation.py`
- `tests/test_NonLinearController.py`
- `tests/test_PidController.py`
- `tests/test_StateEstimator.py`
- `tests/test_SafetyMonitor.py`
