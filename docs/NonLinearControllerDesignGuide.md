# NonLinearController Design Guide

This document is the handoff for the person who will implement the real balancing
controller in `src/LegoBalance/NonLinearController.py`.

The key point is that the repo already defines the controller boundary. The work is to
fill in the control law, not to redesign the plumbing around it.

## 1. File And Class To Edit

- Canonical module: `src/LegoBalance/NonLinearController.py`
- Canonical class: `NonLinearController`
- Public method to implement: `Compute(state) -> ControlOutput`
- Lifecycle hook for stateful controllers: `Reset()`

`src/LegoBalance/LyapunovController.py` is now only a backward-compatible alias.

## 2. What The Controller Receives

Each call to `Compute` receives a `BalanceState` produced by `StateEstimator.Update`.

The implemented state is:

```text
x = [theta, thetaDot, phi, phiDot]
```

with the following field names on `BalanceState`:

- `state.tilt` or `state.theta`
- `state.tiltRate` or `state.thetaDot`
- `state.phi`
- `state.phiDot`
- `state.timestamp`
- `state.valid`

All values are already in SI units and already use the repo's verified sign convention.
The controller should never see raw Pybricks degrees, raw encoder directions, or raw IMU
signs.

## 3. Derived Quantities You May Use

The translational pair is derived on demand:

```text
p = r * phi
pDot = r * phiDot
```

You can compute those directly from the state:

```python
p = state.LinearPosition(self.config.chassis.wheelRadius)
pDot = state.LinearVelocity(self.config.chassis.wheelRadius)
```

Use `phi` / `phiDot` directly if that is more natural for your design. The repo keeps
both options open on purpose.

## 4. What The Controller Must Return

`Compute` must return a `ControlOutput`:

```python
return ControlOutput(
    leftCommand=u,
    rightCommand=u,
    mode=ControlMode.Velocity,
    timestamp=state.timestamp,
)
```

Important conventions:

- Positive command means forward chassis motion / increasing `phi`.
- Do not apply motor mounting sign flips in the controller.
- For pure balance, left and right commands should stay equal.
- Preserve `timestamp=state.timestamp`.
- If `state.valid` is `False`, return a stop command immediately.

The easiest first implementation target is `ControlMode.Velocity`, because Pybricks
already exposes `Motor.run(...)` as a velocity command.

## 5. How The Controller Is Used In The Loop

The runtime contract is:

```python
measurement = MakeMeasurement(...)
state = estimator.Update(measurement)
rawCommand = controller.Compute(state)
safeCommand = safety.Check(state, rawCommand, currentTime=state.timestamp)
ApplyToMotors(safeCommand)
```

That means:

- the controller computes the raw balancing command,
- the safety monitor is the final gate,
- the controller does not talk to motors directly,
- the controller does not own the estimator.

## 6. What Is Already Available In `self.config`

The controller constructor receives the full `RobotConfig`. Useful fields include:

- `self.config.chassis.wheelRadius`
- `self.config.chassis.wheelBase`
- `self.config.chassis.bodyMass`
- `self.config.chassis.bodyHeightCoM`
- `self.config.chassis.bodyInertia`
- `self.config.control.maxWheelRate`
- `self.config.control.maxTilt`
- `self.config.control.maxTiltRate`
- `self.config.controller.lambdaTheta`
- `self.config.controller.lambdaPhiDot`
- `self.config.controller.lambdaPhi`
- `self.config.controller.kTheta`
- `self.config.controller.kThetaDot`
- `self.config.controller.kPhi`
- `self.config.controller.kPhiDot`
- `self.config.controller.kSigma`
- `self.config.controller.boundaryLayerWidth`

If the controller needs precomputed gains or matrices, load or build them in `__init__`.
Keep `Compute` focused on one control step.

## 7. Recommended Implementation Pattern

1. Keep the controller boundary exactly as it is.
2. Precompute gains, matrices, operating points, or observer constants in `__init__`.
3. In `Compute`, extract the state, evaluate the control law, saturate intentionally, and
   return a `ControlOutput`.
4. If you add internal controller state, clear it in `Reset()`.

A reasonable first shape is:

```python
from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.Saturation import SaturateSymmetric

def Compute(self, state):
    if not state.valid:
        return ControlOutput.Stop(mode=ControlMode.Velocity, timestamp=state.timestamp)

    theta = state.tilt
    thetaDot = state.tiltRate
    p = state.LinearPosition(self.config.chassis.wheelRadius)
    pDot = state.LinearVelocity(self.config.chassis.wheelRadius)

    u = ...  # your control law here
    u = SaturateSymmetric(u, self.config.control.maxWheelRate)

    return ControlOutput(
        leftCommand=u,
        rightCommand=u,
        mode=ControlMode.Velocity,
        timestamp=state.timestamp,
    )
```

## 8. Design Guidance

- A Lyapunov-based design is the current expected first implementation, but the class
  name intentionally does not force that choice.
- Start with a symmetric sagittal-plane controller only.
- Prefer explicit saturation rather than relying only on the safety monitor.
- Keep units consistent. The repo boundary is radians, rad/s, meters, and seconds.
- Keep hardware-specific sign handling out of the controller.

If you want the controller to remain runnable through a shared package path on the hub,
avoid desktop-only dependencies such as `numpy` inside the controller module.

## 9. Minimum Validation Checklist

Before handing the controller back:

1. Update `tests/test_NonLinearController.py`.
2. Run `pytest`.
3. Run `python examples/ClosedLoopSimulation.py`.
4. Confirm `IsPlaceholder()` now reflects reality.
5. Decide whether the controller is desktop-only for now or intended to stay
   MicroPython-safe for future on-hub reuse.

## 10. What This Controller Does Not Need To Solve Yet

- Yaw control
- Trajectory tracking
- Online parameter estimation
- Rewriting the estimator or safety monitor interfaces

Those can come later. The immediate job is a stable upright balancer that honors the
existing repo contracts.
