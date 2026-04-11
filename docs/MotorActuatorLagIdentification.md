# Motor Actuator Lag Identification

This note records what was done in the single-motor velocity-step experiment
and what was inferred from it for controller design.

## Experiment

The experiment used the currently wired single motor on Port `F` with:

- `hub/HubSingleMotorStepResponseF.py`

The commanded velocity sequence was:

```text
0 -> 200 -> 0 -> 500 -> 0 -> 750 -> 0 -> 1000 -> 0 deg/s
```

The experiment logged:

- time,
- measured motor angle,
- commanded velocity reference,
- measured motor speed.

The resulting desktop plot is:

- `plots/HubSingleMotorStepResponse.png`

## What Was Observed

The measured speed does not jump to the command instantly. Over the lower and
middle command range, the up-steps look approximately like a first-order
velocity response.

Reading the speed plot by eye gives approximate 63% rise times of:

- `200 deg/s` step: about `0.18 s`,
- `500 deg/s` step: about `0.18 s`,
- `750 deg/s` step: about `0.20 s`,
- `1000 deg/s` step: about `0.25-0.30 s`.

The `1000 deg/s` step is slower than the others, which suggests the effective
actuator dynamics are not constant all the way to the speed limit.

## Inference

For controller design, the motor is approximated as a first-order velocity
actuator:

```text
d(phiDot)/dt = (u - phiDot) / tau
```

with a practical nominal time constant:

```text
tau_eff ≈ 0.20 s
```

This value should be treated as an engineering estimate, not as a precision
identification result. It was inferred from the plotted experiment rather than
from a least-squares fit to the raw telemetry.

## How It Is Used

The geometry aware nonlinear controller treats the actuator lag as a core
part of the plant rather than a small post hoc correction. The first order
model

```text
d(phiDot)/dt = (u - phiDot) / tau
```

implies that the instantaneous chassis linear acceleration is

```text
a = r * phiDDot = (r / tau) * (u - phiDot)
```

so the wheel velocity command required to produce a desired chassis
acceleration `aDes` is

```text
u = phiDot + (tau / r) * aDes
```

This inversion is used directly inside the controller's fast inner loop:
the Lyapunov style nominal law and the smooth sliding mode robust
correction together compute a desired chassis acceleration, and this
formula maps that acceleration into the wheel velocity command that
Pybricks' `Motor.run(...)` actually accepts. Without the inversion, the
wheel would never reach the velocity the controller is asking for in
time, and the effective chassis acceleration would always be a small
fraction of the commanded one. That mismatch is exactly what caused the
older composite variable tanh law to plateau in performance on this
hardware.

The same `tau` value is also used in the outer recentering loop, where
the reference tilt derivative is computed from the previous bounded
command via `aEstPrev = (r / tau) * (uPrev - phiDot)`. Using the
previous bounded command avoids the algebraic loop that would otherwise
couple the new reference derivative to the new desired acceleration. If
`tau <= 0` the controller treats the mapping as degenerate and falls
back to `aEstPrev = 0` and `u = phiDot`; configurations that want the
new law to work must therefore always provide a strictly positive
`actuatorTau`.

The current `configs/Default.yaml` sets `actuatorTau = 0.25 s` as a
conservative nominal value drawn from the experiment above.

## Limitations

- The estimate came from a PNG plot, not a saved raw log fit.
- The test used one motor rather than the full balancing robot.
- The effective time constant likely depends on operating point, battery state,
  and load.

So `tau = 0.20 s` is best viewed as a conservative nominal value for
controller-side compensation, not a universal motor constant.
