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

The nonlinear controller uses this actuator lag only on the wheel-state part of
the law.

The compensation strategy is:

1. compute a provisional wheel-velocity command,
2. predict `phi` and `phiDot` one control interval ahead under a first-order
   motor model parameterised by the identified `tau`,
3. recompute the controller output using those predicted wheel states.

This anticipates that the measured `phi` and `phiDot` are lagging behind the
command the controller has already issued.

## Limitations

- The estimate came from a PNG plot, not a saved raw log fit.
- The test used one motor rather than the full balancing robot.
- The effective time constant likely depends on operating point, battery state,
  and load.

So `tau = 0.20 s` is best viewed as a conservative nominal value for
controller-side compensation, not a universal motor constant.
