---
name: Tanh composite-variable controller implemented
description: NonLinearController replaced with model-light tanh law; sScale added to config
type: project
---

Replaced the CLF gravity-feedforward controller in NonLinearController.py with a tanh composite-variable law.

**Control law:** `u = maxWheelRate * tanh(s / sScale)` where `s = kTheta*db(theta) + kThetaDot*db(thetaDot) - kPhi*phi - kPhiDot*phiDot`

**Why:** avoids reliance on uncertain mass/inertia/CoM estimates; tanh provides smooth bounded output; no model inversion.

**Config changes:** kTheta bumped 30→60 (to compensate for removing gravity feedforward), new field `sScale: 15.0` added to ControllerConfig and Default.yaml. gravityCompGain preserved but unused.

**Command limit:** `config.control.maxWheelRate = 17.44 rad/s ≈ 1000 deg/s` — both soft tanh limit and hard SaturateSymmetric use this value in rad/s.

**Why:** The old controller has gravityCompGain * alpha_hat * sin(theta) term which requires accurate m, g, l, I estimates that are rough on LEGO hardware.

**How to apply:** When tuning, adjust kTheta (dominant), kThetaDot (damping), sScale (nonlinear knee). Drift via kPhi/kPhiDot. Toy simulation shows 0.110 rad final tilt vs 0.285 rad for old CLF — marginally better due to composite-variable drift suppression entering before tanh saturation.
