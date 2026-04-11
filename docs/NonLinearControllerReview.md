# Comprehensive Review: NonLinearController (Tanh Composite-Variable Balancer)

## 1. The Physical System

The robot is a **two-wheel inverted pendulum** (a "Segway"-type platform) built on LEGO SPIKE Prime hardware, running Pybricks MicroPython.

### 1.1 State Vector

The canonical state is:

$$\mathbf{x} = \begin{bmatrix} \theta \\ \dot\theta \\ \phi \\ \dot\phi \end{bmatrix}$$

| Symbol | Meaning | Units |
|---|---|---|
| $\theta$ | Body tilt angle (0 = upright, + = forward lean) | rad |
| $\dot\theta$ | Body tilt rate | rad/s |
| $\phi$ | Mean wheel rotation angle (+ = forward roll) | rad |
| $\dot\phi$ | Mean wheel rotation rate | rad/s |

The control output is a single scalar velocity command applied equally to both wheels (pure sagittal balance, no yaw):

$$u_{\text{left}} = u_{\text{right}} = u$$

### 1.2 Inverted Pendulum Dynamics

The standard equations of motion for a wheeled inverted pendulum (simplified, neglecting friction and motor dynamics) are:

$$(I + ml^2)\ddot\theta = mgl\sin\theta - ml\ddot p$$

$$\left(\frac{I_w}{r^2} + M + m\right)\ddot p = \frac{\tau}{r} - ml\ddot\theta\cos\theta + ml\dot\theta^2\sin\theta$$

where:
- $m$ = body mass, $l$ = distance from axle to CoM, $I$ = body inertia about axle
- $M$ = total wheel mass, $I_w$ = wheel inertia, $r$ = wheel radius
- $p = r\phi$ = linear position, $\tau$ = motor torque

The dominant instability is captured by the linearized tilt dynamics:

$$\ddot\theta \approx \alpha\,\theta - \beta\,u$$

where $\alpha = \frac{mgl}{I + ml^2}$ is the **gravitational instability parameter**. The natural divergence time scale is $\sim 1/\sqrt{\alpha}$. With the config values ($m = 0.276$ kg, $l = 0.105$ m, $I = 0.00337$ kg m$^2$):

$$\alpha \approx \frac{0.276 \times 9.81 \times 0.105}{0.00337 + 0.276 \times 0.105^2} \approx \frac{0.2843}{0.006413} \approx 44.3 \;\text{rad/s}^2$$

$$T_{\text{diverge}} = 1/\sqrt{44.3} \approx 0.15\;\text{s}$$

This means the robot tips over in roughly 150 ms if uncorrected, which is the fundamental timing constraint on the control loop (running at 100 Hz = 10 ms period, giving ~15 samples to react).

---

## 2. The Control Law -- Full Mathematical Description

### 2.1 Balance-Point Error

$$\theta_e = \theta - \theta_{\text{target}}$$

where $\theta_{\text{target}}$ (Default.yaml line 72, currently $-0.04$ rad) compensates for CoM offset so the robot does not walk.

### 2.2 Optional IIR Low-Pass Filter on $\dot\theta$

When `thetaDotFilterAlpha` $= \alpha_f > 0$:

$$\dot\theta_{\text{filt}}[k] = \alpha_f \cdot \dot\theta_{\text{filt}}[k-1] + (1 - \alpha_f) \cdot \dot\theta[k]$$

This is a first-order IIR with cutoff frequency:

$$f_c = \frac{f_s}{2\pi} \cdot \frac{1 - \alpha_f}{\alpha_f}$$

At $f_s = 100$ Hz, $\alpha_f = 0.2$: $f_c \approx 63$ Hz (light). At $\alpha_f = 0.3$: $f_c \approx 37$ Hz.

### 2.3 Continuous Deadband

$$\text{db}(x, w) = \begin{cases} x - w & \text{if } x > w \\ x + w & \text{if } x < -w \\ 0 & \text{if } |x| \le w \end{cases}$$

This is **continuous** (no step at the boundary), preserving smoothness of the composite variable.

```
      output
        |      /
        |     /   slope = 1
        |    /
   -----+---/---+---- input
  -w    |  /    w
        | /
        |/
```

### 2.4 Composite Variable

$$s = k_\theta \cdot \text{db}(\theta_e,\, w_\theta) + k_{\dot\theta} \cdot \text{db}(\dot\theta_{\text{in}},\, w_{\dot\theta}) - k_\phi \cdot \phi - k_{\dot\phi} \cdot \dot\phi$$

Current default gains from Default.yaml lines 97-105:

| Gain | Value | Role |
|---|---|---|
| $k_\theta$ | 250.0 | Dominant restoring (tilt proportional) |
| $k_{\dot\theta}$ | 25.0 | Damping (tilt rate) |
| $k_\phi$ | 0.0 | Drift suppression (wheel position) -- currently off |
| $k_{\dot\phi}$ | 0.0 | Drift suppression (wheel rate) -- currently off |
| $s_{\text{scale}}$ | 20.0 | Normalization scale |

### 2.5 Tanh Command

$$u_{\text{soft}} = u_{\max} \cdot \tanh\!\left(\frac{s}{s_{\text{scale}}}\right)$$

$$u = \text{sat}(u_{\text{soft}},\; u_{\max})$$

where $u_{\max} = 17.44$ rad/s (= 1000 deg/s, validated motor limit).

### 2.6 Small-Signal (Linearized) Effective Gains

Near upright, $\tanh(x) \approx x$ for small $x$, so:

$$u \approx \frac{u_{\max}}{s_{\text{scale}}} \left( k_\theta \cdot \theta_e + k_{\dot\theta} \cdot \dot\theta - k_\phi \cdot \phi - k_{\dot\phi} \cdot \dot\phi \right)$$

The effective linear gains are:

$$k_{\theta,\text{eff}} = \frac{u_{\max} \cdot k_\theta}{s_{\text{scale}}} = \frac{17.44 \times 250}{20} = 218.0 \;\text{rad/s per rad}$$

$$k_{\dot\theta,\text{eff}} = \frac{u_{\max} \cdot k_{\dot\theta}}{s_{\text{scale}}} = \frac{17.44 \times 25}{20} = 21.8 \;\text{rad/s per (rad/s)}$$

### 2.7 Actuator Lag Compensation

The motor is modeled as a first-order system with time constant $\tau_a$ (identified experimentally as approximately 0.20 s):

$$\dot\phi_{\text{actual}} \to \dot\phi_{\text{cmd}} \quad \text{with} \quad \frac{d\dot\phi}{dt} = \frac{u - \dot\phi}{\tau_a}$$

The controller predicts one step ahead:

$$\dot\phi_{\text{pred}} = \dot\phi + \min\!\left(\frac{\Delta t}{\tau_a},\; 1\right) \cdot (u_{\text{provisional}} - \dot\phi)$$

$$\phi_{\text{pred}} = \phi + \Delta t \cdot \dot\phi_{\text{pred}}$$

Then recomputes the final command using $(\phi_{\text{pred}}, \dot\phi_{\text{pred}})$ instead of $(\phi, \dot\phi)$.

This is only active when $\tau_a > 0$ **and** $k_\phi$ or $k_{\dot\phi}$ are nonzero. With the current config ($k_\phi = k_{\dot\phi} = 0$), **the lag compensation is effectively disabled**.

---

## 3. System Architecture Diagram

```
  +---------------------------------------------------------------------------+
  |                        Control Loop (100 Hz)                              |
  |                                                                           |
  |  +-------------+    +------------------+    +------------------------+    |
  |  |   IMU +     |    |                  |    |  NonLinearController   |    |
  |  |  Encoders   +--->| StateEstimator   +--->|                       |    |
  |  |  (raw)      |    |                  |    |  1. Balance-pt error   |    |
  |  +-------------+    | - sign correct   |    |  2. IIR filter         |    |
  |                     | - zero offset    |    |  3. Deadband           |    |
  |                     | - gyro bias      |    |  4. Composite s        |    |
  |                     | - wheel avg      |    |  5. tanh(s/sScale)     |    |
  |                     |                  |    |  6. Lag compensation   |    |
  |                     | Output:          |    |  7. Hard saturation    |    |
  |                     | BalanceState     |    |                       |    |
  |                     | [th,thd,ph,phd]  |    | Output:               |    |
  |                     +------------------+    | ControlOutput(u,u)    |    |
  |                                             +----------+------------+    |
  |                                                        |                 |
  |                                             +----------v------------+    |
  |                                             |   SafetyMonitor       |    |
  |                                             | - tilt limit check    |    |
  |                                             | - NaN/inf guard       |    |
  |                                             | - watchdog            |    |
  |                                             | - command saturation  |    |
  |                                             +----------+------------+    |
  |                                                        |                 |
  |                                             +----------v------------+    |
  |                                             |   Motors (Pybricks)   |    |
  |                                             |   Motor.run(u)        |    |
  |                                             +-----------------------+    |
  +---------------------------------------------------------------------------+
```

### Signal flow for the tanh law:

```
  th --+-- (-th_target) --> db(., w_th) --> x k_th --------+
      |                                                     |
  thd-+-- [IIR filter] --> db(., w_thd) --> x k_thd ------+
      |                                                     +-> s -> tanh(s/sScale) -> x u_max -> sat -> u
  ph -+-------------------------------------------> x(-k_ph) --+
      |                                                     |
  phd-+-------------------------------------------> x(-k_phd)--+
                    ^                                       |
                    |                                       |
                    +-- [lag compensation: predict ph,phd] <+
                        (only when tau_a > 0 & k_ph or k_phd > 0)
```

---

## 4. Theoretical Background and Classification

### 4.1 Where Does This Fit in Nonlinear Control Theory?

This controller is best classified as a **bounded state-feedback law with a smooth saturating nonlinearity**. It is related to, but distinct from, several formal approaches:

| Approach | Key Idea | Relation to This Controller |
|---|---|---|
| **Sliding Mode Control** | Force state onto a sliding surface $s = 0$ with discontinuous (sign) control | The composite variable $s$ is conceptually the same sliding surface, but tanh replaces $\text{sign}(s)$ -- avoids chattering |
| **Control Lyapunov Function (CLF)** | Find $V(x)$ such that $\dot V < 0$; design $u$ to guarantee descent | No explicit Lyapunov function is constructed; stability is empirical, not proven |
| **Feedback Linearization** | Cancel nonlinearities (e.g., $mg l \sin\theta$) then apply linear control | Deliberately avoided because LEGO model parameters are uncertain |
| **Backstepping** | Recursive Lyapunov construction for cascaded nonlinear systems | Not used; the tanh law is a single-step static mapping |
| **Energy-Based / Passivity** | Shape the energy function to have a minimum at upright | Not used, though the bounded tanh has a passivity-like flavor |
| **Linear State Feedback** | $u = -K\mathbf{x}$ where $K$ is from LQR or pole placement | This controller reduces to linear feedback near upright (small $s/s_{\text{scale}}$) |
| **Saturated Linear Feedback** | $u = \text{sat}(-K\mathbf{x})$ | Closest formal match. tanh is a smooth approximation of hard saturation |

### 4.2 The Sliding Surface Interpretation

The composite variable:

$$s = k_\theta \theta_e + k_{\dot\theta} \dot\theta - k_\phi \phi - k_{\dot\phi} \dot\phi$$

is exactly the form of a **sliding surface** from sliding mode control. In classical SMC, you would apply:

$$u = -u_{\max} \cdot \text{sign}(s)$$

which gives maximum corrective effort but **chatters** (oscillates infinitely fast across $s = 0$). Common fixes:

- **Boundary layer**: replace $\text{sign}(s)$ with $\text{sat}(s/\epsilon)$ -- piecewise linear, still has corners
- **tanh smoothing**: replace $\text{sign}(s)$ with $\tanh(s/\epsilon)$ -- infinitely differentiable, no corners

This controller uses the tanh variant. The parameter $s_{\text{scale}}$ is the boundary layer width. The tanh shaping means:

- **Inside the boundary** ($|s| \ll s_{\text{scale}}$): approximately linear, $u \approx u_{\max} \cdot s / s_{\text{scale}}$
- **Outside the boundary** ($|s| \gg s_{\text{scale}}$): saturated, $u \to \pm u_{\max}$

```
   u_max ----------------------------.   .----------------------------
                                      \ /
                                       X   <-- smooth transition region
                                      / \      (width ~ 2*sScale)
  -u_max ----------------------------'   '----------------------------
                                     s -->
          <--- saturated --->  < linear >  <--- saturated --->
```

### 4.3 Does This Controller Have Formal Stability Guarantees?

**Short answer: No, not as implemented.**

For a formal stability proof you would need to:

1. Construct a Lyapunov function $V(\theta, \dot\theta, \phi, \dot\phi) > 0$
2. Show $\dot V < 0$ along trajectories under the tanh law
3. Handle the actuator dynamics (first-order lag)
4. Account for the deadband and filter

The controller relies on **empirical stability**: the gains are tuned until the robot balances. This is standard engineering practice for hobbyist and educational platforms.

However, the **structure** of the controller is sound for the following reasons:

**Necessary condition check** (Routh-Hurwitz on the linearized system): Near upright, the closed loop with $u = -(k_{\theta,\text{eff}} \theta + k_{\dot\theta,\text{eff}} \dot\theta)$ applied to $\ddot\theta = \alpha\theta - \beta u$ gives:

$$\ddot\theta + \beta k_{\dot\theta,\text{eff}} \dot\theta + (\beta k_{\theta,\text{eff}} - \alpha)\theta = 0$$

For stability:
- $\beta k_{\dot\theta,\text{eff}} > 0$ (damping positive -- satisfied since all gains are positive)
- $\beta k_{\theta,\text{eff}} > \alpha$ (restoring force overcomes gravity)

With $k_{\theta,\text{eff}} = 218$ rad/s per rad, even a small $\beta$ (coupling from wheel velocity to tilt acceleration) should overcome $\alpha \approx 44.3$. This is a reasonable margin.

---

## 5. Implementation Review

### 5.1 File Dependency Graph

```
NonLinearController.py
    +-- ControllerBase.py          (abstract base: Compute, Reset)
    +-- ControlInterfaces.py       (ControlMode, ControlOutput, Measurement)
    +-- Saturation.py              (SaturateSymmetric)
    +-- BalanceState.py            (state container with th/thd/ph/phd)
    |
    +-- Used by:
    |   +-- BalanceControllerFactory.py  (selects tanh vs pid)
    |   +-- SafetyMonitor.py             (post-controller safety gate)
    |   +-- LyapunovController.py        (backward-compat alias)
    |
    +-- Configured by:
        +-- RobotConfig.py / Default.yaml
```

### 5.2 Code Quality Assessment

**Strengths:**

- **MicroPython safe**: No numpy, no lists inside the loop, scalar-only arithmetic, Pade fallback for tanh. This is critical for running on the SPIKE hub with ~256KB RAM.
- **NaN guards everywhere**: Both the `Compute` method and the deadband check `value != value` for NaN. Defensive and appropriate for a safety-critical path.
- **Separation of concerns**: The controller does not know about motors, encoders, or Pybricks APIs. It consumes `BalanceState`, produces `ControlOutput`.
- **Extensive test suite**: 20+ tests in `tests/test_NonLinearController.py` covering contract, structure, sensitivity, filter behavior, and backward compat.
- **Well-documented**: The module docstring is a self-contained design document.

**Minor issues:**

1. **`_Tanh` Pade approximant accuracy** (NonLinearController.py lines 164-171): The Pade [3,3] approximant $\frac{x(27 + x^2)}{27 + 9x^2}$ is decent but the error claim "< 2.2% for $|x| \le 2.5$" is conservative. The actual max absolute error at $x = 2.5$ is:

   $$\text{pade}(2.5) = \frac{2.5(27 + 6.25)}{27 + 56.25} = \frac{83.125}{83.25} \approx 0.9985$$
   $$\tanh(2.5) \approx 0.9866$$

   Error approximately 0.012 (1.2%), which is fine for control. But the hard clamp at $|x| \ge 3$ introduces a tiny discontinuity in the derivative -- unlikely to matter in practice.

2. **`LyapunovController` naming** (LyapunovController.py): This is a backward-compat alias but the name is misleading since the controller does **not** use a Lyapunov function. The file is only 6 lines and re-exports `NonLinearController`.

3. **Filter alpha warning in docstring vs config**: The docstring at NonLinearController.py lines 222-227 warns "DO NOT use alpha > 0.3" but the config dataclass default is `thetaDotFilterAlpha: 0.3` (RobotConfig.py line 98). The active config uses `0.0` so this is not a runtime issue, but the defaults are contradictory.

---

## 6. Comparison with the PID Controller

Both controllers are available via the factory (`BalanceControllerFactory.py`):

| Feature | NonLinearController (tanh) | PidController |
|---|---|---|
| **Control law** | $u = u_{\max} \tanh(s / s_{\text{scale}})$ | $u = K_p e + K_i \Sigma e + K_d \Delta e + K_s \phi$ |
| **State feedback** | Full 4-state $(\theta, \dot\theta, \phi, \dot\phi)$ | 2-state $(\theta, \phi)$; $\dot\theta$ derived via differencing |
| **Internal state** | Optional IIR filter only | Integrator + previous error |
| **Saturation** | Inherent (tanh is bounded) | External clamp only |
| **Wind-up** | Impossible (no integrator) | Anti-windup clamp needed |
| **Units** | All rad/s internally | Internally in degrees, converted at output |
| **Tilt-rate damping** | Explicit $k_{\dot\theta}$ gain | Implicit via derivative term (noisy) |
| **Drift suppression** | Configurable $k_\phi$, $k_{\dot\phi}$ | $K_s \cdot \phi$ term |
| **Actuator lag** | Optional predictive compensation | None |
| **Noise sensitivity** | Deadband + optional filter | Deadband on tilt only; derivative amplifies noise |

**Key difference**: The PID controller derives tilt rate from finite differencing ($\Delta e = e[k] - e[k-1]$), which amplifies high-frequency noise. The tanh controller uses the gyroscope rate directly via $\dot\theta$ from the estimator, which is cleaner.

**Key advantage of PID**: It has an integrator, which eliminates steady-state error (e.g., persistent lean due to CoM offset). The tanh controller handles this via `targetTilt` instead, which requires manual calibration.

---

## 7. Comparison with Other Nonlinear Control Approaches

### 7.1 vs. Energy-Based Swing-Up + LQR (Astrom & Furuta style)

The classic approach for cart-pole systems:
- **Swing-up region** ($|\theta| > \theta_{\text{switch}}$): Use energy-pumping control $u = k_E (E - E_0) \text{sign}(\dot\theta \cos\theta)$
- **Balance region** ($|\theta| < \theta_{\text{switch}}$): Switch to LQR $u = -K\mathbf{x}$

**Comparison**: The tanh controller does not have a swing-up mode; it assumes the robot starts near upright. However, the tanh naturally provides a "recovery-like" behavior because it saturates to maximum effort at large tilts, which is qualitatively similar to a bang-bang recovery. The smooth transition avoids the mode-switching discontinuity.

### 7.2 vs. Feedback Linearization

Feedback linearization cancels the gravitational nonlinearity:

$$u = \frac{1}{\beta}(\alpha \sin\theta + v)$$

where $v$ is a linear stabilizing input. This requires knowing $\alpha$ and $\beta$ accurately.

**Comparison**: On LEGO hardware, $\alpha$ depends on body mass, CoM height, and inertia, which are rough estimates (the config has commented-out alternatives for wheel radius, CoM height, and zero offset). A 20% error in $\alpha$ creates a persistent disturbance equal to 20% of the gravitational torque. The tanh controller avoids this failure mode entirely.

### 7.3 vs. Full CLF / Lyapunov Design

A CLF controller would choose:

$$V = \frac{1}{2}(\theta^2 + c\dot\theta^2) \quad \text{and design } u \text{ such that } \dot V < 0$$

leading to something like:

$$u = \frac{\alpha\theta\dot\theta + c\alpha\dot\theta\cos\theta \cdot \theta + \text{...}}{\text{...}}$$

This is mathematically elegant but requires the full model and produces complex expressions.

**Comparison**: The tanh controller sacrifices the formal $\dot V < 0$ guarantee for implementation simplicity and model-independence. For an educational LEGO platform, this is the right tradeoff.

---

## 8. Does the Theory Hold?

### 8.1 Linearized Stability Check

Near upright with the current gains ($k_\theta = 250$, $k_{\dot\theta} = 25$, $s_{\text{scale}} = 20$):

Effective gains: $k_{\theta,\text{eff}} = 218$, $k_{\dot\theta,\text{eff}} = 21.8$

The linearized tilt dynamics become:

$$\ddot\theta + 21.8\beta\,\dot\theta + (218\beta - 44.3)\,\theta = 0$$

For stability we need $218\beta > 44.3$, i.e., $\beta > 0.203$. The coupling coefficient $\beta$ represents how wheel velocity translates to tilt acceleration. For a typical wheeled pendulum, $\beta$ is order 1-10, so **the stability margin is large**.

Damping ratio check: $\zeta = \frac{21.8\beta}{2\sqrt{218\beta - 44.3}}$. For $\beta = 1$: $\zeta = \frac{21.8}{2\sqrt{173.7}} = \frac{21.8}{26.4} \approx 0.83$, which is well-damped (underdamped but close to critical).

### 8.2 Saturation and Large-Angle Behavior

At large tilt, $\tanh \to \pm 1$ and the controller gives maximum effort ($\pm 17.44$ rad/s). This is correct: when the robot is falling, you want full motor authority. The tanh provides this naturally without mode switching.

The concern is whether the motor limit is fast enough to catch the fall. With $\alpha \approx 44.3$ and max $\dot\phi = 17.44$ rad/s, the maximum linear recovery acceleration is roughly $\beta \cdot 17.44$. For the robot to recover from a tilt $\theta_0$:

$$\beta \cdot 17.44 > \alpha \cdot \theta_0 \quad \Rightarrow \quad \theta_0 < \frac{\beta \cdot 17.44}{44.3}$$

This defines the **region of attraction**. For $\beta = 1$: $\theta_0 < 0.39$ rad (approximately 22 deg). The safety monitor trips at $|\theta| > 2.0$ rad, which is well beyond what the controller can recover from, so the safety limit is a catch-all, not a control boundary.

### 8.3 Deadband Considerations

The deadbands ($w_\theta$, $w_{\dot\theta}$) create a "dead zone" where the controller outputs zero. Inside this zone, the open-loop dynamics dominate:

$$\ddot\theta \approx \alpha\theta \quad (\text{unstable, no control})$$

For safety, the deadband must be narrow enough that the robot does not diverge significantly before control engages. The time to grow from $w_\theta$ to $2w_\theta$ under open-loop divergence is:

$$\Delta t \approx \frac{1}{\sqrt\alpha} \ln(2) \approx \frac{0.693}{6.66} \approx 0.104 \;\text{s}$$

With the current config having $w_\theta = 0.0$ and $w_{\dot\theta} = 0.0$, deadbands are disabled, so this is not an active concern.

---

## 9. Suggestions for Improvement

### 9.1 Enable Drift Suppression ($k_\phi$, $k_{\dot\phi}$)

Currently both are 0.0 in the config. Without them, the robot will balance upright but slowly translate. Start with very small values:

$$k_\phi \approx 0.5\text{-}2.0, \quad k_{\dot\phi} \approx 0.5\text{-}2.0$$

These should be ~1-2% of $k_\theta$ to avoid coupling noisy wheel states into the tilt loop.

### 9.2 Add Integral Action on Tilt

The `targetTilt` parameter is a manual open-loop bias correction. A slow integrator on $\theta_e$ would auto-trim this:

$$\theta_{\text{target}}[k+1] = \theta_{\text{target}}[k] + k_I \cdot \theta_e[k] \cdot \Delta t$$

with a small $k_I$ (e.g., 0.1-0.5 rad/s) and anti-windup. This would eliminate the need to hand-tune `targetTilt` for each build configuration.

### 9.3 Gain Scheduling

The tanh already provides implicit gain scheduling (high gain near upright, saturated at large angles). But you could explicitly schedule $s_{\text{scale}}$:

- Small $s_{\text{scale}}$ at large $|\theta|$ for aggressive recovery
- Large $s_{\text{scale}}$ near upright for smooth, quiet balancing

### 9.4 Consider Duty-Cycle Mode

The controller outputs velocity commands ($u$ in rad/s) which go through Pybricks' internal velocity PID. This adds a layer of dynamics. Switching to duty-cycle mode (`Motor.dc(...)`) would give more direct torque-like control, potentially reducing the effective actuator lag. The tradeoff is losing Pybricks' built-in speed regulation.

### 9.5 Formal Region-of-Attraction Estimation

If you want to strengthen the theoretical foundation for a report, you could:

1. Define a candidate Lyapunov function $V = \frac{1}{2}\mathbf{x}^T P \mathbf{x}$ where $P$ solves the Lyapunov equation for the linearized system
2. Numerically estimate the largest level set $V(\mathbf{x}) \le c$ where $\dot V < 0$ still holds under the tanh saturation
3. This gives a provable region of attraction, even without global stability

### 9.6 Better Estimator

The current `StateEstimator` is a pure pass-through (no filtering, no fusion). A complementary filter is already stubbed:

$$\hat\theta[k] = \alpha(\hat\theta[k-1] + \dot\theta \cdot \Delta t) + (1-\alpha)\theta_{\text{accel}}$$

This would improve tilt estimation robustness during fast motions where the accelerometer alone is unreliable.

### 9.7 The Filter Default Warning

Fix the contradictory default: either change `thetaDotFilterAlpha` default in RobotConfig.py line 98 from `0.3` to `0.0`, or remove the "DO NOT use > 0.3" warning. The active config correctly uses `0.0`.

---

## 10. Summary Assessment

| Aspect | Assessment |
|---|---|
| **Design philosophy** | Sound. Model-light is the right call for uncertain LEGO parameters. |
| **Mathematical structure** | Correct. Composite variable + tanh is a well-known smooth sliding mode variant. |
| **Linearized stability** | Satisfied with large margin given the current gains. |
| **Implementation quality** | High. MicroPython-safe, well-tested, well-documented, defensive. |
| **Theoretical rigor** | Moderate. No formal Lyapunov proof, but the linearized analysis is solid and the structure supports one. |
| **Practical readiness** | The balance loop works; drift suppression ($k_\phi$, $k_{\dot\phi}$) and estimator improvement are the main gaps. |
| **Compared to alternatives** | Better than plain PID (smooth saturation, no wind-up, direct rate feedback). Less rigorous than CLF/feedback linearization but more robust to model error. |

The controller is a pragmatic, well-engineered design that sits in a sweet spot between "simple PID" and "full model-based nonlinear control." For an educational LEGO platform with uncertain parameters, this is an excellent choice.
