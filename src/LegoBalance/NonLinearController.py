"""Tanh composite-variable nonlinear balancing controller.

This module implements a model-light nonlinear velocity controller for the
LEGO SPIKE inverted pendulum. The design is intentionally decoupled from the
physical model: no gravity feedforward, no feedback linearization, no torque
computation, no reliance on mass / inertia / center-of-mass estimates.

Why model-light and tanh-based?
--------------------------------

A gravity-feedforward (CLF) controller achieves formal stability guarantees
by inverting the dominant nonlinearity ``alpha * sin(theta)``, but that
inversion requires accurate estimates of body mass, inertia, and center-of-mass
height. On LEGO hardware those estimates are rough. A feedforward error is an
additive disturbance on the tilt dynamics that can make the robot *worse* when
the estimated ``alpha_hat`` deviates from reality by even a few percent.

This controller takes the opposite stance: tilt and tilt rate are measured /
estimated and trusted; the plant model is not. The balancing law is:

    s     =   kTheta    * deadband(theta)
            + kThetaDot * deadband(thetaDot)
            - kPhi      * phi
            - kPhiDot   * phiDot

    u_soft  =  uSoftMax * tanh(s / sScale)      [smooth bounded output]
    u       =  SaturateSymmetric(u_soft, maxWheelRate)   [hard backstop]

where ``uSoftMax = maxWheelRate = config.control.maxWheelRate``.

Why the composite variable s?
------------------------------

For an inverted pendulum the dominant instability is the tilt angle theta.
Tilt rate thetaDot provides the necessary damping. Together they form the
minimal state pair needed to balance. The composite variable s aggregates them
into a scalar that encodes both how far the robot is from upright and how fast
it is moving away.

Wheel position phi and rate phiDot appear with negative-sign, weak coefficients.
Their sole purpose is drift suppression over longer horizons: without them the
robot would balance near upright but slowly translate away from its starting
position. Setting kPhi = kPhiDot = 0 leaves pure tilt balancing intact.

Why tilt and tilt-rate dominate (kTheta, kThetaDot are large)?
---------------------------------------------------------------

The tilt angle is the state whose unstable mode must be stabilised. The
gravitational instability time scale is roughly 1/sqrt(alpha) where
``alpha = m*g*l/(I+m*l^2)``. Any controller that fails to provide restoring
force faster than this time scale will lose the robot. Setting kTheta large
relative to sScale ensures that even without gravity feedforward, the
proportional tilt term overwhelms the gravitational divergence.

kThetaDot provides the damping required to prevent oscillations around the
upright equilibrium. A well-damped closed loop satisfies
``kThetaDot_eff^2 > 4 * kTheta_eff`` (overdamped) or more practically
``kThetaDot_eff ~ sqrt(kTheta_eff)`` (critically damped).

Why wheel position / speed are only secondary terms?
-----------------------------------------------------

phi and phiDot estimates are noisier and more sensitive to wheel-ground slip
than the IMU-derived tilt. Including them in the dominant control path would
amplify noise and couple uncertain wheel dynamics into the tilt stabiliser.
Keeping kPhi and kPhiDot small (typically 1-5% of the tilt gain) gives drift
correction without compromising the primary balance loop.

Why tanh?
----------

1. **Smooth near zero**: near upright the law behaves like a linear state
   feedback law. The effective linear gains are
   ``kTheta_eff = uSoftMax * kTheta / sScale`` and
   ``kThetaDot_eff = uSoftMax * kThetaDot / sScale``.
2. **Bounded output**: tanh maps all reals to (-1, 1). Scaled by ``uSoftMax``
   the command is naturally limited to ``[-uSoftMax, +uSoftMax]`` without
   any discontinuous clipping.
3. **No mode switching**: no hard bang-bang transitions, no chattering, no
   deadband-induced steps in the command signal.
4. **Robustness to model error**: because the law contains no model-derived
   cancellation terms, model error cannot introduce a destabilising disturbance.

Command limit interpretation
------------------------------

``config.control.maxWheelRate = 17.44 rad/s`` corresponds to approximately
1000 deg/s, which is the validated LEGO SPIKE motor rate limit. All controller
signals are in rad/s. Both the soft tanh limit (``uSoftMax = maxWheelRate``)
and the hard ``SaturateSymmetric`` call operate in rad/s directly. The motor
application layer is responsible for any hardware-level deg/s conversion.

MicroPython safety
------------------

The controller uses only scalar arithmetic and a custom ``_Tanh`` that falls
back to a Padé rational approximant when ``math``/``umath`` is unavailable.
No numpy, no lists, no heap allocations inside the control loop.

Gain tuning guide
------------------

- Robot falls before correcting: increase ``kTheta``.
- Robot oscillates around upright: increase ``kThetaDot``.
- Correct speed, but too gentle at moderate tilt: decrease ``sScale``.
- Overcorrects at large tilt (slams to max and rebounds): increase ``sScale``.
- Balances but drifts away: increase ``kPhi`` or ``kPhiDot`` slightly.
- Buzzing / noise near upright: widen ``thetaDeadband`` / ``thetaDotDeadband``.
"""

try:
    import math as _math
except ImportError:
    try:
        import umath as _math
    except ImportError:
        _math = None

# Resolve tanh once at import time.  umath on LEGO SPIKE MicroPython exists but
# may not expose every function (e.g. tanh is absent on some firmware builds).
# Binding the function object here avoids a per-call AttributeError and lets the
# Padé fallback kick in transparently on constrained runtimes.
try:
    _tanh_fn = _math.tanh  # type: ignore[union-attr]
except AttributeError:
    _tanh_fn = None

from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.ControllerBase import ControllerBase
from LegoBalance.Saturation import SaturateSymmetric


def _Tanh(x):
    """MicroPython-safe tanh.

    Uses ``math.tanh`` (or ``umath.tanh``) when available. Falls back to a
    Padé [1,1] rational approximant accurate to ±0.025 for |x| ≤ 2.5, with
    a hard clamp to ±1 for |x| ≥ 3 (``|tanh(3)| = 0.9951 ≈ 1``).

    The fallback covers two cases:
    - platforms that have neither ``math`` nor ``umath`` (very rare)
    - platforms where ``umath`` exists but does not expose ``tanh`` (some LEGO
      SPIKE MicroPython firmware builds)
    """
    if _tanh_fn is not None:
        return _tanh_fn(x)
    # Fallback: Padé [1,1] approximant x*(27 + x²) / (27 + 9x²)
    # Error < 2.2 % for |x| ≤ 2.5; hard clamp beyond that.
    if x >= 3.0:
        return 1.0
    if x <= -3.0:
        return -1.0
    x2 = x * x
    return x * (27.0 + x2) / (27.0 + 9.0 * x2)


class NonLinearController(ControllerBase):
    """Tanh composite-variable velocity controller for pure sagittal balance.

    Model-light by design: dominant tilt / tilt-rate feedback via tanh, weak
    wheel-position / wheel-rate drift suppression. No gravity feedforward, no
    exact feedback linearization, no reliance on mass or inertia estimates.
    """

    def __init__(self, config) -> None:
        ControllerBase.__init__(self, config)
        self._maxWheelRate = config.control.maxWheelRate

        # uSoftMax: tanh scales to this limit, matching the hard saturation
        # ceiling. Because tanh asymptotically approaches ±1, the command
        # approaches ±uSoftMax but never exceeds it. uSoftMax = maxWheelRate
        # means the soft and hard limits coincide; SaturateSymmetric is a
        # safety backstop for floating-point edge cases only.
        self._uSoftMax = config.control.maxWheelRate

        # sScale: normalises the composite variable before it enters tanh.
        # Near upright the effective linear tilt gain is uSoftMax*kTheta/sScale.
        # Smaller sScale → sharper nonlinear "knee" (more assertive at moderate
        # tilt). Larger sScale → gentler response (more linear over wider range).
        # getattr fallback keeps the hub package working when the frozen
        # ControllerConfig on the hub predates this field.
        self._sScale = getattr(config.controller, "sScale", 15.0)

        # Gains from config.
        # kTheta and kThetaDot are the dominant balancing terms; kPhi and
        # kPhiDot are kept small to avoid amplifying noisy wheel-state estimates.
        cc = config.controller
        self._kTheta = cc.kTheta
        self._kThetaDot = cc.kThetaDot
        self._kPhi = cc.kPhi
        self._kPhiDot = cc.kPhiDot
        self._thetaDeadband = cc.thetaDeadband
        self._thetaDotDeadband = cc.thetaDotDeadband

        # Balance-point reference. If the estimator's zero does not align exactly
        # with the physical upright, there will be a constant tilt bias that drives
        # phi monotonically. Subtracting targetTilt from theta shifts the control
        # law to track the configured balance point instead of hard-coded zero.
        self._targetTilt = config.control.targetTilt

        # Optional first-order IIR low-pass filter for thetaDot.
        # alpha = 0.0  → pure passthrough, controller stays stateless (default).
        # alpha = 0.2  → light filter, cutoff ≈ 25 Hz at 100 Hz — removes
        #                 high-frequency gyro noise with only ~11° phase lag at 5 Hz.
        # DO NOT use alpha > 0.3 for oscillation suppression: a heavy filter adds
        # phase lag at the control frequency and will worsen gain-instability
        # oscillations. Use it only to cut noise above the control bandwidth.
        self._filterAlpha = getattr(cc, "thetaDotFilterAlpha", 0.0)
        self._thetaDotFiltered = 0.0  # filter state; cleared by Reset()

    def IsPlaceholder(self) -> bool:
        """Report that the controller contains a real balancing law."""
        return False

    def _ApplyDeadband(self, value, width):
        """Continuous deadband: zero inside [-width, +width], slope-1 outside.

        Avoids a step discontinuity at the deadband boundary so the composite
        variable s stays continuous and the tanh output stays smooth.

            out = value - sign(value) * width    for |value| > width
            out = 0                              for |value| <= width
        """
        if value != value:   # NaN guard
            return 0.0
        if width <= 0.0:
            return value
        if value > width:
            return value - width
        if value < -width:
            return value + width
        return 0.0

    def _ComputeVelocityCommand(self, theta, thetaDot, phi, phiDot):
        """Evaluate the tanh composite-variable control law.

        Returns the raw (pre-hard-saturation) wheel-velocity command in rad/s.
        Mutates self._thetaDotFiltered when filterAlpha > 0 (stateful path).

        Steps:
        1. Subtract targetTilt from theta so the controller tracks the
           configured balance point rather than hard-coded zero. This
           eliminates the persistent tilt bias that drives phi drift.
        2. Optionally smooth thetaDot through a first-order IIR low-pass
           filter to cut high-frequency gyro noise.
        3. Apply deadbands to suppress jitter near upright.
        4. Form the composite stabilising variable s.
        5. Map s through tanh → smooth, bounded velocity command.
        """
        # Step 1: balance-point offset. A non-zero targetTilt means the robot
        # balances at a slight lean rather than perfectly vertical; subtracting
        # it shifts the zero of the restoring law to that lean angle.
        theta_error = theta - self._targetTilt

        # Step 2: optional IIR low-pass on thetaDot.
        # alpha=0 → passthrough (no state, no phase lag).
        # alpha=0.2 → light noise filter, cutoff ≈ 25 Hz (minimal phase lag).
        if self._filterAlpha > 0.0:
            self._thetaDotFiltered = (
                self._filterAlpha * self._thetaDotFiltered
                + (1.0 - self._filterAlpha) * thetaDot
            )
            thetaDot_in = self._thetaDotFiltered
        else:
            thetaDot_in = thetaDot

        # Step 3: deadbands — suppress sensor jitter on tilt channels only
        theta_db    = self._ApplyDeadband(theta_error, self._thetaDeadband)
        thetaDot_db = self._ApplyDeadband(thetaDot_in, self._thetaDotDeadband)

        # Step 4: composite stabilising variable.
        # Tilt terms (kTheta, kThetaDot) dominate — they correct the primary
        # instability. Wheel terms (kPhi, kPhiDot) are weak secondary drift
        # suppressors; their negative sign opposes persistent displacement.
        s = (
            self._kTheta    * theta_db
            + self._kThetaDot * thetaDot_db
            - self._kPhi    * phi
            - self._kPhiDot * phiDot
        )

        # Smooth bounded velocity command.
        # tanh(s/sScale) ∈ (-1, 1); scaled by uSoftMax it stays within the
        # motor rate limit without any discontinuous clipping.
        if self._sScale <= 0.0:
            return 0.0
        return self._uSoftMax * _Tanh(s / self._sScale)

    def _ClampCommand(self, command):
        """Hard saturation backstop at the motor rate limit.

        Since uSoftMax == maxWheelRate the tanh already keeps the command
        within bounds. This call guards against floating-point edge cases.
        """
        return SaturateSymmetric(command, self._maxWheelRate)

    def Compute(self, state):
        """Compute one bounded symmetric wheel-velocity command.

        Pipeline:
        1. Guard: invalid state or any NaN in the state vector → return a stop
           command immediately so the safety monitor can take over.
        2. Evaluate the tanh composite-variable law to obtain the raw command.
        3. Apply hard saturation to [-maxWheelRate, +maxWheelRate].
        4. Return equal left/right commands for pure sagittal balance.

        The controller is stateless: each call depends only on the current
        state, not on any internal dynamics. This makes it safe under actuator
        saturation and well-defined after transient disturbances.
        """
        if not state.valid:
            return ControlOutput.Stop(mode=ControlMode.Velocity, timestamp=state.timestamp)

        theta = state.theta
        thetaDot = state.thetaDot
        phi = state.phi
        phiDot = state.phiDot

        if (
            theta != theta
            or thetaDot != thetaDot
            or phi != phi
            or phiDot != phiDot
        ):
            return ControlOutput.Stop(mode=ControlMode.Velocity, timestamp=state.timestamp)

        command = self._ComputeVelocityCommand(theta, thetaDot, phi, phiDot)
        boundedCommand = self._ClampCommand(command)

        return ControlOutput(
            leftCommand=boundedCommand,
            rightCommand=boundedCommand,
            mode=ControlMode.Velocity,
            timestamp=state.timestamp,
        )

    def Reset(self) -> None:
        """Clear controller state.

        When thetaDotFilterAlpha = 0 (default) the controller is stateless and
        this is a no-op. When a filter is active, thetaDotFiltered must be
        cleared so stale history from a previous run does not corrupt the first
        commands after a restart.
        """
        self._thetaDotFiltered = 0.0
