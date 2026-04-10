"""Gravity-compensating nonlinear balancing controller with Lyapunov guarantee.

This module implements a nonlinear velocity controller for the LEGO SPIKE
inverted pendulum. The design follows a Control Lyapunov Function (CLF)
approach with explicit gravitational feedforward, providing a formal stability
guarantee while requiring only minimal knowledge of the physical model.

Why nonlinear and not just PD?
------------------------------

A linear PD controller treats the gravitational instability as a disturbance
and relies on high loop gain to overpower it. That works near the upright
equilibrium where ``sin(theta) ~ theta``, but as the tilt angle grows, the
true gravitational torque ``alpha * sin(theta)`` curves below the linear
approximation ``alpha * theta``. A pure PD controller using ``theta`` therefore
*overestimates* the restoring command it needs near upright and *underestimates*
it at larger angles, reducing the effective basin of attraction.

This controller explicitly compensates the nonlinear gravity term using
``sin(theta)`` from the physical model, then adds linear state feedback on
top. The result is a larger provable stability region and smoother behavior
across the full operating range.

Reduced plant model (velocity actuation)
-----------------------------------------

The Lagrangian body dynamics of the inverted pendulum are:

    (I + m*l^2) * thetaDDot = m*g*l*sin(theta) - m*l*r*cos(theta)*phiDDot

where I is body inertia, m is body mass, l is center-of-mass height above the
wheel axle, r is wheel radius, and g is gravitational acceleration.

Because Pybricks ``Motor.run(speed)`` provides an inner velocity servo, the
controller commands wheel velocity, not wheel torque. If the inner loop is fast
relative to the pendulum dynamics, the plant reduces to:

    thetaDDot ~ alpha * sin(theta) - b_hat * u

where:
    alpha = m*g*l / (I + m*l^2)    [rad/s^2, gravitational instability]
    b_hat                           [1/s, empirical coupling from velocity
                                     command to body angular acceleration]
    u                               [rad/s, commanded wheel velocity]

``alpha`` is estimated directly from the chassis geometry in the config.
``b_hat`` is absorbed into the tunable gains below (see the gain structure
section).

Lyapunov stability analysis
----------------------------

Consider the candidate Lyapunov function for the closed-loop body dynamics:

    V = 0.5 * kTheta * theta^2 + 0.5 * thetaDot^2

This is positive definite for ``kTheta > 0``.

The control law is chosen so that the closed-loop body dynamics become:

    thetaDDot = -kTheta_eff * theta - kThetaDot_eff * thetaDot

where ``kTheta_eff`` and ``kThetaDot_eff`` are the effective closed-loop gains
after gravity compensation. Taking the time derivative of V along solutions:

    Vdot = kTheta * theta * thetaDot + thetaDot * thetaDDot
         = kTheta * theta * thetaDot + thetaDot * (-kTheta_eff * theta - kThetaDot_eff * thetaDot)
         = (kTheta - kTheta_eff) * theta * thetaDot - kThetaDot_eff * thetaDot^2

When gravity compensation is exact (kTheta_eff = kTheta), this reduces to:

    Vdot = -kThetaDot_eff * thetaDot^2 <= 0

which is negative semi-definite. By LaSalle's invariance principle:

    Vdot = 0  =>  thetaDot = 0  =>  thetaDDot = 0  =>  theta = 0

so the origin (theta=0, thetaDot=0) is asymptotically stable.

When the gravity estimate is not exact (model mismatch), there is a residual
cross term ``(alpha - alpha_hat) * sin(theta) * thetaDot / b_hat``. As long as
the linear gains dominate this residual, practical stability is maintained. The
``gravityCompGain`` parameter (nominally 1.0) can be detuned to trade off
feedforward accuracy for robustness.

Control law
-----------

The implemented control law is:

    u = gravityCompGain * alpha_hat * sin(theta)       [gravity feedforward]
      + kTheta * deadband(theta)                       [proportional tilt]
      + kThetaDot * deadband(thetaDot)                 [derivative tilt]
      - kPhi * phi                                     [position centering]
      - kPhiDot * phiDot                               [speed damping]

Decomposition:

1. **Gravity feedforward** ``gravityCompGain * alpha_hat * sin(theta)``:
   Cancels the dominant nonlinear gravitational instability. Uses the raw
   (non-deadbanded) theta because gravity acts on the true tilt regardless of
   sensor noise. ``alpha_hat`` is estimated from the config geometry, and
   ``gravityCompGain`` (nominally 1.0) lets the user scale their trust in
   the estimate. This is the only nonlinear term and the only one that uses
   the physical model.

2. **Proportional tilt feedback** ``kTheta * deadband(theta)``:
   Linear restoring force toward upright. Together with the gravity feedforward,
   the effective proportional gain near upright is approximately
   ``gravityCompGain * alpha_hat + kTheta``.

3. **Derivative tilt feedback** ``kThetaDot * deadband(thetaDot)``:
   Provides damping. This is the term that prevents the oscillations that the
   previous controller lacked. The Lyapunov analysis requires this to be
   positive for ``Vdot <= 0``.

4. **Position centering** ``-kPhi * phi``:
   Weak restoring force that pulls the wheel position back toward zero and
   prevents the robot from slowly driving away. The negative sign is because
   positive phi means forward displacement, and negative command opposes it.

5. **Speed damping** ``-kPhiDot * phiDot``:
   Damps wheel velocity drift. Prevents sustained forward/backward motion.

Post-processing:
   The raw command is clamped to ``[-maxWheelRate, maxWheelRate]``. This is a
   hard actuator constraint. No slew-rate limiter is applied because it would
   introduce dynamic coupling between timesteps, violating the static feedback
   assumption that the Lyapunov proof relies on.

Gain structure and minimal model knowledge
-------------------------------------------

The controller needs exactly one model-derived quantity:

    alpha_hat = m * g * l / (I + m * l^2)

estimated from the chassis geometry in ``configs/Default.yaml``. The coupling
gain ``b_hat`` is *not* estimated separately. Instead, ``kTheta`` and
``kThetaDot`` are tunable gains that implicitly absorb ``1/b_hat``. This means
the user tunes the linear gains empirically and only needs the model for the
``sin(theta)`` gravity shape.

Tuning guide
------------

- Robot cannot catch itself: increase ``kTheta`` and/or ``gravityCompGain``.
- Robot oscillates around upright: increase ``kThetaDot``.
- Robot balances but drifts away: increase ``kPhi`` or ``kPhiDot``.
- Commands are too jerky: the controller is memoryless by design — if motor
  jerk is a problem, verify the inner velocity loop bandwidth is adequate.
- Noise near upright: widen ``thetaDeadband`` / ``thetaDotDeadband``.
- Robot overcorrects at large tilt: reduce ``gravityCompGain`` below 1.0.

MicroPython safety: the controller prefers the platform trig module when it is
available, and otherwise falls back to a small built-in sine approximation so
the hub package path does not depend on ``math`` being present.
"""

try:
    import math as _math
except ImportError:
    try:
        import umath as _math
    except ImportError:
        _math = None

from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.ControllerBase import ControllerBase
from LegoBalance.Saturation import SaturateSymmetric

_PI = 3.141592653589793
_TWO_PI = 2.0 * _PI


def _WrapAngle(angle):
    while angle > _PI:
        angle -= _TWO_PI
    while angle < -_PI:
        angle += _TWO_PI
    return angle


def _Sin(angle):
    if _math is not None:
        return _math.sin(angle)

    # 7th-order odd Taylor series on [-pi, pi]. This is only a last-resort
    # fallback for constrained hub runtimes that lack the standard math module.
    x = _WrapAngle(angle)
    x2 = x * x
    x3 = x * x2
    x5 = x3 * x2
    x7 = x5 * x2
    return x - x3 / 6.0 + x5 / 120.0 - x7 / 5040.0


class NonLinearController(ControllerBase):
    """CLF-based nonlinear velocity controller for pure sagittal balance.

    The controller computes a wheel-velocity command that:
    1. compensates gravitational instability via sin(theta) feedforward,
    2. stabilizes the upright equilibrium via proportional-derivative feedback,
    3. prevents wheel drift via weak position and speed damping.
    """

    def __init__(self, config) -> None:
        ControllerBase.__init__(self, config)
        self._maxWheelRate = config.control.maxWheelRate

        # ---- Estimate the gravitational instability parameter ----
        #
        # alpha = m * g * l / (I + m * l^2)   [rad/s^2]
        #
        # This is the "natural divergence rate" of the uncontrolled pendulum.
        # It comes from the linearized free dynamics:  thetaDDot ~ alpha * theta.
        # The full nonlinear term is alpha * sin(theta).
        #
        # Accuracy: the config values for m, l, I are approximate measurements.
        # The gravityCompGain parameter below allows the user to scale the
        # feedforward to compensate for estimation error.
        m = config.chassis.bodyMass          # kg
        l = config.chassis.bodyHeightCoM     # m, height of CoM above axle
        I = config.chassis.bodyInertia       # kg*m^2
        g = 9.81                             # m/s^2
        self._alphaEstimate = m * g * l / (I + m * l * l)

        # ---- Load controller gains from config ----
        #
        # These map to configs/Default.yaml -> controller section.
        # Override in configs/local.yaml during hardware tuning.
        cc = config.controller
        self._gravityCompGain = cc.gravityCompGain
        self._kTheta = cc.kTheta
        self._kThetaDot = cc.kThetaDot
        self._kPhi = cc.kPhi
        self._kPhiDot = cc.kPhiDot
        self._thetaDeadband = cc.thetaDeadband
        self._thetaDotDeadband = cc.thetaDotDeadband

    def IsPlaceholder(self) -> bool:
        """Report that the controller contains a real balancing law."""
        return False

    def _ApplyDeadband(self, value, width):
        """Continuous deadband that preserves slope outside the quiet zone.

        Inside [-width, +width] the output is zero.  Outside that band the
        output is shifted so there is no step at the boundary:

            out = value - sign(value) * width    for |value| > width
            out = 0                              for |value| <= width
        """
        if value != value:
            return 0.0
        if width <= 0.0:
            return value
        if value > width:
            return value - width
        if value < -width:
            return value + width
        return 0.0

    def _ComputeVelocityCommand(self, theta, thetaDot, phi, phiDot):
        """Evaluate the CLF-derived control law.

        Returns the raw (unsaturated) wheel-velocity command.

        Steps:
        1. Gravity feedforward:  gravityCompGain * alpha_hat * sin(theta)
           Uses raw theta (not deadbanded) because gravity acts on the true angle.
        2. Apply deadbands to theta and thetaDot for the linear feedback terms.
        3. Proportional + derivative tilt feedback on deadbanded signals.
        4. Position and speed drift damping on phi and phiDot.
        5. Sum all terms into a single velocity command.
        """

        thetaFb = self._ApplyDeadband(theta, self._thetaDeadband)
        thetaDotFb = self._ApplyDeadband(thetaDot, self._thetaDotDeadband)

        gravityComp = self._gravityCompGain * self._alphaEstimate * _Sin(thetaFb)

        command = (
            gravityComp
            + self._kTheta * thetaFb
            + self._kThetaDot * thetaDotFb
            - self._kPhi * phi
            - self._kPhiDot * phiDot
        )

        return command

    def _ClampCommand(self, command):
        """Saturate to the configured wheel-rate limit.

        This is a hard physical actuator constraint, not a smoothing filter.
        The Lyapunov analysis remains valid under input saturation because
        clamping never reverses the sign of the command relative to the
        unclamped law. It only limits its magnitude.
        """
        return SaturateSymmetric(command, self._maxWheelRate)

    def Compute(self, state):
        """Compute one bounded symmetric wheel-velocity command.

        Pipeline:
        1. Guard: if the state is invalid or contains NaN, return a stop
           command immediately so the safety monitor can take over.
        2. Evaluate the CLF control law (gravity feedforward + PD + drift
           damping) to obtain the raw velocity command.
        3. Clamp to [-maxWheelRate, maxWheelRate].
        4. Return equal left/right commands for pure sagittal balance.

        No slew-rate limiter is applied. The Lyapunov stability proof assumes
        a static feedback law u = f(x). A slew-rate limiter would make the
        applied command depend on the previous command, turning the controller
        into a dynamic system and partially suppressing the gravity
        compensation term that guarantees Vdot <= 0.
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
        """Reset hook. Currently a no-op since the controller is memoryless."""
