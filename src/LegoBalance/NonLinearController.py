"""Nonlinear balancing controller for the LEGO SPIKE inverted pendulum.

This module implements the repository's first real balancing controller inside
the existing `NonLinearController` boundary. The controller is intentionally
practical rather than mechanically exact:

- the estimator already provides the sign-corrected SI state
  ``x = [theta, thetaDot, phi, phiDot]^T``,
- the output boundary is a wheel *velocity* command, not axle torque,
- Pybricks `Motor.run(...)` supplies the inner velocity loop,
- therefore the outer controller is designed directly around measured state and
  commanded wheel velocity.

Control objective
-----------------

The controller prioritizes sagittal upright balance:

1. drive ``theta -> 0`` and ``thetaDot -> 0``,
2. keep ``phiDot`` bounded in steady state,
3. weakly pull ``phi`` back toward zero to reduce runaway drift.

Left and right wheel commands remain equal because yaw control is out of scope.

Sliding variable and boundary layer
-----------------------------------

A practical sliding variable is

    sigma = thetaDot
            + lambdaTheta * theta
            + lambdaPhiDot * phiDot
            + lambdaPhi * phi

with ``lambdaTheta`` dominant and the wheel terms weaker. A candidate reaching
energy is

    V = 0.5 * sigma^2

so ``Vdot = sigma * sigmaDot``. Classical sliding mode control would shape

    sigmaDot = -k * sign(sigma)

which gives ``Vdot = -k * |sigma| < 0`` away from the origin, but the hard
`sign(...)` switch chatters badly on sampled LEGO hardware because of sensor
noise, backlash, quantization, and motor dead zones.

This implementation replaces the discontinuous switch with a boundary-layer
saturation:

    sat(z) = z      for |z| <= 1
           = sign(z) otherwise

applied as ``sat(sigma / epsilon)``. Outside the boundary layer the reaching
behavior stays strong. Inside the layer the controller behaves like a
continuous high-gain stabilizer, which is much less jittery on the real robot.

Practical actuator assumption
-----------------------------

Because the command crossing the controller boundary is wheel velocity rather
than wheel torque, this is a sliding-mode-inspired *outer-loop* controller,
not a strict torque-input SMC derivation from the full rigid-body equations.
That matches both the repo contract and the intended Pybricks deployment path.
"""

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.ControllerBase import ControllerBase
from LegoBalance.Saturation import SaturateSymmetric


class NonLinearController(ControllerBase):
    """Sliding-mode-inspired velocity controller for pure sagittal balance."""

    def __init__(self, config) -> None:
        ControllerBase.__init__(self, config)
        self._maxWheelRate = config.control.maxWheelRate

        # Starter gains are kept local so the first real controller can land
        # without widening the config/generation surface. They are chosen so
        # tilt stabilization dominates wheel centering, matching the project
        # priority order and keeping the hot path MicroPython-friendly.
        #
        # Tuning priorities:
        # - If the robot cannot catch itself, increase kTheta, kThetaDot,
        #   and/or kSigma.
        # - If it oscillates, increase kThetaDot and/or widen epsilon.
        # - If it balances but slowly drives away, increase kPhi or lambdaPhi.
        # - If commands chatter, widen epsilon and possibly reduce kSigma.
        self._lambdaTheta = 6.0
        self._lambdaPhiDot = 0.05
        self._lambdaPhi = 0.01

        # Positive tilt means leaning forward, and positive wheel velocity is
        # the corrective action in this repo's sign convention. That makes the
        # tilt terms positive here, while phi/phiDot terms oppose drift.
        self._kTheta = 30.0
        self._kThetaDot = 7.5
        self._kPhi = 1.5
        self._kPhiDot = 3.0
        self._kSigma = 2.5
        self._boundaryLayerWidth = 0.2

        self._lastTimestamp = 0.0
        self._lastSlidingVariable = 0.0
        self._lastCommand = 0.0

    def IsPlaceholder(self) -> bool:
        """Report that the controller now contains a real balancing law."""
        return False

    def _ComputeSlidingVariable(self, theta, thetaDot, phi, phiDot):
        return (
            thetaDot
            + self._lambdaTheta * theta
            + self._lambdaPhiDot * phiDot
            + self._lambdaPhi * phi
        )

    def _Sat(self, value):
        """Piecewise-linear saturation used for the switching term."""
        if value != value:
            return 0.0
        if value > 1.0:
            return 1.0
        if value < -1.0:
            return -1.0
        return value

    def _ComputeVelocityCommand(self, theta, thetaDot, phi, phiDot):
        sigma = self._ComputeSlidingVariable(theta, thetaDot, phi, phiDot)

        # The linear part gives smooth local stabilization. The boundary-layer
        # term adds robustness against model mismatch and unmodeled bias
        # without the hard switching that would chatter on LEGO hardware.
        linearCommand = (
            self._kTheta * theta
            + self._kThetaDot * thetaDot
            - self._kPhi * phi
            - self._kPhiDot * phiDot
        )
        switchingCommand = self._kSigma * self._Sat(sigma / self._boundaryLayerWidth)
        return linearCommand + switchingCommand, sigma

    def _ClampCommand(self, command):
        return SaturateSymmetric(command, self._maxWheelRate)

    def Compute(self, state):
        """Compute one bounded symmetric wheel-velocity command.

        Invalid states stop immediately. Valid states are mapped to the
        sliding variable ``sigma`` and then to a saturated wheel-rate command.
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

        command, sigma = self._ComputeVelocityCommand(theta, thetaDot, phi, phiDot)
        boundedCommand = self._ClampCommand(command)

        self._lastTimestamp = state.timestamp
        self._lastSlidingVariable = sigma
        self._lastCommand = boundedCommand

        return ControlOutput(
            leftCommand=boundedCommand,
            rightCommand=boundedCommand,
            mode=ControlMode.Velocity,
            timestamp=state.timestamp,
        )

    def Reset(self) -> None:
        """Reset internal bookkeeping used by the controller."""
        self._lastTimestamp = 0.0
        self._lastSlidingVariable = 0.0
        self._lastCommand = 0.0
