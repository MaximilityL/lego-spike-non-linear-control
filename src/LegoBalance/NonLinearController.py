"""Geometry aware robust nonlinear balance controller.

This module implements the active balance law for the LEGO SPIKE inverted
pendulum. Unlike the earlier tanh composite variable design, this controller
builds its feedback on the reduced tilt dynamics of the chassis and uses the
identified first order wheel velocity actuator model to map a desired chassis
acceleration into the actual motor command.

Reduced plant model
-------------------

The canonical balance state the estimator provides is::

    x = [theta, thetaDot, phi, phiDot]

with the usual repo convention that positive theta is a forward lean, positive
phi is the wheel base rolled forward, and positive commands drive the chassis
forward. The sagittal tilt dynamics reduce to::

    (I + m*l^2) * thetaDDot = m*g*l*sin(theta) - m*l*a + dTheta

where ``I`` is the pitch inertia of the body about its own center of mass,
``m`` is the body mass, ``l`` is the vertical offset from the wheel axle to
the center of mass, ``a`` is the chassis linear acceleration, and ``dTheta``
lumps every unmodeled effect. Normalizing by ``J = I + m*l^2``::

    thetaDDot = alphaHat*sin(theta) - betaHat*a + DeltaTheta
    alphaHat  = (m*g*l) / J
    betaHat   = (m*l)   / J

Both ``alphaHat`` and ``betaHat`` are derived from configured geometry. The
controller uses nominal values because the LEGO build parameters are only
approximately known; the robust correction below is what soaks up that
uncertainty.

Actuator model
--------------

Pybricks drives the wheels through an internal velocity loop, so the plant
the balance law sees is not a torque source. The Port F sweep gives an
effective first order response::

    phiDotActualDot = (u - phiDot) / tau

where ``u`` is the commanded wheel angular velocity and ``tau`` is the
actuator time constant measured from the step response. In linear position
units this becomes ``a = r*phiDDot = (r/tau)*(u - phiDot)``, which can be
inverted to map a desired chassis acceleration into the wheel velocity
command::

    u = phiDot + (tau / r) * aDes

This mapping is mandatory: without it the controller would issue a velocity
command whose steady state the actuator never actually produces.

Slow outer loop
---------------

The outer loop generates a bounded lean reference from the linear position
and velocity derived from ``phi`` and ``phiDot``::

    p              = r * phi
    pDot           = r * phiDot
    offsetRaw      = -(kOuterP*p + kOuterD*pDot)
    offset         = sat(offsetRaw, maxReferenceTiltOffset)
    thetaRef       = targetTilt + offset

``targetTilt`` stays the configured static bias used to correct assembly
asymmetry. The outer loop adds a slow position centering lean around that
base value. The saturation keeps the recentering loop from asking for an
unreasonable lean that the inner loop could not track. Because the outer
loop is intentionally slow, it does not dominate the fast anti fall loop.

Fast inner loop
---------------

The inner loop uses a Lyapunov style tilt energy function::

    V(e, eDot) = 0.5 * eDot^2 + lambda * (1 - cos(e))

and chooses a nominal desired chassis acceleration so that the ideal small
signal error dynamics satisfy ``eDDot + kD*eDot + lambda*e approx 0``::

    aNom = ((alphaHat + lambda)*sin(e) + kD*eDot) / betaHat

Robust correction
-----------------

To absorb uncertainty in inertia, contact, friction and unmodeled terms, a
smooth sliding surface like robust term is added on top of the nominal law::

    s       = eDot + cSurface*sin(e)
    aRobust = (kRobust / betaHat) * tanh(s / epsilonBoundary)
    aDes    = aNom + aRobust

Two things matter here: the robust term is always smooth (``tanh``, never
``sign``), and ``epsilonBoundary`` is strictly positive. Together that
guarantees the command is continuous even under pure sliding behavior.

Backward compatibility
----------------------

The external interface is unchanged. ``BalanceControllerFactory`` still
instantiates ``NonLinearController(config)`` for both the ``tanh`` and the
``nonlinear`` algorithm aliases, ``LyapunovController`` still subclasses this
class, and the public methods are still ``Compute``, ``Reset``, and
``IsPlaceholder``. The legacy tanh composite variable gains are retained in
config only so existing YAML and the hub runtime mirror keep loading; this
controller does not read them.

MicroPython safety
------------------

Only scalar arithmetic is used. ``_Tanh`` and ``_Sin`` both fall back to
tiny polynomial approximants so this module works on hub firmware builds
that do not expose ``umath.tanh`` or ``umath.sin``. There are no list or
numpy allocations inside the control path.
"""

try:
    import math as _math
except ImportError:
    try:
        import umath as _math
    except ImportError:
        _math = None

# Resolve tanh and sin once at import time. The hub Pybricks MicroPython
# build may expose umath but occasionally omits individual functions. Binding
# the function references here lets the scalar fallbacks kick in without a
# per call AttributeError.
try:
    _tanh_fn = _math.tanh  # type: ignore[union-attr]
except AttributeError:
    _tanh_fn = None

try:
    _sin_fn = _math.sin  # type: ignore[union-attr]
except AttributeError:
    _sin_fn = None

from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.ControllerBase import ControllerBase
from LegoBalance.Saturation import SaturateSymmetric


_GRAVITY = 9.81


def _Tanh(x):
    """MicroPython safe ``tanh``.

    Uses the host ``math.tanh`` or ``umath.tanh`` when available. Otherwise
    falls back to a Pade [1,1] rational approximant ``x*(27 + x^2) / (27 +
    9*x^2)`` with a hard clamp to plus or minus one for ``|x| >= 3``. The
    clamp is fine here because the robust term only needs a smooth bounded
    nonlinearity, not an exact transcendental.
    """
    if _tanh_fn is not None:
        return _tanh_fn(x)
    if x >= 3.0:
        return 1.0
    if x <= -3.0:
        return -1.0
    x2 = x * x
    return x * (27.0 + x2) / (27.0 + 9.0 * x2)


def _Sin(x):
    """MicroPython safe ``sin``.

    Uses the host ``math.sin`` or ``umath.sin`` when available. Otherwise
    falls back to a fifth order Taylor series, which is more than enough
    accuracy for the tilt range the balance controller ever visits before
    the safety monitor trips.
    """
    if _sin_fn is not None:
        return _sin_fn(x)
    x3 = x * x * x
    x5 = x3 * x * x
    return x - x3 / 6.0 + x5 / 120.0


class NonLinearController(ControllerBase):
    """Geometry aware robust nonlinear controller.

    The controller blends three ingredients:

    1. a nominal Lyapunov style acceleration law built from configured
       chassis geometry,
    2. a smooth sliding surface like robust correction that soaks up model
       mismatch, and
    3. a first order actuator lag inversion that turns the desired chassis
       acceleration into the wheel velocity command the Pybricks motor API
       actually accepts.

    On top of the inner loop, a slow outer loop biases the tilt reference
    from the mean wheel position so the robot returns toward its starting
    position without letting wheel state regulation dominate the fast anti
    fall loop. Left and right wheel commands are always equal because the
    project only implements pure sagittal balance.
    """

    def __init__(self, config) -> None:
        ControllerBase.__init__(self, config)

        self._ValidateAndLoadGeometry(config)

        # Command limits come from the control subsection because they are
        # shared with the safety monitor and hub adapters.
        self._maxWheelRate = config.control.maxWheelRate
        self._targetTilt = config.control.targetTilt

        cc = config.controller

        # Nominal small signal response: eDDot + kDamping*eDot + lambda*e = 0
        innerOmega = getattr(cc, "innerNaturalFrequency", 6.0)
        innerZeta = getattr(cc, "innerDampingRatio", 1.0)
        if innerOmega <= 0.0:
            raise ValueError("controller.innerNaturalFrequency must be strictly positive")
        if innerZeta < 0.0:
            raise ValueError("controller.innerDampingRatio must be non negative")
        self._lambdaGain = innerOmega * innerOmega
        self._kDamping = 2.0 * innerZeta * innerOmega

        # Sliding surface and smooth robust correction parameters.
        self._surfaceGain = getattr(cc, "surfaceGain", 3.0)
        self._robustGain = getattr(cc, "robustGain", 2.0)
        self._boundaryLayerWidth = getattr(cc, "boundaryLayerWidth", 0.5)
        if self._surfaceGain < 0.0:
            raise ValueError("controller.surfaceGain must be non negative")
        if self._robustGain < 0.0:
            raise ValueError("controller.robustGain must be non negative")
        if self._boundaryLayerWidth <= 0.0:
            raise ValueError("controller.boundaryLayerWidth must be strictly positive")

        # Outer loop recentering gains and the cap on the tilt offset the
        # outer loop may ever ask the inner loop to track.
        self._outerPositionGain = getattr(cc, "outerPositionGain", 0.0)
        self._outerVelocityGain = getattr(cc, "outerVelocityGain", 0.0)
        self._maxReferenceTiltOffset = getattr(cc, "maxReferenceTiltOffset", 0.1)
        if self._outerPositionGain < 0.0:
            raise ValueError("controller.outerPositionGain must be non negative")
        if self._outerVelocityGain < 0.0:
            raise ValueError("controller.outerVelocityGain must be non negative")
        if self._maxReferenceTiltOffset < 0.0:
            raise ValueError("controller.maxReferenceTiltOffset must be non negative")

        # Actuator time constant and optional tilt rate smoothing. Both are
        # read through getattr so frozen hub configs without the fields
        # still load cleanly.
        self._actuatorTau = getattr(cc, "actuatorTau", 0.0)
        self._filterAlpha = getattr(cc, "thetaDotFilterAlpha", 0.0)
        if self._filterAlpha < 0.0 or self._filterAlpha >= 1.0:
            raise ValueError("controller.thetaDotFilterAlpha must be in [0.0, 1.0)")

        # Controller state. Both fields are cleared by Reset().
        self._thetaDotFiltered = 0.0
        self._previousCommand = 0.0

    # ------------------------------------------------------------------
    # Geometry
    # ------------------------------------------------------------------

    def _ValidateAndLoadGeometry(self, config) -> None:
        """Read and validate the chassis geometry used by the reduced model.

        The controller depends on four numbers: the wheel radius, the body
        mass, the vertical offset from the wheel axle to the body center of
        mass, and the pitch inertia of the body about its own center of
        mass. The configured ``chassis.bodyInertia`` is the nominal CoM
        pitch inertia; the parallel axis term ``m*l^2`` is added internally
        when forming the reduced inertia ``J``.

        When ``bodyInertia`` is zero or missing the controller falls back
        to the rectangular body approximation::

            I = m * (bodyLength^2 + bodyHeight^2) / 12

        This fallback is only a rough estimate and intentionally fails with
        a clear error when neither source of inertia information is
        available, rather than silently substituting a constant.
        """
        chassis = config.chassis
        if chassis.wheelRadius <= 0.0:
            raise ValueError("chassis.wheelRadius must be positive")
        if chassis.bodyMass <= 0.0:
            raise ValueError("chassis.bodyMass must be positive")
        if chassis.bodyHeightCoM <= 0.0:
            raise ValueError("chassis.bodyHeightCoM must be positive")

        self._wheelRadius = chassis.wheelRadius
        self._bodyMass = chassis.bodyMass
        self._centerOfMassHeight = chassis.bodyHeightCoM

        inertia = getattr(chassis, "bodyInertia", 0.0)
        if inertia <= 0.0:
            bodyLength = getattr(chassis, "bodyLength", 0.0)
            bodyHeight = getattr(chassis, "bodyHeight", 0.0)
            if bodyLength > 0.0 and bodyHeight > 0.0:
                # Rectangular body approximation about the pitch axis. This
                # is an estimate used only when no measured or estimated
                # pitch inertia is available. Documented as an estimate so
                # tuning does not silently rely on it.
                inertia = self._bodyMass * (
                    bodyLength * bodyLength + bodyHeight * bodyHeight
                ) / 12.0
            else:
                raise ValueError(
                    "NonLinearController requires chassis.bodyInertia to be "
                    "positive, or both chassis.bodyLength and chassis.bodyHeight "
                    "to be positive so the box fallback can estimate the pitch "
                    "inertia."
                )

        self._bodyInertiaPitch = inertia

        # Reduced tilt model parameters:
        #   J        = I + m*l^2       (reduced pitch inertia about the axle)
        #   alphaHat = m*g*l / J       (gravitational instability parameter)
        #   betaHat  = m*l   / J       (chassis acceleration coupling gain)
        #
        # alphaHat and betaHat are the nominal physics terms used by the
        # CLF style law. The robust correction downstream is what tolerates
        # moderate mismatch between these nominal values and reality.
        reducedInertia = (
            self._bodyInertiaPitch
            + self._bodyMass * self._centerOfMassHeight * self._centerOfMassHeight
        )
        if reducedInertia <= 0.0:
            raise ValueError("reduced tilt inertia must be strictly positive")
        self._reducedInertia = reducedInertia
        self._alphaHat = (
            self._bodyMass * _GRAVITY * self._centerOfMassHeight / reducedInertia
        )
        self._betaHat = self._bodyMass * self._centerOfMassHeight / reducedInertia
        if self._betaHat <= 0.0:
            raise ValueError("reduced coupling betaHat must be strictly positive")

    # ------------------------------------------------------------------
    # Public contract
    # ------------------------------------------------------------------

    def IsPlaceholder(self) -> bool:
        """Report that this module contains a real balancing law."""
        return False

    def Compute(self, state):
        """Map one balance state to a symmetric wheel velocity command.

        Pipeline:

        1. Guard the invalid state and any NaN in the state vector by
           returning a stop command. The safety monitor is still the final
           authority, but the controller itself does not propagate garbage.
        2. Filter the tilt rate lightly if a filter is enabled.
        3. Estimate the chassis acceleration the actuator is currently
           producing, using the previous bounded command and the actuator
           model.
        4. Evaluate the outer loop to get the bounded tilt reference and
           its derivative.
        5. Form the inner error pair.
        6. Compute the desired chassis acceleration from the nominal law
           plus the smooth robust correction.
        7. Invert the actuator model to get the wheel velocity command,
           clamp it to the configured limit, and remember it for the next
           call.
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

        thetaDotUsed = self._PrepareThetaDot(thetaDot)
        aEstPrev = self._EstimatePreviousAcceleration(phiDot)

        thetaRef = self._ComputeReferenceTilt(phi, phiDot)
        thetaRefDot = self._ComputeReferenceTiltDerivative(phiDot, aEstPrev)

        e, eDot = self._ComputeInnerError(theta, thetaDotUsed, thetaRef, thetaRefDot)
        aDes = self._ComputeDesiredAcceleration(e, eDot)
        uRaw = self._MapAccelerationToVelocityCommand(aDes, phiDot)
        uBounded = self._ClampCommand(uRaw)

        # Keep the bounded command for the next call so aEstPrev reflects
        # what the actuator actually received after saturation, not an
        # unsaturated value that never reached the motors.
        self._previousCommand = uBounded

        return ControlOutput(
            leftCommand=uBounded,
            rightCommand=uBounded,
            mode=ControlMode.Velocity,
            timestamp=state.timestamp,
        )

    def Reset(self) -> None:
        """Clear internal state between runs.

        Clears both the optional tilt rate filter and the cached previous
        bounded command, so the first call after ``Reset`` behaves exactly
        like the first call after construction.
        """
        self._thetaDotFiltered = 0.0
        self._previousCommand = 0.0

    # ------------------------------------------------------------------
    # Step helpers
    # ------------------------------------------------------------------

    def _PrepareThetaDot(self, thetaDot):
        """Return the tilt rate the inner loop should use.

        When ``thetaDotFilterAlpha`` is zero the controller is stateless on
        this channel and simply passes the raw rate through. Otherwise a
        single pole IIR is applied. The filter is intentionally light: its
        only purpose is cutting high frequency gyro noise, never providing
        damping.
        """
        if self._filterAlpha > 0.0:
            self._thetaDotFiltered = (
                self._filterAlpha * self._thetaDotFiltered
                + (1.0 - self._filterAlpha) * thetaDot
            )
            return self._thetaDotFiltered
        return thetaDot

    def _EstimatePreviousAcceleration(self, phiDot):
        """Estimate the chassis acceleration from the previous command.

        Under the first order actuator model
        ``phiDotActualDot = (u - phiDot) / tau``, the linear acceleration is
        ``a = r * phiDotActualDot``. Using the previous bounded command for
        ``u`` avoids the algebraic loop that would otherwise appear when the
        reference derivative depends on the acceleration that the current
        command will produce.

        With ``tau = 0`` the actuator model is degenerate and no estimate
        is available, so the method safely reports zero.
        """
        if self._actuatorTau <= 0.0:
            return 0.0
        return (self._wheelRadius / self._actuatorTau) * (
            self._previousCommand - phiDot
        )

    def _ComputeReferenceTilt(self, phi, phiDot):
        """Slow outer loop: produce a bounded tilt reference offset.

        The outer loop is a simple PD on the linear chassis position and
        velocity derived from ``phi`` and ``phiDot`` via the wheel radius.
        The raw offset is saturated to ``maxReferenceTiltOffset`` so the
        outer loop can never ask the inner loop for an unreasonably large
        lean. ``targetTilt`` remains the configured static bias.
        """
        p = self._wheelRadius * phi
        pDot = self._wheelRadius * phiDot
        offsetRaw = -(self._outerPositionGain * p + self._outerVelocityGain * pDot)
        offset = SaturateSymmetric(offsetRaw, self._maxReferenceTiltOffset)
        return self._targetTilt + offset

    def _ComputeReferenceTiltDerivative(self, phiDot, aEstPrev):
        """Analytical derivative of the unclamped reference offset.

        ``thetaRef = targetTilt - kOuterP * p - kOuterD * pDot``. Taking the
        time derivative gives ``thetaRefDot = -kOuterP*pDot - kOuterD*a``
        where ``a`` is the current chassis acceleration. The acceleration is
        estimated from the previous bounded command to avoid an algebraic
        loop. Note that this derivative is of the unclamped reference; when
        the offset is actively saturated the controller is intentionally a
        little looser on the feed forward, which is acceptable because the
        outer loop is slow by design.
        """
        pDot = self._wheelRadius * phiDot
        return -(
            self._outerPositionGain * pDot + self._outerVelocityGain * aEstPrev
        )

    def _ComputeInnerError(self, theta, thetaDotUsed, thetaRef, thetaRefDot):
        """Form the inner loop error and error rate."""
        e = theta - thetaRef
        eDot = thetaDotUsed - thetaRefDot
        return e, eDot

    def _ComputeDesiredAcceleration(self, e, eDot):
        """Nominal Lyapunov style law plus smooth robust correction.

        Nominal term (derived from ``V = 0.5*eDot^2 + lambda*(1 - cos(e))``)::

            aNom = ((alphaHat + lambda)*sin(e) + kD*eDot) / betaHat

        Sliding surface and robust correction::

            s       = eDot + cSurface*sin(e)
            aRobust = (kRobust / betaHat) * tanh(s / epsilonBoundary)

        Together, the desired chassis acceleration is ``aDes = aNom +
        aRobust``. The robust term is always smooth and never uses
        ``sign``: this is intentional so the command is continuous even
        under pure sliding behavior.
        """
        sinE = _Sin(e)
        nominal = (
            (self._alphaHat + self._lambdaGain) * sinE
            + self._kDamping * eDot
        ) / self._betaHat

        surface = eDot + self._surfaceGain * sinE
        robust = (
            (self._robustGain / self._betaHat)
            * _Tanh(surface / self._boundaryLayerWidth)
        )
        return nominal + robust

    def _MapAccelerationToVelocityCommand(self, aDes, phiDot):
        """Invert the first order actuator model.

        ``phiDotActualDot = (u - phiDot) / tau`` gives
        ``u = phiDot + (tau / r) * aDes`` once ``a = r*phiDDot`` is
        substituted. When ``tau`` is zero, the actuator model is degenerate
        and the mapping is not invertible; the controller then falls back
        to commanding the current wheel speed so the returned command is
        at least well defined. Configurations that care about the new law
        must always provide a strictly positive ``actuatorTau``.
        """
        if self._actuatorTau <= 0.0:
            return phiDot
        return phiDot + (self._actuatorTau / self._wheelRadius) * aDes

    def _ClampCommand(self, command):
        """Hard symmetric saturation at the configured wheel rate limit."""
        return SaturateSymmetric(command, self._maxWheelRate)
