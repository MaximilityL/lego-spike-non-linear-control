"""Tests for LegoBalance.NonLinearController.

These tests cover the geometry aware robust nonlinear controller. They
exercise the public contract (stop on invalid and NaN state, symmetric
velocity output, saturation), the inner and outer loop structure (zero
error equilibrium, outer loop tilt bias, actuator aware mapping,
geometry derived alphaHat and betaHat), the reset behavior, the sign
convention, and the backward compatible alias.

All tests use ``LoadConfig(applyLocalOverride=False)`` so results stay
deterministic regardless of any local.yaml overrides present on the
developer machine.
"""

from __future__ import annotations

import math

import pytest

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import ControlMode, ControlOutput
from LegoBalance.LyapunovController import LyapunovController
from LegoBalance.NonLinearController import NonLinearController
from LegoBalance.RobotConfig import LoadConfig


def _BuildValidState(**overrides) -> BalanceState:
    defaults = {
        "tilt": 0.0,
        "tiltRate": 0.0,
        "phi": 0.0,
        "phiDot": 0.0,
        "timestamp": 1.23,
        "valid": True,
    }
    defaults.update(overrides)
    return BalanceState(**defaults)


def _BaseConfig():
    """Deterministic config with the new controller knobs pinned to
    predictable values.

    Every test that wants to reason about the new law in isolation uses
    this helper. It explicitly zeros the outer loop, sets a known actuator
    tau, and picks round numbers for the inner loop so the expected
    command is easy to compute by hand.
    """
    config = LoadConfig(applyLocalOverride=False)
    config.control.maxWheelRate = 20.0
    config.control.targetTilt = 0.0
    config.controller.innerNaturalFrequency = 4.0
    config.controller.innerDampingRatio = 1.0
    config.controller.surfaceGain = 2.0
    config.controller.robustGain = 1.0
    config.controller.boundaryLayerWidth = 1.0
    config.controller.outerPositionGain = 0.0
    config.controller.outerVelocityGain = 0.0
    config.controller.maxReferenceTiltOffset = 0.2
    config.controller.actuatorTau = 0.2
    config.controller.thetaDotFilterAlpha = 0.0
    return config


def _ExpectedCommand(config, theta, thetaDot, phi, phiDot, previousCommand=0.0):
    """Reproduce the controller math in pure python for assertion use."""
    r = config.chassis.wheelRadius
    m = config.chassis.bodyMass
    l = config.chassis.bodyHeightCoM
    I = config.chassis.bodyInertia
    tau = config.controller.actuatorTau
    J = I + m * l * l
    alphaHat = m * 9.81 * l / J
    betaHat = m * l / J

    innerOmega = config.controller.innerNaturalFrequency
    innerZeta = config.controller.innerDampingRatio
    lam = innerOmega * innerOmega
    kD = 2.0 * innerZeta * innerOmega

    cSurface = config.controller.surfaceGain
    kRobust = config.controller.robustGain
    eps = config.controller.boundaryLayerWidth

    kOuterP = config.controller.outerPositionGain
    kOuterD = config.controller.outerVelocityGain
    offsetCap = config.controller.maxReferenceTiltOffset

    p = r * phi
    pDot = r * phiDot
    offsetRaw = -(kOuterP * p + kOuterD * pDot)
    offset = max(-offsetCap, min(offsetCap, offsetRaw))
    thetaRef = config.control.targetTilt + offset

    if tau > 0.0:
        aEstPrev = (r / tau) * (previousCommand - phiDot)
    else:
        aEstPrev = 0.0
    thetaRefDot = -(kOuterP * pDot + kOuterD * aEstPrev)

    e = theta - thetaRef
    eDot = thetaDot - thetaRefDot

    sinE = math.sin(e)
    aNom = ((alphaHat + lam) * sinE + kD * eDot) / betaHat
    s = eDot + cSurface * sinE
    aRobust = (kRobust / betaHat) * math.tanh(s / eps)
    aDes = aNom + aRobust

    if tau > 0.0:
        uRaw = phiDot + (tau / r) * aDes
    else:
        uRaw = phiDot

    maxRate = config.control.maxWheelRate
    if uRaw > maxRate:
        return maxRate
    if uRaw < -maxRate:
        return -maxRate
    return uRaw


# ---------------------------------------------------------------------------
# Basic contract tests
# ---------------------------------------------------------------------------


def test_IsPlaceholderFalseForRealController():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    assert controller.IsPlaceholder() is False


def test_ComputeReturnsStopForInvalidState():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    invalid = BalanceState(tilt=1.0, timestamp=4.5, valid=False)
    output = controller.Compute(invalid)
    assert isinstance(output, ControlOutput)
    assert output.leftCommand == 0.0
    assert output.rightCommand == 0.0
    assert output.mode == ControlMode.Velocity
    assert output.timestamp == pytest.approx(4.5)


def test_ComputeReturnsStopForNaNState():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    nanState = _BuildValidState(tilt=float("nan"), timestamp=7.0)
    output = controller.Compute(nanState)
    assert output.leftCommand == 0.0
    assert output.rightCommand == 0.0
    assert output.timestamp == pytest.approx(7.0)


def test_ComputeReturnsSymmetricVelocityCommand():
    config = _BaseConfig()
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.08, tiltRate=0.25, phi=0.05, phiDot=0.2, timestamp=2.0)
    output = controller.Compute(state)
    assert isinstance(output, ControlOutput)
    assert output.mode == ControlMode.Velocity
    assert output.leftCommand == pytest.approx(output.rightCommand)
    assert output.timestamp == pytest.approx(2.0)


def test_ComputeSaturatesAgainstConfiguredWheelRate():
    """A wildly unbalanced state must be clipped to the configured limit."""
    config = _BaseConfig()
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=10.0, tiltRate=30.0, phi=10.0, phiDot=20.0)
    output = controller.Compute(state)
    assert output.leftCommand == pytest.approx(config.control.maxWheelRate)
    assert output.rightCommand == pytest.approx(config.control.maxWheelRate)


# ---------------------------------------------------------------------------
# Inner and outer loop structure tests
# ---------------------------------------------------------------------------


def test_ZeroErrorEquilibriumGivesZeroCommand():
    """theta == targetTilt with zero rates and zero wheel drift must yield u = 0."""
    config = _BaseConfig()
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.0, tiltRate=0.0, phi=0.0, phiDot=0.0)
    output = controller.Compute(state)
    assert output.leftCommand == pytest.approx(0.0)
    assert output.rightCommand == pytest.approx(0.0)


def test_ZeroErrorEquilibriumAtNonZeroTargetTilt():
    """Equilibrium must still hold at a non zero targetTilt."""
    config = _BaseConfig()
    config.control.targetTilt = 0.05
    controller = NonLinearController(config)
    output = controller.Compute(_BuildValidState(tilt=0.05, tiltRate=0.0))
    assert output.leftCommand == pytest.approx(0.0, abs=1e-9)


def test_ComputeMatchesExpectedFormulaInInnerLoopOnly():
    """The full inner law must match the hand computed value exactly."""
    config = _BaseConfig()
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.15, tiltRate=0.4, phi=0.0, phiDot=0.0)
    output = controller.Compute(state)
    expected = _ExpectedCommand(
        config,
        theta=state.tilt,
        thetaDot=state.tiltRate,
        phi=state.phi,
        phiDot=state.phiDot,
        previousCommand=0.0,
    )
    assert output.leftCommand == pytest.approx(expected)


def test_ForwardPositionErrorBiasesReferenceTilt():
    """Positive phi must drop thetaRef (lean backward) and change the command."""
    config = _BaseConfig()
    config.controller.outerPositionGain = 2.0
    controller = NonLinearController(config)

    # Helper access: internal reference tilt computation. This is a public
    # helper method on the controller so the test can verify the outer
    # loop wiring directly.
    baseRef = controller._ComputeReferenceTilt(phi=0.0, phiDot=0.0)
    biasedRef = controller._ComputeReferenceTilt(phi=1.5, phiDot=0.0)

    # With positive outerPositionGain and phi > 0, the outer loop must
    # pull the reference toward a backward lean (lower than targetTilt).
    assert biasedRef < baseRef

    # And the inner loop commands must reflect that: with theta pinned at
    # targetTilt, the e = theta - thetaRef error is positive, so the
    # forward wheel command grows.
    commandNoDrift = controller.Compute(
        _BuildValidState(tilt=0.0, phi=0.0)
    ).leftCommand
    controller.Reset()
    commandWithDrift = controller.Compute(
        _BuildValidState(tilt=0.0, phi=1.5)
    ).leftCommand
    assert commandWithDrift > commandNoDrift


def test_OuterLoopReferenceTiltOffsetIsSaturated():
    """The outer loop offset must clip at maxReferenceTiltOffset."""
    config = _BaseConfig()
    config.controller.outerPositionGain = 10.0
    config.controller.maxReferenceTiltOffset = 0.05
    controller = NonLinearController(config)

    offsetRef = controller._ComputeReferenceTilt(phi=100.0, phiDot=0.0)
    # targetTilt is 0.0, so the offset alone is the reference.
    assert offsetRef == pytest.approx(-0.05)


# ---------------------------------------------------------------------------
# Geometry and actuator mapping tests
# ---------------------------------------------------------------------------


def test_GeometryDerivedAlphaBetaComputedFromConfig():
    """alphaHat and betaHat must come from configured chassis geometry."""
    config = _BaseConfig()
    config.chassis.bodyMass = 0.4
    config.chassis.bodyHeightCoM = 0.12
    config.chassis.bodyInertia = 0.005
    controller = NonLinearController(config)

    m = 0.4
    l = 0.12
    I = 0.005
    J = I + m * l * l
    expectedAlpha = m * 9.81 * l / J
    expectedBeta = m * l / J

    assert controller._alphaHat == pytest.approx(expectedAlpha)
    assert controller._betaHat == pytest.approx(expectedBeta)


def test_BodyInertiaBoxFallbackWhenExplicitInertiaMissing():
    """If bodyInertia is zero, the box body approximation must kick in."""
    config = _BaseConfig()
    config.chassis.bodyInertia = 0.0
    config.chassis.bodyLength = 0.2
    config.chassis.bodyHeight = 0.15
    controller = NonLinearController(config)

    m = config.chassis.bodyMass
    expected = m * (0.2 * 0.2 + 0.15 * 0.15) / 12.0
    assert controller._bodyInertiaPitch == pytest.approx(expected)


def test_MissingInertiaRaisesClearError():
    """Construction must fail loudly if no inertia source is available."""
    config = _BaseConfig()
    config.chassis.bodyInertia = 0.0
    config.chassis.bodyLength = 0.0
    config.chassis.bodyHeight = 0.0
    with pytest.raises(ValueError):
        NonLinearController(config)


def test_ActuatorAwareMappingUsesTauAndWheelRadius():
    """The acceleration to velocity mapping must use tau and wheelRadius."""
    config = _BaseConfig()
    controller = NonLinearController(config)

    aDes = 3.5
    phiDot = 0.0
    mapped = controller._MapAccelerationToVelocityCommand(aDes, phiDot)
    expected = phiDot + (
        config.controller.actuatorTau / config.chassis.wheelRadius
    ) * aDes
    assert mapped == pytest.approx(expected)


def test_ActuatorTauDirectlyScalesTheCommand():
    """Doubling tau must double the command contribution from the desired
    acceleration. The inner loop is held identical by construction so the
    only difference between the two controllers is the tau mapping."""
    stateKwargs = dict(tilt=0.1, tiltRate=0.0, phi=0.0, phiDot=0.0)

    config_lo = _BaseConfig()
    config_lo.controller.actuatorTau = 0.1
    out_lo = NonLinearController(config_lo).Compute(_BuildValidState(**stateKwargs))

    config_hi = _BaseConfig()
    config_hi.controller.actuatorTau = 0.2
    out_hi = NonLinearController(config_hi).Compute(_BuildValidState(**stateKwargs))

    # aDes depends on tau through aEstPrev in the feedforward, but with
    # phiDot = 0 and previousCommand = 0, aEstPrev is zero in both cases.
    # The inner law yields the same aDes, so the only difference in u is
    # the tau/r factor: u = phiDot + (tau/r)*aDes.
    ratio = out_hi.leftCommand / out_lo.leftCommand
    assert ratio == pytest.approx(2.0, rel=1e-9)


# ---------------------------------------------------------------------------
# Sign convention
# ---------------------------------------------------------------------------


def test_ForwardLeanCommandsForwardWheelMotion():
    """Positive tilt (forward lean) must produce a positive wheel command.

    This follows the repo wide sign convention: the robot drives under its
    center of mass, so positive theta maps to a positive forward wheel
    velocity. The new geometry aware controller must preserve that
    invariant.
    """
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    # tilt well above targetTilt so e is clearly positive
    state = _BuildValidState(tilt=config.control.targetTilt + 0.1)
    output = controller.Compute(state)
    assert output.leftCommand > 0.0
    assert output.rightCommand > 0.0


def test_BackwardLeanCommandsBackwardWheelMotion():
    """Negative tilt (backward lean) must produce a negative wheel command."""
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=config.control.targetTilt - 0.1)
    output = controller.Compute(state)
    assert output.leftCommand < 0.0
    assert output.rightCommand < 0.0


# ---------------------------------------------------------------------------
# Reset behavior
# ---------------------------------------------------------------------------


def test_ResetClearsFilterAndPreviousCommand():
    """Reset must zero both internal state fields so the next call starts clean."""
    config = _BaseConfig()
    config.controller.thetaDotFilterAlpha = 0.5
    controller = NonLinearController(config)

    state = _BuildValidState(tilt=0.15, tiltRate=0.4, phi=0.1, phiDot=0.05)
    controller.Compute(state)
    controller.Compute(state)
    # After two calls, both the filter and the previous command are non zero.
    assert controller._thetaDotFiltered != 0.0
    assert controller._previousCommand != 0.0

    controller.Reset()
    assert controller._thetaDotFiltered == 0.0
    assert controller._previousCommand == 0.0


def test_ResetMakesControllerBehaveAsFreshInstance():
    """Two calls separated by Reset must match a fresh controller's first call."""
    config = _BaseConfig()
    config.controller.thetaDotFilterAlpha = 0.5
    state = _BuildValidState(tilt=0.12, tiltRate=0.2)

    first = NonLinearController(config).Compute(state)

    stateful = NonLinearController(config)
    for _ in range(5):
        stateful.Compute(state)
    stateful.Reset()
    afterReset = stateful.Compute(state)

    assert afterReset.leftCommand == pytest.approx(first.leftCommand)


# ---------------------------------------------------------------------------
# Filter behavior
# ---------------------------------------------------------------------------


def test_FilterAlphaZeroIsPassthrough():
    """With filterAlpha = 0 the tilt rate channel must be stateless."""
    config = _BaseConfig()
    config.controller.thetaDotFilterAlpha = 0.0
    controller = NonLinearController(config)

    state = _BuildValidState(tilt=0.05, tiltRate=0.1)
    first = controller.Compute(state)
    # Even after many calls the output must be identical because the
    # inner law is memoryless when filterAlpha is zero (the previous
    # command feedback does enter through aEstPrev, but only as a slow
    # forward term in the reference derivative; with outer gains = 0
    # that term is zero).
    for _ in range(5):
        controller.Compute(state)
    assert controller.Compute(state).leftCommand == pytest.approx(first.leftCommand)


def test_FilterAccumulatesStateWhenActive():
    """With filterAlpha > 0, repeated calls with the same state must drive
    the filter from its initial zero toward the raw rate, so the command
    changes between the first call and the converged value."""
    config = _BaseConfig()
    config.controller.thetaDotFilterAlpha = 0.7
    controller = NonLinearController(config)

    state = _BuildValidState(tilt=0.0, tiltRate=1.0)
    first = controller.Compute(state)
    for _ in range(50):
        controller.Compute(state)
    converged = controller.Compute(state)
    assert abs(converged.leftCommand) > abs(first.leftCommand)


# ---------------------------------------------------------------------------
# Backward compatibility
# ---------------------------------------------------------------------------


def test_LegacyLyapunovControllerAliasStillWorks():
    config = LoadConfig(applyLocalOverride=False)
    controller = LyapunovController(config)
    assert controller.IsPlaceholder() is False
    output = controller.Compute(_BuildValidState(tilt=0.1))
    assert isinstance(output, ControlOutput)
    assert output.leftCommand == pytest.approx(output.rightCommand)


def test_DefaultConfigLoadsAndBalancesNearUpright():
    """The shipped defaults must build a valid controller whose zero state
    command is bounded and whose equilibrium is at targetTilt."""
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    output = controller.Compute(
        _BuildValidState(tilt=config.control.targetTilt)
    )
    # A tiny residual from the actuator feedforward path is allowed, but
    # it must be a small fraction of the wheel rate limit.
    assert abs(output.leftCommand) < 1e-6
