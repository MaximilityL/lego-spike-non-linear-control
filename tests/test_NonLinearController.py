"""Tests for LegoBalance.NonLinearController.

All tests use LoadConfig(applyLocalOverride=False) to stay deterministic
regardless of any local.yaml overrides present on the developer's machine.
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


def test_ComputeReturnsSymmetricVelocityCommand():
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.08, tiltRate=0.25, phi=0.05, phiDot=0.2, timestamp=2.0)
    output = controller.Compute(state)
    assert isinstance(output, ControlOutput)
    assert output.mode == ControlMode.Velocity
    assert output.leftCommand == pytest.approx(output.rightCommand)
    assert output.timestamp == pytest.approx(2.0)


def test_ComputeSaturatesAgainstConfiguredWheelRate():
    """Extreme state must be clamped to maxWheelRate (hard saturation backstop)."""
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=5.0, tiltRate=15.0, phi=-3.0, phiDot=-4.0)
    output = controller.Compute(state)
    assert output.leftCommand == pytest.approx(config.control.maxWheelRate)
    assert output.rightCommand == pytest.approx(config.control.maxWheelRate)


# ---------------------------------------------------------------------------
# Control law structure tests
# ---------------------------------------------------------------------------


def test_ComputeUsesConfiguredControllerGains():
    """Verify the tanh composite-variable law with targetTilt offset:
        theta_error = theta - targetTilt
        s = kTheta*theta_error + kThetaDot*thetaDot - kPhi*phi - kPhiDot*phiDot
        u = uSoftMax * tanh(s / sScale)
    where uSoftMax = config.control.maxWheelRate."""
    config = LoadConfig(applyLocalOverride=False)
    config.control.maxWheelRate = 100.0
    config.control.targetTilt = 0.0   # zero out offset so formula is simple
    config.controller.kTheta = 2.0
    config.controller.kThetaDot = 3.0
    config.controller.kPhi = 4.0
    config.controller.kPhiDot = 5.0
    config.controller.sScale = 10.0
    config.controller.actuatorTau = 0.0
    config.controller.thetaDeadband = 0.0
    config.controller.thetaDotDeadband = 0.0
    config.controller.thetaDotFilterAlpha = 0.0
    controller = NonLinearController(config)

    state = _BuildValidState(tilt=0.3, tiltRate=0.2, phi=0.1, phiDot=0.05)
    output = controller.Compute(state)

    s = (
        2.0 * state.tilt      # theta_error = tilt - 0.0 = tilt
        + 3.0 * state.tiltRate
        - 4.0 * state.phi
        - 5.0 * state.phiDot
    )
    expected = 100.0 * math.tanh(s / 10.0)

    assert output.leftCommand == pytest.approx(expected)
    assert output.rightCommand == pytest.approx(expected)


def test_ComputeAppliesTargetTiltOffset():
    """Controller should use (theta - targetTilt) as the tilt error, not raw theta.
    A non-zero targetTilt shifts the balance point so phi drift is suppressed."""
    config = LoadConfig(applyLocalOverride=False)
    config.control.maxWheelRate = 100.0
    config.control.targetTilt = 0.05   # configured lean offset
    config.controller.kTheta = 10.0
    config.controller.kThetaDot = 0.0
    config.controller.kPhi = 0.0
    config.controller.kPhiDot = 0.0
    config.controller.sScale = 5.0
    config.controller.actuatorTau = 0.0
    config.controller.thetaDeadband = 0.0
    config.controller.thetaDotDeadband = 0.0
    config.controller.thetaDotFilterAlpha = 0.0
    controller = NonLinearController(config)

    # tilt == targetTilt → theta_error = 0 → s = 0 → command = 0
    at_target = controller.Compute(_BuildValidState(tilt=0.05))
    assert at_target.leftCommand == pytest.approx(0.0)

    # tilt > targetTilt → positive error → positive command
    above_target = controller.Compute(_BuildValidState(tilt=0.10))
    s_above = 10.0 * (0.10 - 0.05)
    assert above_target.leftCommand == pytest.approx(100.0 * math.tanh(s_above / 5.0))


def test_ComputeAppliesConfiguredQuietDeadbands():
    """States within both deadbands produce zero command; states outside produce
    the exact tanh value of the deadbanded composite variable."""
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0   # zero offset for predictable formula
    config.controller.kTheta = 10.0
    config.controller.kThetaDot = 10.0
    config.controller.kPhi = 0.0
    config.controller.kPhiDot = 0.0
    config.controller.thetaDeadband = 0.1
    config.controller.thetaDotDeadband = 0.2
    config.controller.sScale = 5.0
    config.controller.actuatorTau = 0.0
    config.controller.thetaDotFilterAlpha = 0.0
    controller = NonLinearController(config)

    quiet = controller.Compute(_BuildValidState(tilt=0.08, tiltRate=-0.15))
    active = controller.Compute(_BuildValidState(tilt=0.3, tiltRate=-0.5))

    # Both components inside deadband → s = 0 → tanh(0) = 0
    assert quiet.leftCommand == pytest.approx(0.0)
    assert quiet.rightCommand == pytest.approx(0.0)

    # Outside deadband: theta_db = 0.3-0.1 = 0.2, thetaDot_db = -0.5+0.2 = -0.3
    s_active = 10.0 * 0.2 + 10.0 * (-0.3)   # = -1.0
    expected_active = config.control.maxWheelRate * math.tanh(s_active / 5.0)
    assert active.leftCommand == pytest.approx(expected_active)
    assert active.rightCommand == pytest.approx(expected_active)


def test_ComputeZeroStateProducesZeroCommand():
    """Zero state maps to s = 0 → tanh(0) = 0 → zero command."""
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0
    controller = NonLinearController(config)
    output = controller.Compute(_BuildValidState())
    assert output.leftCommand == pytest.approx(0.0)
    assert output.rightCommand == pytest.approx(0.0)


def test_ComputeTanhSoftLimitApproachesMaxWheelRateForLargeTilt():
    """Very large composite variable (tiny sScale) drives the tanh to ≈ 1,
    so the command approaches maxWheelRate."""
    config = LoadConfig(applyLocalOverride=False)
    config.controller.sScale = 0.1   # tiny scale → s/sScale huge → tanh ≈ 1
    config.controller.thetaDeadband = 0.0
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.5)
    output = controller.Compute(state)
    assert output.leftCommand == pytest.approx(config.control.maxWheelRate, rel=1e-3)
    assert output.rightCommand == pytest.approx(config.control.maxWheelRate, rel=1e-3)


# ---------------------------------------------------------------------------
# Sign convention test
# ---------------------------------------------------------------------------


def test_ForwardLeanCommandsForwardWheelMotion():
    """Forward tilt (positive theta outside deadband) must produce a positive
    (forward) wheel velocity command — the robot drives under its CoM."""
    config = LoadConfig(applyLocalOverride=False)
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.1)
    output = controller.Compute(state)
    assert output.leftCommand > 0.0
    assert output.rightCommand > 0.0


# ---------------------------------------------------------------------------
# Filter tests
# ---------------------------------------------------------------------------


def test_FilterAlphaZeroIsPassthrough():
    """With filterAlpha = 0 the controller is fully stateless: two identical
    calls always produce the same output regardless of any previous calls."""
    config = LoadConfig(applyLocalOverride=False)
    config.controller.thetaDotFilterAlpha = 0.0
    controller = NonLinearController(config)

    state = _BuildValidState(tilt=0.05, tiltRate=0.1, phi=0.02, phiDot=0.03)
    first = controller.Compute(state)
    second = controller.Compute(state)

    assert second.leftCommand == pytest.approx(first.leftCommand)


def test_FilterAccumulatesStateWhenActive():
    """With filterAlpha > 0 repeated calls with the same tiltRate converge
    the filtered value toward tiltRate, so the output changes across calls."""
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0
    config.controller.thetaDotFilterAlpha = 0.8
    config.controller.kTheta = 0.0    # isolate the thetaDot path
    config.controller.kThetaDot = 10.0
    config.controller.kPhi = 0.0
    config.controller.kPhiDot = 0.0
    config.controller.thetaDeadband = 0.0
    config.controller.thetaDotDeadband = 0.0
    controller = NonLinearController(config)

    state = _BuildValidState(tiltRate=0.5)
    first = controller.Compute(state)   # filter sees 0→0.5, output is small
    for _ in range(30):
        controller.Compute(state)       # filter converges toward 0.5
    converged = controller.Compute(state)

    # After convergence the filtered thetaDot ≈ 0.5, so command > first call
    assert abs(converged.leftCommand) > abs(first.leftCommand)


def test_ResetClearsFilterState():
    """Reset() must zero the filter so the next run starts clean.
    After Reset the output matches the very-first call, not the converged value."""
    config = LoadConfig(applyLocalOverride=False)
    config.control.targetTilt = 0.0
    config.controller.thetaDotFilterAlpha = 0.8
    config.controller.kTheta = 0.0
    config.controller.kThetaDot = 10.0
    config.controller.kPhi = 0.0
    config.controller.kPhiDot = 0.0
    config.controller.thetaDeadband = 0.0
    config.controller.thetaDotDeadband = 0.0
    controller = NonLinearController(config)

    state = _BuildValidState(tiltRate=0.5)
    firstCall = controller.Compute(state)

    # Let the filter converge
    for _ in range(30):
        controller.Compute(state)

    controller.Reset()
    afterReset = controller.Compute(state)

    # After reset, output should equal the very first call
    assert afterReset.leftCommand == pytest.approx(firstCall.leftCommand)


def test_ResetIsNoOpWhenFilterDisabled():
    """When filterAlpha = 0 the controller is stateless, so Reset() has no
    observable effect: output is the same before and after."""
    config = LoadConfig(applyLocalOverride=False)
    config.controller.thetaDotFilterAlpha = 0.0
    controller = NonLinearController(config)
    state = _BuildValidState(tilt=0.05, tiltRate=0.1, phi=0.02, phiDot=0.03, timestamp=3.0)
    before = controller.Compute(state)
    controller.Reset()
    after = controller.Compute(state)
    assert after.leftCommand == pytest.approx(before.leftCommand)
    assert after.rightCommand == pytest.approx(before.rightCommand)


# ---------------------------------------------------------------------------
# Backward-compatibility alias
# ---------------------------------------------------------------------------


def test_LegacyLyapunovControllerAliasStillWorks():
    config = LoadConfig(applyLocalOverride=False)
    controller = LyapunovController(config)
    assert controller.IsPlaceholder() is False


# ---------------------------------------------------------------------------
# Config field sensitivity tests
#
# Each test changes exactly one config field and verifies the controller
# output changes in the expected direction.  These tests are the proof that
# every YAML knob is wired through to the control law — not just loaded and
# silently ignored.
# ---------------------------------------------------------------------------


def _BaseConfig():
    """Minimal, predictable config for sensitivity tests.

    All fields zeroed or neutralised except for the ones under test so that
    each test can reason about a single parameter in isolation.
    """
    config = LoadConfig(applyLocalOverride=False)
    config.control.maxWheelRate = 20.0
    config.control.targetTilt = 0.0
    config.controller.kTheta = 10.0
    config.controller.kThetaDot = 10.0
    config.controller.kPhi = 5.0
    config.controller.kPhiDot = 5.0
    config.controller.sScale = 10.0
    config.controller.actuatorTau = 0.0
    config.controller.thetaDotFilterAlpha = 0.0
    config.controller.thetaDeadband = 0.0
    config.controller.thetaDotDeadband = 0.0
    return config


def test_kThetaScalesOutputWithTilt():
    """Doubling kTheta doubles the composite variable and increases the command."""
    state = _BuildValidState(tilt=0.2)

    config_lo = _BaseConfig()
    config_lo.controller.kTheta = 5.0
    out_lo = NonLinearController(config_lo).Compute(state)

    config_hi = _BaseConfig()
    config_hi.controller.kTheta = 10.0
    out_hi = NonLinearController(config_hi).Compute(state)

    assert out_hi.leftCommand > out_lo.leftCommand


def test_kThetaDotScalesOutputWithTiltRate():
    """Doubling kThetaDot for a state with non-zero tiltRate increases the command."""
    state = _BuildValidState(tiltRate=0.3)

    config_lo = _BaseConfig()
    config_lo.controller.kTheta = 0.0   # isolate thetaDot path
    config_lo.controller.kThetaDot = 5.0
    out_lo = NonLinearController(config_lo).Compute(state)

    config_hi = _BaseConfig()
    config_hi.controller.kTheta = 0.0
    config_hi.controller.kThetaDot = 10.0
    out_hi = NonLinearController(config_hi).Compute(state)

    assert out_hi.leftCommand > out_lo.leftCommand


def test_kPhiOpposesWheelPosition():
    """Positive phi produces a negative contribution to s, reducing a forward command."""
    # tilt gives a positive command; phi must subtract from it
    state = _BuildValidState(tilt=0.4, phi=0.5)

    config_no_phi = _BaseConfig()
    config_no_phi.controller.kPhi = 0.0
    config_no_phi.controller.kPhiDot = 0.0
    out_no_phi = NonLinearController(config_no_phi).Compute(state)

    config_with_phi = _BaseConfig()
    config_with_phi.controller.kPhiDot = 0.0
    # positive kPhi subtracts phi from s
    out_with_phi = NonLinearController(config_with_phi).Compute(state)

    assert out_with_phi.leftCommand < out_no_phi.leftCommand


def test_kPhiDotOpposesWheelRate():
    """Positive phiDot produces a negative contribution to s, reducing a forward command."""
    state = _BuildValidState(tilt=0.4, phiDot=0.5)

    config_no = _BaseConfig()
    config_no.controller.kPhi = 0.0
    config_no.controller.kPhiDot = 0.0
    out_no = NonLinearController(config_no).Compute(state)

    config_with = _BaseConfig()
    config_with.controller.kPhi = 0.0
    out_with = NonLinearController(config_with).Compute(state)

    assert out_with.leftCommand < out_no.leftCommand


def test_sScaleSofterKneeWithLargerValue():
    """Larger sScale → smaller s/sScale → tanh further from saturation → smaller command."""
    state = _BuildValidState(tilt=0.3)

    config_tight = _BaseConfig()
    config_tight.controller.sScale = 5.0
    out_tight = NonLinearController(config_tight).Compute(state)

    config_wide = _BaseConfig()
    config_wide.controller.sScale = 20.0
    out_wide = NonLinearController(config_wide).Compute(state)

    assert out_tight.leftCommand > out_wide.leftCommand


def test_maxWheelRateScalesSaturationCeiling():
    """Doubling maxWheelRate doubles the soft saturation ceiling and output."""
    state = _BuildValidState(tilt=0.3)

    config_lo = _BaseConfig()
    config_lo.control.maxWheelRate = 10.0
    out_lo = NonLinearController(config_lo).Compute(state)

    config_hi = _BaseConfig()
    config_hi.control.maxWheelRate = 20.0
    out_hi = NonLinearController(config_hi).Compute(state)

    assert out_hi.leftCommand > out_lo.leftCommand


def test_targetTiltShiftsZeroCommandAngle():
    """When tilt == targetTilt the error is zero → command is zero."""
    for target in (0.0, 0.05, -0.03):
        config = _BaseConfig()
        config.control.targetTilt = target
        out = NonLinearController(config).Compute(_BuildValidState(tilt=target))
        assert out.leftCommand == pytest.approx(0.0), f"failed for targetTilt={target}"


def test_thetaDeadbandSuppressesSmallTilts():
    """A tilt just inside the deadband produces zero command; outside produces non-zero."""
    config = _BaseConfig()
    config.controller.thetaDeadband = 0.1
    config.controller.kThetaDot = 0.0   # isolate tilt path

    inside = NonLinearController(config).Compute(_BuildValidState(tilt=0.09))
    outside = NonLinearController(config).Compute(_BuildValidState(tilt=0.11))

    assert inside.leftCommand == pytest.approx(0.0)
    assert outside.leftCommand != pytest.approx(0.0)


def test_thetaDotDeadbandSuppressesSmallRates():
    """A tiltRate just inside the deadband produces zero command; outside produces non-zero."""
    config = _BaseConfig()
    config.controller.kTheta = 0.0     # isolate tiltRate path
    config.controller.thetaDotDeadband = 0.2

    inside = NonLinearController(config).Compute(_BuildValidState(tiltRate=0.19))
    outside = NonLinearController(config).Compute(_BuildValidState(tiltRate=0.21))

    assert inside.leftCommand == pytest.approx(0.0)
    assert outside.leftCommand != pytest.approx(0.0)


def test_thetaDotFilterAlphaAffectsTransientResponse():
    """With filterAlpha > 0 the first command after a step is smaller than with no filter.

    The IIR filter starts at zero and ramps toward the raw value, so the
    effective thetaDot seen by the controller on the first call is less than
    the raw tiltRate.  This is the observable sign that the filter is wired in.
    """
    state = _BuildValidState(tiltRate=1.0)

    config_off = _BaseConfig()
    config_off.controller.kTheta = 0.0  # isolate thetaDot path
    config_off.controller.thetaDotFilterAlpha = 0.0
    out_off = NonLinearController(config_off).Compute(state)

    config_on = _BaseConfig()
    config_on.controller.kTheta = 0.0
    config_on.controller.thetaDotFilterAlpha = 0.5
    out_on = NonLinearController(config_on).Compute(state)

    # Filter starts at 0, so first effective thetaDot = (1-0.5)*1.0 = 0.5 < 1.0
    assert abs(out_on.leftCommand) < abs(out_off.leftCommand)


def test_ActuatorTauReducesCommandWhenWheelTermsAreActive():
    """Lag compensation should temper the command when wheel terms are used."""
    state = _BuildValidState(tilt=0.25, phi=0.0, phiDot=0.0)

    config_no_tau = _BaseConfig()
    config_no_tau.controller.kTheta = 8.0
    config_no_tau.controller.kThetaDot = 0.0
    config_no_tau.controller.kPhi = 2.0
    config_no_tau.controller.kPhiDot = 2.0
    config_no_tau.controller.actuatorTau = 0.0
    out_no_tau = NonLinearController(config_no_tau).Compute(state)

    config_with_tau = _BaseConfig()
    config_with_tau.controller.kTheta = 8.0
    config_with_tau.controller.kThetaDot = 0.0
    config_with_tau.controller.kPhi = 2.0
    config_with_tau.controller.kPhiDot = 2.0
    config_with_tau.controller.actuatorTau = 0.2
    out_with_tau = NonLinearController(config_with_tau).Compute(state)

    assert out_with_tau.leftCommand < out_no_tau.leftCommand


def test_ActuatorTauHasNoEffectWhenWheelTermsAreDisabled():
    """With kPhi = kPhiDot = 0 the lag compensation should be inert."""
    state = _BuildValidState(tilt=0.25, phi=0.0, phiDot=0.0)

    config_no_tau = _BaseConfig()
    config_no_tau.controller.kTheta = 8.0
    config_no_tau.controller.kThetaDot = 0.0
    config_no_tau.controller.kPhi = 0.0
    config_no_tau.controller.kPhiDot = 0.0
    config_no_tau.controller.actuatorTau = 0.0
    out_no_tau = NonLinearController(config_no_tau).Compute(state)

    config_with_tau = _BaseConfig()
    config_with_tau.controller.kTheta = 8.0
    config_with_tau.controller.kThetaDot = 0.0
    config_with_tau.controller.kPhi = 0.0
    config_with_tau.controller.kPhiDot = 0.0
    config_with_tau.controller.actuatorTau = 0.2
    out_with_tau = NonLinearController(config_with_tau).Compute(state)

    assert out_with_tau.leftCommand == pytest.approx(out_no_tau.leftCommand)
