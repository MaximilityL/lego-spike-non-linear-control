# Test Strategy

Balancing-robot projects are difficult to test directly on hardware, so this
repository pushes as much logic as possible into pure Python modules that can
run under `pytest` on a laptop.

The goal is not to pretend hardware does not matter. The goal is to keep the
hardware-only surface small and explicit.

## 1. Automated Test Layers

### 1.1. Pure logic and configuration tests

These tests check typed config loading, validation, and unit conversions:

- `tests/test_RobotConfig.py`
- `tests/test_Units.py`
- `tests/test_Saturation.py`
- `tests/test_BalanceState.py`

### 1.2. Estimator tests

These tests verify the implemented state definition and sign conventions:

- `tests/test_StateEstimator.py`

They confirm, for example:

- `phi` and `phiDot` are built from sign-corrected wheel measurements,
- wheel rotation sign matches forward motion,
- linear position and velocity are derived views rather than stored state.

### 1.3. Controller tests

Controller-specific tests cover both balance controllers:

- `tests/test_NonLinearController.py`
- `tests/test_PidController.py`

The nonlinear-controller tests cover:

- invalid-state stop behavior,
- symmetric command output,
- saturation behavior,
- target-tilt offset,
- deadbands,
- `thetaDot` filtering,
- gain influence on the output,
- MicroPython-safe compatibility alias behavior.

The PID tests cover:

- factory selection,
- proportional, integral, derivative, and wheel-position terms,
- deadband behavior,
- integral reset,
- saturation.

### 1.4. Safety tests

`tests/test_SafetyMonitor.py` checks:

- arming and disarming,
- tilt and tilt-rate trips,
- non-finite state and command handling,
- watchdog timeout behavior,
- command clipping to the wheel-rate ceiling.

### 1.5. Integration tests against mocks

These tests run multiple modules together:

- `tests/test_ClosedLoopFlow.py`
- `tests/test_PreBalancingFlow.py`
- `tests/test_ConnectionDiagnostics.py`
- `tests/test_HubDriveSmokeRuntime.py`

They verify:

- end-to-end estimator -> controller -> safety -> logger flow,
- pre-balancing forward/backward/stop behavior,
- generated hub-runtime config parity with desktop config,
- shared estimator and controller behavior under hub-safe config objects.

## 2. Why Dependency Injection Matters

Modules that need hardware receive those dependencies explicitly instead of
reaching for global state. Tests can therefore pass mocks while real runs pass
real adapters.

That is why the same logical pipeline can be exercised in:

- unit tests,
- integration tests,
- desktop simulation,
- real-hub package-backed runs.

## 3. Manual Hardware Checks

Some checks are intentionally not part of `pytest`:

- verifying IMU axis choice on the physical chassis,
- verifying motor mounting direction and encoder sign,
- observing wheel slip and battery effects,
- checking loop timing and Bluetooth behavior on the real hub,
- tuning controller gains on the real robot.

Useful manual scripts are:

- `hub/HubMain.py`
- `hub/HubImuTest.py`
- `hub/HubEncoderTest.py`
- `hub/HubMotorTest.py`
- `hub/HubDriveSmoke.py`
- `src/HubPackageBalance.py`

Useful desktop companions are:

- `scripts/PlotHubMainLive.py`
- `scripts/PlotHubDriveSmoke.py`
- `scripts/PlotHubPackageBalance.py`
- `scripts/PlotHubPackageDriveSmoke.py`

## 4. Recommended Validation Sequence

When changing the estimator, controller, or configuration, a good validation
order is:

1. run the relevant unit tests,
2. run the full `pytest` suite,
3. run `examples/ClosedLoopSimulation.py`,
4. inspect the generated plots if the change affects control behavior,
5. move to real-hub execution only after the shared logic is stable.

## 5. Running The Tests

```bash
source .venv/bin/activate
pytest
pytest tests/test_NonLinearController.py
pytest tests/test_PidController.py
pytest tests/test_StateEstimator.py
pytest tests/test_SafetyMonitor.py
pytest --cov=LegoBalance
```

## 6. What The Tests Do Not Prove

Passing tests does not guarantee the robot is tuned well on hardware. The tests
prove that:

- state conventions are consistent,
- controller contracts are stable,
- safety logic behaves as designed,
- desktop and hub-safe config paths stay aligned.

They do not prove:

- perfect gain tuning,
- robustness to wheel slip,
- robustness to battery variation,
- long-duration real-world balance performance.
