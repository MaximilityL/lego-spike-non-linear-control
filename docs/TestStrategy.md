# Test Strategy

Hardware projects are notoriously hard to test. The strategy here is to push as much
behavior as possible onto pure Python objects that can be exercised with `pytest` on a
laptop, and to keep the irreducible hardware contact surface as thin as possible.

## 1. Automated Desktop Tests

The repo already has three useful automated layers:

- **Unit and contract tests.** `tests/test_RobotConfig.py`, `tests/test_Units.py`,
  `tests/test_Saturation.py`, `tests/test_BalanceState.py`,
  `tests/test_StateEstimator.py`, `tests/test_NonLinearController.py`,
  `tests/test_DriveCommandController.py`, and `tests/test_SafetyMonitor.py`.
- **Integration tests against mocks.** `tests/test_ClosedLoopFlow.py`,
  `tests/test_PreBalancingFlow.py`, `tests/test_HubDriveSmokeRuntime.py`, and
  `tests/test_ConnectionDiagnostics.py`.
- **Example-backed validation.** `examples/ClosedLoopSimulation.py` and
  `examples/DriveCommandSmoke.py` exercise the same runtime shape as the real app, but
  against mock hardware.

## 2. Dependency Injection

Every module that needs hardware takes its hardware dependency as a constructor
argument. Tests pass mocks. Production code passes real adapters. There is no global
state and no module level singletons.

## 3. Manual Hardware Checks

These are intentionally not part of `pytest`:

- IMU sign verification on the real chassis.
- Motor direction and encoder sign verification on the real chassis.
- Bluetooth discovery and permissions.
- Any behavior that depends on battery condition, wheel slip, or actual hub timing.

Use these scripts for manual checks:

- `hub/HubMain.py` for sensor bring-up.
- `hub/HubDriveSmoke.py` for the self-contained hub smoke flow.
- `python scripts/PlotHubPackageDriveSmoke.py` for the package-backed shared-code smoke
  flow.

## 4. Running The Tests

```bash
source .venv/bin/activate
pytest                        # all tests
pytest tests/test_RobotConfig.py
pytest tests/test_NonLinearController.py
pytest -k saturation          # by name pattern
pytest --cov=LegoBalance       # coverage
```

## 5. Expectations For The Real Balancing Controller

When `NonLinearController` stops being a placeholder, add or update tests for at least:

- invalid-state behavior (`Compute` must still stop cleanly),
- command symmetry for the two-wheel balance case,
- saturation against configured limits,
- any internal controller state cleared by `Reset`,
- placeholder status flipping from `True` to `False` once the real implementation lands.

## 6. Adding New Tests

- Put new pure logic tests next to existing ones in `tests/`.
- Name files `test_*.py`.
- Use mocks from `LegoBalance.MockAdapters` rather than building your own where
  possible.
- Keep each test focused on one behavior. Many small tests beat one big one.
