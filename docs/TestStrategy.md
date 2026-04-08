# Test Strategy

Hardware projects are notoriously hard to test. The strategy here is to push as much
behavior as possible onto pure Python objects that can be exercised with `pytest` on a
laptop, and to keep the irreducible hardware contact surface as thin as possible.

## 1. Three Layers Of Tests

### 1.1. Pure logic tests

Live in `tests/`. Run with `pytest` on the desktop. No hardware needed.

Coverage:

- `RobotConfig` loads, validates, deep merges overrides.
- `Saturation` clamps values, both scalar and componentwise.
- `Units` converts radians, degrees, mm/s, m/s correctly.
- `BalanceState` is a well behaved state container with a stable shape.
- `StateEstimator` returns a `BalanceState` of the right shape regardless of input.
- `LyapunovController` returns a `ControlOutput` and reports its placeholder status.
- `SafetyMonitor` trips on tilt over the limit, on watchdog timeout, on excessive
  control output, and disarms commands when not armed.
- `ConnectionDiagnostics` runs against a `MockHub` and reports correct pass/fail.

### 1.2. Mock based system tests

Also live in `tests/`. They wire up the real desktop side abstractions against
`MockAdapters` so that you can test the data flow end to end. The `closed loop
simulation` example is itself a system test and is partially exercised in unit tests.

### 1.3. Hardware smoke scripts

Live in `hub/` and `examples/`. They are not run by `pytest` because they need a real
hub. They are intentionally small and observable. The expectation is that you run
them manually and read the output.

## 2. Dependency Injection

Every module that needs hardware takes its hardware dependency as a constructor
argument. Tests pass mocks. Production code passes real adapters. There is no global
state and no module level singletons.

## 3. What Is Not Tested Automatically

- Sign conventions on the physical IMU. There is no way to test these without the
  actual robot. Verify them manually with `HubImuTest.py` before any motor commands.
- Real motor torque constants. These need a characterization run.
- Bluetooth pairing. This is a property of your operating system, not of this code.

## 4. Running The Tests

```bash
source .venv/bin/activate
pytest                        # all tests
pytest tests/test_RobotConfig.py
pytest -k saturation          # by name pattern
pytest --cov=LegoBalance       # coverage
```

## 5. Adding New Tests

- Put new pure logic tests next to existing ones in `tests/`.
- Name files `test_*.py`.
- Use mocks from `LegoBalance.MockAdapters` rather than building your own where
  possible.
- Keep each test focused on one behavior. Many small tests beat one big one.
