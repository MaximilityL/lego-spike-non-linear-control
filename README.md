# LEGO SPIKE Inverted Pendulum (Pybricks Starter)

A clean, modular starter environment for building a two wheel self balancing inverted
pendulum robot on the LEGO SPIKE Prime hub using [Pybricks](https://pybricks.com).
The long term target is a Lyapunov based nonlinear balancing controller. This repository
focuses on the scaffolding, bring up, and testing that comes before that controller is
written, so that the controller can be plugged in later without rewriting the project.

This is intentionally a starter, not a finished controller. Every interface is documented,
every hardware coupling point is isolated, and the simulation stub lets you exercise the
control API on a desktop before any hardware is involved.

---

## 1. Project Purpose

- Provide a real, modular Python project layout for a LEGO SPIKE robotics project.
- Support a first time SPIKE user from "can I see the hub" all the way to
  "can I plug in a balancing controller".
- Keep hardware specific code isolated from logic that can be tested on a laptop.
- Be ready to host a future Lyapunov based controller for a two wheel self balancing robot.

## 2. Why Pybricks

Pybricks runs MicroPython directly on the SPIKE Prime hub. It is the right choice for an
inverted pendulum project for several reasons.

- **Deterministic on hub control loops.** Pybricks runs your Python code natively on the
  hub firmware. You do not need a host computer in the loop, so you can run a tight
  control loop entirely on the hub at a stable rate.
- **First class motor and IMU APIs.** Pybricks exposes the SPIKE Prime built in IMU
  (`hub.imu`) and the encoded motors (`Motor`) with simple, well documented Python APIs.
  Both are needed for any inverted pendulum.
- **Open source and transparent.** Unlike the official LEGO Python which is layered on
  top of a proprietary stack, Pybricks is open and the firmware behavior is documented.
  This matters when you start tuning a real controller.
- **Easy onboarding.** You can start using it from the browser at
  [code.pybricks.com](https://code.pybricks.com) without installing anything locally.
- **Scriptable from the command line.** When you outgrow the browser editor you can
  flash and run programs from the desktop with `pybricksdev`.

A short comparison with the alternatives:

| Option            | On hub Python  | Open source | IMU access | Good for tight control loops |
| ----------------- | -------------- | ----------- | ---------- | ---------------------------- |
| Pybricks          | Yes            | Yes         | Yes        | Yes                          |
| LEGO SPIKE App    | Limited        | No          | Yes        | Limited                      |
| EV3 / Mindstorms  | Different hub  | Mixed       | Different  | Not applicable               |

## 3. What Runs Where

This project is split into two halves on purpose.

- **Hub side (`hub/`, plus the package smoke entrypoint under `src/`).** Tiny Pybricks
  programs for the real SPIKE hub. Most are self contained and import only from
  `pybricks.*`. `src/HubPackageDriveSmoke.py` is the deliberate exception: it imports the
  shared `LegoBalance` estimator, controller, and safety modules so you can test package
  logic on hardware.
- **Desktop side (`src/LegoBalance/`).** A normal Python package with type hints,
  dataclasses, tests, and abstract interfaces. This is where the controller, estimator,
  configuration, simulation, and tests live. Most of this does not run on the hub
  directly; hub code can only import modules kept MicroPython-safe, like the smoke-test
  estimator/controller path and `HubDriveSmokeRuntime`.

This split is the most important architectural decision in the repo. Read it twice. It is
what lets you write tested, maintainable Python on your laptop while still being honest
about what the hub can and cannot do.

## 4. Repository Layout

```
lego-spike-invrted-pendulum/
├── README.md                  Project overview and quickstart (this file)
├── CHANGELOG.md               Version history
├── VERSION                    Plain text version, currently 1.0.3
├── pyproject.toml             Desktop side packaging and tool config
├── requirements.txt           Minimal runtime dependencies for desktop side
├── .gitignore
├── configs/
│   ├── Default.yaml           Default robot configuration
│   └── README.md              How to override config values
├── docs/
│   ├── ArchitectureOverview.md
│   ├── HardwareAssumptions.md
│   ├── FutureControlRoadmap.md
│   ├── TestStrategy.md
│   └── PybricksNotes.md
├── src/
│   ├── HubPackageDriveSmoke.py Package-backed Pybricks drive smoke entrypoint
│   └── LegoBalance/           Desktop side Python package
│       ├── __init__.py
│       ├── HubDriveSmokeRuntime.py Hub-safe default config for hardware smoke
│       ├── HubInterface.py
│       ├── MotorInterface.py
│       ├── ImuInterface.py
│       ├── RobotConfig.py
│       ├── DataLogger.py
│       ├── SafetyMonitor.py
│       ├── StateEstimator.py
│       ├── BalanceState.py
│       ├── ControlInterfaces.py
│       ├── ControllerBase.py
│       ├── LyapunovController.py
│       ├── ConnectionDiagnostics.py
│       ├── HardwareTestRunner.py
│       ├── DriveCommandController.py
│       ├── Saturation.py
│       ├── Units.py
│       └── MockAdapters.py
├── hub/                       Code that runs ON the SPIKE hub under Pybricks
│   ├── HubMain.py
│   ├── HubBluetoothTest.py
│   ├── HubMotorTest.py
│   ├── HubImuTest.py
│   ├── HubEncoderTest.py
│   ├── HubSingleMotorTestF.py
│   └── HubDriveSmoke.py       Pre balancing forward/backward/stop smoke test
├── scripts/
│   ├── BootstrapEnv.sh        One shot setup script for the desktop venv
│   ├── DetectHub.py           USB / Bluetooth presence sniff
│   ├── PlotHubMainLive.py     Live plots for telemetry streamed by HubMain
│   ├── PlotHubDriveSmoke.py   Post-run plots for HubDriveSmoke telemetry
│   ├── PlotHubPackageDriveSmoke.py Post-run plots for the package-backed smoke
│   ├── RunDiagnostics.py      Run ConnectionDiagnostics from the CLI
│   └── DeployToHub.md         Notes on flashing and running on the hub
├── examples/
│   ├── ClosedLoopSimulation.py
│   ├── DriveCommandSmoke.py     Pre balancing forward/backward/stop smoke flow
│   ├── EstimatorReadout.py      Print the implemented [theta, thetaDot, phi, phiDot]
│   ├── MotorSmokeTest.py
│   └── ReadSensors.py
└── tests/
    ├── __init__.py
    ├── test_RobotConfig.py
    ├── test_Saturation.py
    ├── test_Units.py
    ├── test_StateEstimator.py
    ├── test_LyapunovController.py
    ├── test_SafetyMonitor.py
    ├── test_BalanceState.py
    ├── test_DriveCommandController.py
    ├── test_PreBalancingFlow.py
    └── test_ConnectionDiagnostics.py
```

## 5. First Time Setup

These steps assume Linux (the rest of the project is platform independent, but the
shell helpers are written for `bash`).

### 5.1. Clone or open the project

```bash
cd ~/maxim_env/python_env/lego-spike-invrted-pendulum
```

### 5.2. Create a virtual environment and install the desktop side

```bash
bash scripts/BootstrapEnv.sh
```

This is equivalent to running:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e .[dev]
```

### 5.3. Run the unit tests

```bash
source .venv/bin/activate
pytest
```

You should see all tests pass. None of these tests need a hub.

### 5.4. Run the desktop simulation stub

```bash
python examples/ClosedLoopSimulation.py
```

This wires the placeholder controller, the state estimator, and the mock hub together,
and prints a few iterations of state and control. It is the fastest way to confirm that
the desktop side abstractions agree with each other.

## 6. Installing and Using Pybricks With LEGO SPIKE

Pybricks needs to be installed on the SPIKE Prime hub once. After that the firmware
stays on the hub and you can run any number of Python programs on it.

### 6.1. One time hub firmware install

1. Open Chrome or Edge (Web Bluetooth is required).
2. Go to [code.pybricks.com](https://code.pybricks.com).
3. Click the gear icon, then "Install Pybricks Firmware".
4. Follow the on screen instructions to put the hub in firmware update mode and flash it.
5. After it reboots the hub is now running Pybricks.

The original LEGO firmware can be restored from the same page if you want to switch
back later. Flashing Pybricks does not damage anything and is reversible.

### 6.2. Running a program from the browser

1. In `code.pybricks.com`, click the connect icon to connect to the hub over Bluetooth.
2. Click "New file" and paste the contents of `hub/HubMain.py` into a new program.
3. Click the run (play) button.
4. The hub status light should change and the IMU values should print in the terminal pane.

### 6.3. Running a program from the command line (optional)

If you installed `pybricksdev` (`pip install -e .[hub]`), you can also run hub code from
your laptop without the browser:

```bash
pybricksdev run ble hub/HubMain.py
```

To run `HubMain.py` and show the live state vector
`x = [theta, theta_dot, p, p_dot]^T`:

```bash
python scripts/PlotHubMainLive.py
```

To add raw left/right motor encoder plots as a slower diagnostic view:

```bash
python scripts/PlotHubMainLive.py --show-motors
```

If discovery fails, pass the hub name through to `pybricksdev`:

```bash
python scripts/PlotHubMainLive.py --name "Pybricks Hub"
```

See [scripts/DeployToHub.md](scripts/DeployToHub.md) for the full set of options and
common error fixes.

## 7. Beginner Friendly Commands

| What you want                                  | Command                                     |
| ---------------------------------------------- | ------------------------------------------- |
| Set up the desktop venv                        | `bash scripts/BootstrapEnv.sh`              |
| Activate the venv                              | `source .venv/bin/activate`                 |
| Install desktop side deps only                 | `pip install -e .`                          |
| Install dev tooling                            | `pip install -e .[dev]`                     |
| Install pybricksdev (optional)                 | `pip install -e .[hub]`                     |
| Run all tests                                  | `pytest`                                    |
| Run tests with coverage                        | `pytest --cov=LegoBalance`                  |
| Lint                                           | `ruff check src tests examples scripts`     |
| Type check                                     | `mypy src`                                  |
| Run the simulation stub                        | `python examples/ClosedLoopSimulation.py`   |
| Run a desktop only motor smoke test (mock)     | `python examples/MotorSmokeTest.py`         |
| Print the implemented [theta, thetaDot, phi, phiDot] state | `python examples/EstimatorReadout.py` |
| Run the pre balancing drive command smoke flow | `python examples/DriveCommandSmoke.py`      |
| Read mock sensors                              | `python examples/ReadSensors.py`            |
| Detect a connected SPIKE hub                   | `python scripts/DetectHub.py`               |
| Run HubMain with live plots                    | `python scripts/PlotHubMainLive.py`         |
| Run HubDriveSmoke with a post-run plot         | `python scripts/PlotHubDriveSmoke.py`       |
| Run package-backed drive smoke with a plot     | `python scripts/PlotHubPackageDriveSmoke.py` |
| Run desktop side connection diagnostics        | `python scripts/RunDiagnostics.py`          |
| Run a Pybricks program on the hub via CLI      | `pybricksdev run ble hub/HubMain.py`        |
| Test Bluetooth terminal connection on the hub  | `pybricksdev run ble hub/HubBluetoothTest.py` |
| Spin motors gently on the real hub             | upload `hub/HubMotorTest.py` and run        |
| Stream IMU data on the real hub                | upload `hub/HubImuTest.py` and run          |

## 8. Common Troubleshooting

- **The hub does not show up in the browser.** Make sure you are using Chrome or Edge,
  Bluetooth is on, and the hub is powered. Hold the bluetooth button until the light blinks.
- **`pybricksdev run` fails to find the hub.** Try `pybricksdev run ble --name "Pybricks Hub"`.
  On Linux you may also need to enable Bluetooth permissions for the user.
- **`ImportError: pybricks` on the desktop.** Pybricks is a hub firmware, not a normal pip
  package. The `pybricks.*` imports only work when the program runs on the hub. The
  `src/LegoBalance/` desktop code never imports from `pybricks`.
- **Tests fail with `ModuleNotFoundError: LegoBalance`.** Run `pip install -e .` from the
  project root after activating the venv.
- **The motors do not move.** Run the safety check first. By default `SafetyMonitor`
  refuses to issue commands until you call `Arm()`.

## 9. Safety Notes For Motor Testing

- **Always block the wheels first.** When you run the motor smoke test for the first time,
  put the robot on a stand or hold it in your hand so the wheels are not loaded.
- **Start at low duty.** All test scripts default to a small angular velocity. Do not
  raise it before you have seen the robot respond at the low value.
- **Have the stop ready.** Pybricks programs can be stopped from the browser (stop button)
  or by pressing the center button on the hub.
- **Power off when wiring.** Always power off the hub before plugging or unplugging a motor.
- **Watch the current.** The `SafetyMonitor` exposes a soft current limit. Treat it as the
  upper bound, not as a target.

## 10. Roadmap Toward The Balancing Robot

1. **Bring up.** Confirm hub firmware, run `HubMain.py`, see IMU values print.
2. **Sensor sanity.** Record IMU and encoder traces with `HubImuTest.py` and
   `HubEncoderTest.py`. Verify signs and units against `docs/HardwareAssumptions.md`.
3. **Sign verification (done).** The IMU sign, encoder signs, forward sign, zero offset and
   gyro bias have been measured and committed to `configs/Default.yaml`. Treat these as
   correct from this point forward.
4. **Pre balancing phase (current).** A light `StateEstimator` produces the implemented
   state `[theta, thetaDot, phi, phiDot]`. A `DriveCommandController` issues forward,
   backward and stop commands at the hardware-validated smoke-test magnitude through the
   same estimator + safety pipeline the future balancing controller will use. See section 14 below.
5. **Open loop motor characterization.** Use `HubMotorTest.py` to map duty to angular
   velocity. Capture data with `DataLogger`.
6. **Outer loop balancing controller.** Replace the body of `LyapunovController.Compute`
   with a real Lyapunov based control law. See `docs/FutureControlRoadmap.md`.
7. **On hub deployment.** Prefer a single self contained Pybricks script when you want
   the most reliable upload path. Use the package-backed smoke path when you explicitly
   want to test a hub-safe `LegoBalance` module on the real hub.

## 11. How To Add A New Controller, Sensor, Or Estimator

- **New controller.** Subclass `ControllerBase` and implement `Compute(state) -> ControlOutput`.
  Register it from your application code. The simulation stub
  (`examples/ClosedLoopSimulation.py`) shows how to swap controllers in one line.
- **New sensor.** Add a new abstract interface under `src/LegoBalance/` mirroring
  `ImuInterface`. Provide a mock implementation for tests and a hub side adapter for the
  Pybricks API.
- **New estimator.** Subclass nothing, just provide an `Update(measurements, dt)` method
  that returns a `BalanceState`. The simulation stub treats the estimator as a plain
  callable so any object with the right method signature works.

## 12. Important Honest Caveats

- This release does not include any tested control law for balancing. The
  `LyapunovController` is a documented placeholder.
- The hub side scripts under `hub/` are minimal Pybricks programs. The package-backed
  smoke entrypoint lives under `src/` only so `pybricksdev` can find and upload the
  `LegoBalance` runtime module beside it.
- The IMU sign and axis convention on the SPIKE Prime hub depends on how the hub is
  mounted on the chassis. The exact mapping you need is described in
  `docs/HardwareAssumptions.md` and you must verify it on your specific build.

## 13. License

MIT. See the `pyproject.toml` for the formal entry. Use freely for education and research.

## 14. Pre Balancing Phase: state convention and smoke flows

The repository sits between sign verification and balancing. The hardware sign convention
has been verified manually and the exact wheel radius has not. To stay close to the raw
sensors and to avoid silently depending on a calibration value that is not finalized yet,
the implemented state vector is

```
x = [theta, thetaDot, phi, phiDot]
```

where:

- `theta` (`tilt`) is the body tilt angle in radians; `0` means upright.
- `thetaDot` (`tiltRate`) is the body tilt rate in radians per second.
- `phi` is the **mean wheel rotation angle in radians**, taken as the average of the two
  sign corrected wheel encoder angles. `phi` does **not** depend on the wheel radius.
- `phiDot` is the mean wheel rotation rate in radians per second.

The translation pair `p` (linear distance, meters) and `pDot` (linear velocity, m/s) is a
**secondary derived view**. It is computed from `phi` only when a wheel radius is
available, via:

```
p     = r * phi
pDot  = r * phiDot
```

These are exposed as `BalanceState.LinearPosition(wheelRadius)` /
`BalanceState.LinearVelocity(wheelRadius)` and as `StateEstimator.LinearPosition(state)` /
`StateEstimator.LinearVelocity(state)`. They are intentionally not stored on the state
object so that the core estimator stays radius free.

### Why phi and phiDot are the implemented state at this stage

- They come most directly and safely from the SPIKE Prime motor encoders.
- They keep the estimator close to the raw measured quantities.
- They avoid an early dependence on a wheel radius that may not be finalized.
- They reduce avoidable modeling assumptions before the balancing phase begins.

The future Lyapunov controller can still consume this state. It can either use `phi`
directly when shaping wheel rotation, or convert to `p` via the wheel radius when it
needs a linear translation cost.

### Running the desktop side smoke flows

```bash
# Print the implemented [theta, thetaDot, phi, phiDot] state at every step.
python examples/EstimatorReadout.py

# Walk through the forward / stop / backward / stop schedule and write a CSV log.
python examples/DriveCommandSmoke.py
```

Both scripts use `MockHub` so they need no hardware. They wire the same package
abstractions a real run uses (`StateEstimator`, `DriveCommandController`, `SafetyMonitor`,
`DataLogger`) so that the next step (real hardware) is mechanical.

### Running the hub side drive smoke flow

```bash
pybricksdev run ble hub/HubDriveSmoke.py
```

To run the same hub smoke flow and open a diagnostic plot of the estimator state after
the run finishes:

```bash
python scripts/PlotHubDriveSmoke.py
```

`hub/HubDriveSmoke.py` is a small self contained Pybricks program that mirrors the
desktop drive smoke flow. It applies the same sign convention as `configs/Default.yaml`,
computes `phi` and `phiDot` on the hub, refuses motion above the configured pre balancing
tilt limit, and walks through the same forward / stop / backward / stop schedule at a
hardware-validated wheel velocity. Its `DATA` rows are consumed by `scripts/PlotHubDriveSmoke.py`,
which plots `[theta, thetaDot, phi, phiDot]`, the issued wheel command, and the drive gate
status once the run ends.

To test the package-backed hub path instead of the standalone hub file:

```bash
python scripts/PlotHubPackageDriveSmoke.py
```

That plotter runs `src/HubPackageDriveSmoke.py`, which imports
`LegoBalance.StateEstimator`, `LegoBalance.DriveCommandController`, and
`LegoBalance.SafetyMonitor` on the hub. `LegoBalance.HubDriveSmokeRuntime` only supplies a
MicroPython-safe default config, kept aligned with the desktop config by tests.

> **Safety:** the drive smoke flow commands wheel motion. **Block the wheels or hold the
> robot in your hand** the first time you run it. The default magnitude has been validated
> on the real build, but the robot will still try to roll. The Lyapunov balancing
> controller is not yet implemented, so the robot is **not** balancing while this script runs.

### What is intentionally NOT implemented yet

- A complementary filter or Kalman filter inside `StateEstimator`. The hardware sensors
  are accurate enough at this stage; a future revision will drop a filter into the same
  `Update` method without changing the public interface.
- A balancing controller. `LyapunovController` is still a documented placeholder and
  `LyapunovController.IsPlaceholder()` still returns `True`.
- Online estimation of the upright zero offset or gyro bias. They are loaded from the
  config and assumed correct.
