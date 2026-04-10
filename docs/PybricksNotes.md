# Pybricks Notes

Practical notes for running this project on a LEGO SPIKE Prime hub with
Pybricks.

## 1. What Pybricks Is

Pybricks replaces the stock LEGO firmware with an open MicroPython runtime. The
hub then runs Python programs that import from `pybricks.*`.

For this project, Pybricks matters because it lets the control loop execute on
the hub itself instead of depending on a host computer in the loop.

## 2. Hub Program Styles In This Repo

This repository uses two hub-program styles.

### Self-contained scripts

Files under `hub/` are short Pybricks programs for:

- sensor bring-up,
- encoder checks,
- motor checks,
- simple smoke tests.

These are easy to upload and debug because they avoid package-import complexity.

### Package-backed scripts

The files:

- `src/HubPackageDriveSmoke.py`
- `src/HubPackageBalance.py`

import the shared `LegoBalance` estimator/controller/safety logic on the hub.
They are useful because they exercise the real shared code path rather than a
manually copied standalone script.

Those entrypoints rely on the generated hub-safe config module
`LegoBalance.HubDriveSmokeRuntime`, since the hub does not parse YAML directly.

## 3. Useful Pybricks Imports

```python
from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait
```

## 4. Reading Sensors

Typical sensor access on the hub looks like:

```python
hub = PrimeHub()
pitchDeg, rollDeg = hub.imu.tilt()
gxDegPerSec, gyDegPerSec, gzDegPerSec = hub.imu.angular_velocity()
leftAngleDeg = leftMotor.angle()
leftRateDegPerSec = leftMotor.speed()
```

The shared estimator expects SI units, so hub entrypoints convert degrees to
radians before building a `Measurement`.

## 5. Driving Motors

The default control path uses wheel velocity commands:

```python
motor.run(speed_deg_per_s)
```

This is why the balance controllers return commands in rad/s. The hub-side code
handles conversion to deg/s just before actuation.

`Motor.dc(...)` is still available for future experiments, but it is a weaker
match to the current controller design because it behaves more like an open-loop
duty command than a clean velocity interface.

## 6. Loop Timing

Pybricks is not a hard real-time environment, so hub loops approximate a fixed
rate using `StopWatch` and `wait(...)`.

For this project, around 100 Hz is the main operating point for the outer loop.
That is fast enough for balance experiments while still leaving room for modest
telemetry and safety checks.

## 7. Hub-Side Constraints

When writing code that should run through the shared package path on the hub:

- avoid `numpy`,
- avoid YAML parsing,
- avoid heavy object allocation inside the loop,
- keep logging light,
- keep math scalar and MicroPython-safe.

`NonLinearController` follows those rules so it can run inside
`src/HubPackageBalance.py`.

## 8. Practical Deployment

Browser-based run:

1. Open [code.pybricks.com](https://code.pybricks.com).
2. Connect to the hub.
3. Open or paste a hub script.
4. Click run.

Command-line run:

```bash
pip install -e .[hub]
pybricksdev run ble hub/HubMain.py
pybricksdev run ble hub/HubDriveSmoke.py
pybricksdev run ble src/HubPackageBalance.py
```

For package-backed runs, regenerate the hub-safe config first:

```bash
python scripts/GenerateHubDriveSmokeRuntime.py
```

## 9. Good Habits During Balance Experiments

- keep a hand near the robot during first tests,
- start with small motion and short runs,
- print only sparse telemetry,
- use the hub center button as the emergency stop,
- record the exact gains used for each experiment.
