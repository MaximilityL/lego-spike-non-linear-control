# Pybricks Notes

Practical notes about using Pybricks for this project. This is not a Pybricks tutorial.
For that, read the official documentation at
[docs.pybricks.com](https://docs.pybricks.com).

## 1. What Pybricks Is

Pybricks replaces the LEGO firmware on the SPIKE Prime hub with an open source
MicroPython runtime. After flashing, the hub runs Python programs written against the
`pybricks` API. The original firmware can be restored at any time from the same
installer page.

## 2. Where Pybricks Programs Live

A Pybricks program is usually a single Python file uploaded to the hub. It cannot import
from arbitrary places in your filesystem the way a CPython program can. Multi file
projects are technically possible in recent Pybricks versions, but the simple, reliable,
beginner friendly path is "one file per program".

This is why this repo has a `hub/` directory holding short, self contained scripts. The
normal desktop side `LegoBalance` package is not uploaded to the hub. The one deliberate
exception is `src/HubPackageDriveSmoke.py`, which imports the shared
`LegoBalance.StateEstimator`, `DriveCommandController`, and `SafetyMonitor` modules so
package logic can be tested on hardware. Its hub-safe config helper is generated from
`configs/Default.yaml` by `scripts/GenerateHubDriveSmokeRuntime.py` before upload.

If you eventually want `NonLinearController` to run through the same shared package path,
keep that module MicroPython-safe too.

## 3. Useful Imports

```python
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop, Color, Button
from pybricks.tools import wait, StopWatch
```

## 4. Reading The IMU

```python
hub = PrimeHub()
pitch_deg, roll_deg = hub.imu.tilt()
ax, ay, az = hub.imu.acceleration()       # mm/s^2
gx, gy, gz = hub.imu.angular_velocity()   # deg/s
```

The exact axis to use for your robot's tilt depends on how the hub is mounted. The
`configs/Default.yaml` `imu.tiltAxis` field is the place to record your choice.

## 5. Driving Motors

```python
motor = Motor(Port.A)
motor.run(360)         # 360 deg/s = 1 rev/s
motor.dc(20)           # 20% duty cycle
motor.stop()           # release
motor.brake()          # short the windings
angle_deg = motor.angle()
speed_dps = motor.speed()
```

There is no torque mode in Pybricks today. The closest you have to a torque command is
`Motor.dc`, which is open loop duty. For a balancing controller you can either:

- Use `Motor.run` and treat the motor as a velocity actuator with an inner Pybricks
  loop. This is what the scaffold defaults to.
- Use `Motor.dc` and accept that the relationship to torque is approximate. This needs
  characterization for accurate results.

## 6. Loop Timing

Pybricks does not provide a guaranteed real time scheduler. Use `StopWatch` and `wait`
to approximate a fixed loop rate:

```python
sw = StopWatch()
period_ms = 10
next_tick = period_ms
while True:
    # ... read sensors, compute control, command motors ...
    while sw.time() < next_tick:
        wait(1)
    next_tick += period_ms
```

100 Hz is realistic on the SPIKE Prime hub for a control loop with a small amount of
arithmetic. 200 Hz is achievable but leaves less headroom for logging or printing.

## 7. Limits And Gotchas

- **No `numpy`.** MicroPython does not ship numpy. Use plain `math` and small Python
  lists for hub side code. Keep matrices tiny (4x4 is fine).
- **No `dataclasses`.** Use plain classes or tuples on the hub side.
- **No YAML parsing.** Hub-side config must be hard coded or generated ahead of time.
- **No filesystem.** Logging on the hub goes to the Pybricks Code terminal. Capture it
  there or stream values over Bluetooth from the desktop side.
- **`print` is slow.** Excessive prints will choke a 100 Hz loop. Print every Nth
  iteration or only on events.
- **Hub button interrupts.** Pressing the center button on the hub stops the program.
  This is your hardware kill switch.

## 8. Deploying

Two paths.

### Browser

1. Open [code.pybricks.com](https://code.pybricks.com).
2. Connect to the hub over Bluetooth.
3. Open the file from disk or paste the code.
4. Click run.

### Command Line

```bash
pip install -e .[hub]
pybricksdev run ble hub/HubMain.py
```

If discovery fails:

```bash
pybricksdev run ble --name "Pybricks Hub" hub/HubMain.py
```

On Linux you may need Bluetooth permissions. Adding your user to the `bluetooth` group
usually does it.

For the package-backed smoke path:

```bash
python scripts/GenerateHubDriveSmokeRuntime.py
pybricksdev run ble src/HubPackageDriveSmoke.py
```
