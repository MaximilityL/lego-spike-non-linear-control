# Deploying To The Hub

This page is a practical reference for getting code from this repo onto a
LEGO SPIKE Prime hub running Pybricks.

## 1. Two Supported Paths

### 1.1. Browser (recommended for first time users)

1. Open [code.pybricks.com](https://code.pybricks.com) in Chrome or Edge.
2. Click the connect icon. Pick your hub from the Bluetooth picker.
3. Click "New file" or use the file menu to open one of the scripts under
   `hub/`. The browser editor accepts pasted text from your clipboard, so
   you can also just copy paste the file contents.
4. Click the run (play) button.

The browser editor handles uploading and running for you. There is no
filesystem on the hub that you need to manage manually.

### 1.2. pybricksdev from the desktop

Install the optional dependency once:

```bash
pip install -e .[hub]
```

Then run any hub script directly:

```bash
pybricksdev run ble hub/HubMain.py
pybricksdev run ble hub/HubBluetoothTest.py
pybricksdev run ble hub/HubImuTest.py
pybricksdev run ble hub/HubEncoderTest.py
pybricksdev run ble hub/HubMotorTest.py
pybricksdev run ble hub/HubDriveSmoke.py
pybricksdev run ble src/HubPackageDriveSmoke.py
```

If discovery fails, give it the hub name explicitly:

```bash
pybricksdev run ble --name "Pybricks Hub" hub/HubMain.py
```

To run `HubMain.py` and show the live telemetry plots on your laptop:

```bash
python scripts/PlotHubMainLive.py
```

This opens one live state-vector figure for
`x = [theta, theta_dot, p, p_dot]^T`. Add `--show-motors` if you also want
raw left/right motor encoder plots as a diagnostic view.

To run `HubDriveSmoke.py` and show a diagnostic plot after the forward/stop/backward
schedule finishes:

```bash
python scripts/PlotHubDriveSmoke.py
```

The plotter launches `pybricksdev`, collects the hub `DATA` rows during the run, and then
opens a static plot for `[theta, thetaDot, phi, phiDot]`, wheel command, and drive gate
status.

Do not run `pybricksdev run ble scripts/PlotHubDriveSmoke.py`; that file is the laptop
plotter. Upload/run `hub/HubDriveSmoke.py`, or let `python scripts/PlotHubDriveSmoke.py`
launch it for you.

To test the package-backed drive smoke path instead, run:

```bash
python scripts/PlotHubPackageDriveSmoke.py
```

This launches `src/HubPackageDriveSmoke.py`, which imports the hub-safe
`LegoBalance.HubDriveSmokeRuntime` package module. The entrypoint lives under `src/`
because `pybricksdev` resolves local package imports relative to the uploaded script.

You can rename your hub from the gear icon in `code.pybricks.com`.

## 2. Common Errors

### 2.1. "BleakError: Bluetooth device is turned off"

Turn on Bluetooth on your computer. On Linux check that the `bluetooth`
service is running:

```bash
systemctl status bluetooth
```

### 2.2. Permission denied when scanning

On Linux add your user to the `bluetooth` group:

```bash
sudo usermod -a -G bluetooth "$USER"
```

Log out and back in.

### 2.3. "Could not find hub"

- Make sure the hub is on and the green light is steady.
- Hold the Bluetooth button on the hub until the light blinks. This puts it
  in pairing mode.
- Try the browser editor. If the browser sees the hub but `pybricksdev`
  does not, the issue is in your local Bluetooth stack, not in the hub.

### 2.4. "ImportError" on the hub

Most Pybricks programs here should only import from `pybricks.*`. If you accidentally
import the normal desktop `LegoBalance` modules, `yaml`, `dataclasses`, or another
desktop package, the program can fail on the hub. The exception is
`src/HubPackageDriveSmoke.py`, which imports only the hub-safe
`LegoBalance.HubDriveSmokeRuntime` subset.

## 3. Why The Hub Side Scripts Are Self Contained

Pybricks programs cannot freely import from arbitrary places on your
laptop. Multi file uploads exist in recent Pybricks versions, but they are
resolved relative to the entrypoint file. The clean default rule for this
project is "one self contained file per hub program"; the package-backed
drive smoke test is the deliberate exception used to test hub-safe
`LegoBalance` logic on hardware.

When you finalize a controller on the desktop, the path is:

1. Develop and test the controller in `src/LegoBalance/`.
2. Validate it in `examples/ClosedLoopSimulation.py`.
3. Copy the function bodies into a new file under `hub/`, replacing the
   imports with the Pybricks equivalents.
4. Upload and run.

## 4. Stopping A Running Program

- **From the browser.** Click the stop button.
- **From the hub.** Press the center button. Every script under `hub/`
  checks for this and stops cleanly.
- **From `pybricksdev`.** Press Ctrl C in your terminal. The hub will stop
  on disconnect.
