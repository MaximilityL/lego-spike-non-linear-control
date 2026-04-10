# Deploying To The Hub

This page is the practical reference for getting code from this repository onto
a LEGO SPIKE Prime hub running Pybricks.

## 1. Supported Deployment Paths

### 1.1. Browser workflow

1. Open [code.pybricks.com](https://code.pybricks.com) in Chrome or Edge.
2. Connect to the hub over Bluetooth.
3. Open one of the self-contained scripts under `hub/`, or paste it into a new file.
4. Click run.

This is the simplest path for first-time bring-up.

### 1.2. `pybricksdev` workflow

Install the optional dependency:

```bash
pip install -e .[hub]
```

Then run a hub script directly:

```bash
pybricksdev run ble hub/HubMain.py
pybricksdev run ble hub/HubImuTest.py
pybricksdev run ble hub/HubEncoderTest.py
pybricksdev run ble hub/HubMotorTest.py
pybricksdev run ble hub/HubSingleMotorStepResponseF.py
pybricksdev run ble hub/HubDriveSmoke.py
pybricksdev run ble src/HubPackageDriveSmoke.py
pybricksdev run ble src/HubPackageBalance.py
```

If discovery fails, pass the hub name explicitly:

```bash
pybricksdev run ble --name "Pybricks Hub" hub/HubMain.py
```

## 2. Common Workflows

### 2.1. Sensor bring-up with live plotting

```bash
pybricksdev run ble hub/HubMain.py
python scripts/PlotHubMainLive.py
```

### 2.2. Self-contained hub drive smoke

```bash
pybricksdev run ble hub/HubDriveSmoke.py
python scripts/PlotHubDriveSmoke.py
```

### 2.3. Single-motor step response on Port F

```bash
pybricksdev run ble hub/HubSingleMotorStepResponseF.py
python scripts/PlotHubSingleMotorStepResponse.py
```

### 2.4. Package-backed drive smoke

```bash
python scripts/GenerateHubDriveSmokeRuntime.py
python scripts/PlotHubPackageDriveSmoke.py
```

### 2.5. Package-backed real balance run

```bash
python scripts/GenerateHubDriveSmokeRuntime.py
python scripts/PlotHubPackageBalance.py
```

That script launches `src/HubPackageBalance.py`, captures its telemetry, and
generates a diagnostic plot automatically.

If you prefer to run the hub entrypoint directly:

```bash
python scripts/GenerateHubDriveSmokeRuntime.py
pybricksdev run ble src/HubPackageBalance.py
```

## 3. Why There Are Two Styles Of Hub Scripts

The repository keeps both:

- self-contained scripts under `hub/`, and
- package-backed scripts under `src/`.

The self-contained scripts are easy to upload and debug during bring-up.

The package-backed scripts are valuable because they run the shared
`LegoBalance` estimator/controller/safety logic on the real hub. That gives a
much more faithful end-to-end validation of the desktop-side code.

## 4. Package-Backed Config Generation

The hub cannot parse `configs/Default.yaml`, so the package-backed entrypoints
use a generated helper module:

```bash
python scripts/GenerateHubDriveSmokeRuntime.py
```

That command regenerates `src/LegoBalance/HubDriveSmokeRuntime.py`, which is the
hub-safe mirror of the desktop config for package-backed runs.

## 5. Common Errors

### 5.1. Hub not found

- make sure the hub is on,
- make sure Bluetooth is enabled,
- put the hub into pairing mode,
- try the explicit `--name "Pybricks Hub"` option.

### 5.2. Permission denied during Bluetooth scan

On Linux you may need Bluetooth permissions configured for the user account.

### 5.3. Import errors on the hub

Most hub scripts should import only from `pybricks.*`.

The deliberate exceptions are:

- `src/HubPackageDriveSmoke.py`
- `src/HubPackageBalance.py`

Those are designed to import a MicroPython-safe subset of `LegoBalance`.

## 6. Practical Safety Reminder

Balance experiments can move abruptly. During early runs:

- hold the robot or lift the wheels,
- keep the center button accessible,
- start from conservative gains and short runs.
