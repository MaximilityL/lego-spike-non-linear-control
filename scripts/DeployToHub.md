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
```

If discovery fails, give it the hub name explicitly:

```bash
pybricksdev run ble --name "Pybricks Hub" hub/HubMain.py
```

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

Pybricks programs must only import from `pybricks.*`. If you accidentally
import from `LegoBalance` or any other desktop package, the program will
fail to upload. The hub side scripts under `hub/` are written so that this
cannot happen.

## 3. Why The Hub Side Scripts Are Self Contained

Pybricks programs cannot freely import from arbitrary places on your
laptop. They are uploaded one file at a time. Multi file uploads exist in
recent Pybricks versions but they are not as smooth as a single file. The
clean rule for this project is "one self contained file per hub program".

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
