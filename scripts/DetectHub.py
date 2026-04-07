"""DetectHub.py

Best effort presence sniff for the LEGO SPIKE Prime hub from the desktop.
This script does not import from ``pybricks`` because that package only
exists on the hub. Instead it tries the following, in order:

1. Look for any USB device that matches the LEGO vendor ID. This catches
   the case where the hub is plugged in over USB even though Pybricks does
   not normally use USB after firmware install.
2. If ``pybricksdev`` is installed, list reachable Bluetooth hubs.
3. Print a friendly summary either way.

Run with:

    python scripts/DetectHub.py
"""

from __future__ import annotations

import shutil
import subprocess
import sys

LEGO_USB_VENDOR_ID = "0694"


def DetectUsb() -> list[str]:
    """Return a list of USB device strings that look like LEGO devices.

    Uses ``lsusb`` if available. Returns an empty list on macOS or Windows.
    """
    lsusb = shutil.which("lsusb")
    if lsusb is None:
        return []
    try:
        completed = subprocess.run(
            [lsusb], capture_output=True, text=True, check=False, timeout=5
        )
    except Exception as exc:
        return [f"(lsusb call failed: {exc!r})"]
    matches = []
    for line in completed.stdout.splitlines():
        if LEGO_USB_VENDOR_ID in line.lower() or "lego" in line.lower():
            matches.append(line.strip())
    return matches


def DetectBleViaPybricksDev() -> list[str]:
    """Return a list of Bluetooth hubs reported by pybricksdev, if installed."""
    pybricksdev = shutil.which("pybricksdev")
    if pybricksdev is None:
        return []
    try:
        completed = subprocess.run(
            [pybricksdev, "ble", "scan"],
            capture_output=True,
            text=True,
            check=False,
            timeout=10,
        )
    except Exception as exc:
        return [f"(pybricksdev ble scan failed: {exc!r})"]
    if completed.returncode != 0:
        return [f"(pybricksdev exited with {completed.returncode})"]
    return [line.strip() for line in completed.stdout.splitlines() if line.strip()]


def Main() -> int:
    print("LegoBalance: SPIKE Prime hub presence sniff")
    print("-" * 60)

    usbHits = DetectUsb()
    if usbHits:
        print("USB devices that look LEGO related:")
        for line in usbHits:
            print(f"  {line}")
    else:
        print("No LEGO USB devices found via lsusb (this is normal if Pybricks is on Bluetooth).")

    print()
    bleHits = DetectBleViaPybricksDev()
    if bleHits:
        print("Bluetooth scan results from pybricksdev:")
        for line in bleHits:
            print(f"  {line}")
    else:
        print("pybricksdev not installed or returned no results.")
        print("Install with: pip install -e .[hub]")

    print()
    print("If you do not see your hub here, that does not necessarily mean it is broken.")
    print("Open code.pybricks.com in Chrome or Edge and try connecting from there too.")
    return 0


if __name__ == "__main__":
    sys.exit(Main())
