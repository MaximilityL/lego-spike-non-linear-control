"""Launch the zeroOffset/gyroBias calibration sweep on the SPIKE hub.

Desktop companion for ``src/HubPackageBalanceCalibrationSweep.py``. This
script only regenerates the hub-safe runtime from ``configs/Default.yaml``
and then launches the hub program through ``pybricksdev``. Unlike the
balance plotters it does not collect telemetry rows or render a plot,
because the sweep itself only prints short per-test summaries and a final
best-score ranking back over BLE. Everything printed by the hub script is
streamed straight to the terminal so you can watch the sweep live.

Usage:

    python scripts/RunHubPackageBalanceCalibrationSweep.py
    python scripts/RunHubPackageBalanceCalibrationSweep.py --name "Pybricks Hub"
    python scripts/RunHubPackageBalanceCalibrationSweep.py --log out.txt
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

if getattr(getattr(sys, "implementation", None), "name", "") == "micropython":
    raise SystemExit(
        "RunHubPackageBalanceCalibrationSweep.py runs on your laptop with "
        "normal Python, not on the SPIKE hub. Run: "
        "python scripts/RunHubPackageBalanceCalibrationSweep.py"
    )

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
SRC_ROOT = REPO_ROOT / "src"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_HUB_SCRIPT = SRC_ROOT / "HubPackageBalanceCalibrationSweep.py"


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hub-script",
        type=Path,
        default=DEFAULT_HUB_SCRIPT,
        help="HubPackageBalanceCalibrationSweep.py path passed to pybricksdev",
    )
    parser.add_argument(
        "--name",
        help='optional hub name for pybricksdev, for example "Pybricks Hub"',
    )
    parser.add_argument(
        "--log",
        type=Path,
        help="tee the hub output to this file as well as the terminal",
    )
    return parser.parse_args()


def BuildPybricksCommand(args: argparse.Namespace) -> list[str]:
    command = ["pybricksdev", "run", "ble"]
    if args.name:
        command.extend(["--name", args.name])
    command.append(str(args.hub_script))
    return command


def Main() -> int:
    from GenerateHubDriveSmokeRuntime import GenerateHubDriveSmokeRuntime
    from LegoBalance.RobotConfig import DEFAULT_CONFIG_PATH

    args = ParseArgs()

    try:
        generatedPath = GenerateHubDriveSmokeRuntime(DEFAULT_CONFIG_PATH)
    except (FileNotFoundError, ValueError) as exc:
        print(f"Hub config generation failed: {exc}", file=sys.stderr)
        return 1
    print(
        f"Generated {generatedPath.relative_to(REPO_ROOT)} from "
        f"{DEFAULT_CONFIG_PATH.relative_to(REPO_ROOT)}",
        file=sys.stderr,
    )

    command = BuildPybricksCommand(args)
    print("Starting:", " ".join(command), file=sys.stderr)
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"

    try:
        process = subprocess.Popen(
            command,
            cwd=REPO_ROOT,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError:
        print(
            "Could not find pybricksdev. Install it with: pip install -e .[hub]",
            file=sys.stderr,
        )
        return 1

    if process.stdout is None:
        raise RuntimeError("pybricksdev stdout pipe was not created")

    logHandle = None
    if args.log is not None:
        args.log.parent.mkdir(parents=True, exist_ok=True)
        logHandle = args.log.open("w", encoding="utf-8")

    try:
        for rawLine in process.stdout:
            line = rawLine.rstrip("\r\n")
            print(line)
            if logHandle is not None:
                logHandle.write(line + "\n")
                logHandle.flush()
    finally:
        if logHandle is not None:
            logHandle.close()
        process.wait()

    return process.returncode or 0


if __name__ == "__main__":
    raise SystemExit(Main())
