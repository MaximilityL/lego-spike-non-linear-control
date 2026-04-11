"""Plot a buffered hub balance run.

Laptop-side companion for ``src/HubPackageBalanceBuffered.py``. The hub
script keeps every 100 Hz control iteration in preallocated float32
buffers during the run, then dumps all samples over BLE once the run
finishes. This script regenerates the hub runtime from
``configs/Default.yaml``, launches the buffered hub program through
``pybricksdev``, collects the dumped telemetry rows, and produces a post
run diagnostic plot identical in layout to ``PlotHubPackageBalance.py``.

Usage:

    python scripts/PlotHubPackageBalanceBuffered.py
    python scripts/PlotHubPackageBalanceBuffered.py --log path/to/run.txt
    pybricksdev run ble src/HubPackageBalanceBuffered.py \\
        | python scripts/PlotHubPackageBalanceBuffered.py --stdin
"""

from __future__ import annotations

import sys
from pathlib import Path

if getattr(getattr(sys, "implementation", None), "name", "") == "micropython":
    raise SystemExit(
        "PlotHubPackageBalanceBuffered.py runs on your laptop with normal "
        "Python, not on the SPIKE hub. Run: "
        "python scripts/PlotHubPackageBalanceBuffered.py"
    )

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
SRC_ROOT = REPO_ROOT / "src"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import PlotHubPackageBalance as base  # noqa: E402

HUB_SCRIPT = SRC_ROOT / "HubPackageBalanceBuffered.py"
DEFAULT_OUTPUT_PATH = REPO_ROOT / "plots" / "HubPackageBalanceBuffered.png"
RUN_NAME = "HubPackageBalanceBuffered"


def Main() -> int:
    from GenerateHubDriveSmokeRuntime import GenerateHubDriveSmokeRuntime
    from LegoBalance.RobotConfig import DEFAULT_CONFIG_PATH

    # Point the reused argparse defaults at the buffered hub script and
    # its own plot output so the shared CollectFromPybricksdev path does
    # the right thing without forking it.
    base.DEFAULT_HUB_SCRIPT = HUB_SCRIPT
    base.DEFAULT_OUTPUT_PATH = DEFAULT_OUTPUT_PATH

    args = base.ParseArgs()
    argStatus = base.ValidateArgs(args)
    if argStatus != 0:
        return argStatus

    if args.log is not None:
        try:
            samples = base.CollectFromLogFile(args.log)
        except FileNotFoundError as exc:
            print(str(exc), file=sys.stderr)
            return 1
    elif args.stdin:
        samples = base.CollectSamples(sys.stdin)
    else:
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
        try:
            samples = base.CollectFromPybricksdev(args)
        except FileNotFoundError:
            print(
                "Could not find pybricksdev. Install it with: pip install -e .[hub], "
                "or pipe a saved log via --log / --stdin.",
                file=sys.stderr,
            )
            return 1

    return base.Plot(
        samples,
        outputPath=args.output,
        showPlot=not args.no_show,
        runName=RUN_NAME,
    )


if __name__ == "__main__":
    raise SystemExit(Main())
