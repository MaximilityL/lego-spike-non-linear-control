"""Plot the package-backed HubPackageDriveSmoke run.

This is the laptop-side companion for ``src/HubPackageDriveSmoke.py``. It is
deliberately tiny and reuses the robust parser/plotter from
``scripts/PlotHubDriveSmoke.py``, but it changes the default hub entrypoint to
the package-backed Pybricks script and loads the drive gate from
``LegoBalance`` on the desktop.

Usage:

    python scripts/PlotHubPackageDriveSmoke.py
    python scripts/PlotHubPackageDriveSmoke.py --log path/to/run.txt
    pybricksdev run ble src/HubPackageDriveSmoke.py | python scripts/PlotHubPackageDriveSmoke.py --stdin
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if getattr(getattr(sys, "implementation", None), "name", "") == "micropython":
    raise SystemExit(
        "PlotHubPackageDriveSmoke.py runs on your laptop with normal Python, not "
        "on the SPIKE hub. Run: python scripts/PlotHubPackageDriveSmoke.py"
    )

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_HUB_SCRIPT = SRC_ROOT / "HubPackageDriveSmoke.py"


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read HubPackageDriveSmoke output from stdin instead of launching pybricksdev",
    )
    parser.add_argument(
        "--log",
        type=Path,
        help="read HubPackageDriveSmoke output from a saved file",
    )
    parser.add_argument(
        "--hub-script",
        type=Path,
        default=DEFAULT_HUB_SCRIPT,
        help="HubPackageDriveSmoke.py path passed to pybricksdev",
    )
    parser.add_argument(
        "--name",
        help='optional hub name for pybricksdev, for example "Pybricks Hub"',
    )
    parser.add_argument(
        "--max-tilt-deg",
        type=float,
        help=(
            "drive gate envelope to draw; defaults to "
            "LegoBalance.LoadConfig().drive.maxTiltForMotion"
        ),
    )
    return parser.parse_args()


def ValidateArgs(args: argparse.Namespace) -> int:
    if Path(args.hub_script).resolve() == Path(__file__).resolve():
        print(
            "PlotHubPackageDriveSmoke.py is the laptop-side plotter, not the hub "
            "program. Use:\n\n"
            "    python scripts/PlotHubPackageDriveSmoke.py\n\n"
            "or run the package-backed hub program directly with:\n\n"
            "    pybricksdev run ble src/HubPackageDriveSmoke.py",
            file=sys.stderr,
        )
        return 1
    return 0


def Main() -> int:
    import PlotHubDriveSmoke as drive_plot

    from LegoBalance.RobotConfig import LoadConfig
    from LegoBalance.Units import RadToDeg

    args = ParseArgs()
    argStatus = ValidateArgs(args)
    if argStatus != 0:
        return argStatus

    if args.max_tilt_deg is None:
        config = LoadConfig()
        maxTiltDeg = RadToDeg(config.drive.maxTiltForMotion)
    else:
        maxTiltDeg = args.max_tilt_deg

    if args.log is not None:
        try:
            samples = drive_plot.CollectFromLogFile(args.log)
        except FileNotFoundError as exc:
            print(str(exc), file=sys.stderr)
            return 1
    elif args.stdin:
        samples = drive_plot.CollectSamples(sys.stdin)
    else:
        try:
            samples = drive_plot.CollectFromPybricksdev(args)
        except FileNotFoundError:
            print(
                "Could not find pybricksdev. Install it with: pip install -e .[hub], "
                "or pipe a saved log via --log / --stdin.",
                file=sys.stderr,
            )
            return 1

    return drive_plot.Plot(
        samples,
        maxTiltDeg=maxTiltDeg,
        runName="HubPackageDriveSmoke",
    )


if __name__ == "__main__":
    raise SystemExit(Main())
