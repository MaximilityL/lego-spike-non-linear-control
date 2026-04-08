"""Plot a real package-backed hub balance run.

This is the laptop-side companion for ``src/HubPackageBalance.py``. It runs
the real balancing controller on the SPIKE hub, captures its telemetry rows,
and renders a post-run diagnostic plot showing:

- tilt reference versus measured tilt,
- the full implemented state ``[theta, thetaDot, phi, phiDot]``,
- raw versus applied wheel velocity command.

Usage:

    python scripts/PlotHubPackageBalance.py
    python scripts/PlotHubPackageBalance.py --log path/to/run.txt
    pybricksdev run ble src/HubPackageBalance.py | python scripts/PlotHubPackageBalance.py --stdin
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

if getattr(getattr(sys, "implementation", None), "name", "") == "micropython":
    raise SystemExit(
        "PlotHubPackageBalance.py runs on your laptop with normal Python, not on "
        "the SPIKE hub. Run: python scripts/PlotHubPackageBalance.py"
    )

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))
MPLCONFIGDIR = Path("/tmp/matplotlib-legobalance")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

DEFAULT_HUB_SCRIPT = SRC_ROOT / "HubPackageBalance.py"
DEFAULT_OUTPUT_PATH = REPO_ROOT / "plots" / "HubPackageBalance.png"
DATA_PREFIX = "DATA,"


@dataclass(frozen=True)
class BalanceSample:
    t_s: float
    theta_ref_deg: float
    theta_deg: float
    theta_dot_deg_per_sec: float
    phi_deg: float
    phi_dot_deg_per_sec: float
    raw_cmd_deg_per_sec: float
    safe_cmd_deg_per_sec: float
    status: str

    @property
    def tripped(self) -> bool:
        return self.status.upper() == "TRIPPED"


def ParseBalanceLine(line: str) -> BalanceSample | None:
    dataStart = line.find(DATA_PREFIX)
    if dataStart < 0:
        return None
    payload = line[dataStart + len(DATA_PREFIX) :]
    fields = next(csv.reader([payload]))
    if len(fields) < 9:
        return None
    try:
        return BalanceSample(
            t_s=float(fields[0]),
            theta_ref_deg=float(fields[1]),
            theta_deg=float(fields[2]),
            theta_dot_deg_per_sec=float(fields[3]),
            phi_deg=float(fields[4]),
            phi_dot_deg_per_sec=float(fields[5]),
            raw_cmd_deg_per_sec=float(fields[6]),
            safe_cmd_deg_per_sec=float(fields[7]),
            status=fields[8],
        )
    except ValueError:
        return None


def CollectSamples(stream: TextIO) -> list[BalanceSample]:
    samples: list[BalanceSample] = []
    for rawLine in stream:
        line = rawLine.rstrip("\r\n")
        sample = ParseBalanceLine(line)
        if sample is None:
            if line:
                print(line, file=sys.stderr)
            continue
        samples.append(sample)
    return samples


def BuildPybricksCommand(args: argparse.Namespace) -> list[str]:
    command = ["pybricksdev", "run", "ble"]
    if args.name:
        command.extend(["--name", args.name])
    command.append(str(args.hub_script))
    return command


def CollectFromPybricksdev(args: argparse.Namespace) -> list[BalanceSample]:
    import subprocess

    command = BuildPybricksCommand(args)
    print("Starting:", " ".join(command), file=sys.stderr)
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    process = subprocess.Popen(
        command,
        cwd=REPO_ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    if process.stdout is None:
        raise RuntimeError("pybricksdev stdout pipe was not created")
    try:
        samples = CollectSamples(process.stdout)
    finally:
        process.wait()
    return samples


def CollectFromLogFile(path: Path) -> list[BalanceSample]:
    if not path.exists():
        raise FileNotFoundError(f"log file not found: {path}")
    with path.open("r", encoding="utf-8") as fh:
        return CollectSamples(fh)


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read HubPackageBalance output from stdin instead of launching pybricksdev",
    )
    parser.add_argument(
        "--log",
        type=Path,
        help="read HubPackageBalance output from a saved file",
    )
    parser.add_argument(
        "--hub-script",
        type=Path,
        default=DEFAULT_HUB_SCRIPT,
        help="HubPackageBalance.py path passed to pybricksdev",
    )
    parser.add_argument(
        "--name",
        help='optional hub name for pybricksdev, for example "Pybricks Hub"',
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_PATH,
        help="path to save the generated PNG plot",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="save the plot without opening a matplotlib window",
    )
    return parser.parse_args()


def ValidateArgs(args: argparse.Namespace) -> int:
    if Path(args.hub_script).resolve() == Path(__file__).resolve():
        print(
            "PlotHubPackageBalance.py is the laptop-side plotter, not the hub "
            "program. Use:\n\n"
            "    python scripts/PlotHubPackageBalance.py\n\n"
            "or run the package-backed hub program directly with:\n\n"
            "    pybricksdev run ble src/HubPackageBalance.py",
            file=sys.stderr,
        )
        return 1
    return 0


def Plot(
    samples: list[BalanceSample],
    outputPath: Path,
    showPlot: bool,
    runName: str = "HubPackageBalance",
) -> int:
    if not samples:
        print(
            "No DATA rows were collected. Nothing to plot. Confirm " + runName + " "
            "ran and printed DATA rows.",
            file=sys.stderr,
        )
        return 1

    try:
        import matplotlib

        if not showPlot:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            "matplotlib is required for plotting. Install it with: pip install matplotlib",
            file=sys.stderr,
        )
        return 1

    times = [s.t_s for s in samples]
    thetaRef = [s.theta_ref_deg for s in samples]
    theta = [s.theta_deg for s in samples]
    thetaDot = [s.theta_dot_deg_per_sec for s in samples]
    phi = [s.phi_deg for s in samples]
    phiDot = [s.phi_dot_deg_per_sec for s in samples]
    rawCmd = [s.raw_cmd_deg_per_sec for s in samples]
    safeCmd = [s.safe_cmd_deg_per_sec for s in samples]

    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(12, 10), num=runName + " run")
    fig.suptitle(
        runName + ": real hub balance telemetry\n"
        "implemented state [theta, thetaDot, phi, phiDot]",
        fontsize=16,
    )

    axTheta, axThetaDot, axPhi, axPhiDot, axCmd = axes

    axTheta.plot(times, theta, color="tab:blue", linewidth=1.6, label="theta")
    axTheta.plot(times, thetaRef, color="tab:red", linestyle="--", linewidth=1.1, label="theta ref")
    axTheta.set_ylabel("theta (deg)")
    axTheta.grid(True, alpha=0.35)
    axTheta.legend(loc="upper right")

    axThetaDot.plot(times, thetaDot, color="tab:orange", linewidth=1.4, label="thetaDot")
    axThetaDot.set_ylabel("thetaDot (deg/s)")
    axThetaDot.grid(True, alpha=0.35)
    axThetaDot.legend(loc="upper right")

    axPhi.plot(times, phi, color="tab:green", linewidth=1.6, label="phi")
    axPhi.set_ylabel("phi (deg)")
    axPhi.grid(True, alpha=0.35)
    axPhi.legend(loc="upper right")

    axPhiDot.plot(times, phiDot, color="tab:purple", linewidth=1.4, label="phiDot")
    axPhiDot.set_ylabel("phiDot (deg/s)")
    axPhiDot.grid(True, alpha=0.35)
    axPhiDot.legend(loc="upper right")

    axCmd.plot(times, rawCmd, color="tab:brown", linewidth=1.4, label="raw cmd")
    axCmd.plot(times, safeCmd, color="tab:cyan", linestyle="--", linewidth=1.4, label="applied cmd")
    axCmd.set_ylabel("wheel cmd (deg/s)")
    axCmd.set_xlabel("time (s)")
    axCmd.grid(True, alpha=0.35)
    axCmd.legend(loc="upper right")

    for sample in samples:
        if sample.tripped:
            for ax in axes:
                ax.axvline(sample.t_s, color="tab:red", alpha=0.18, linewidth=0.9)

    durationSec = times[-1] - times[0] if len(times) >= 2 else 0.0
    fig.text(
        0.01,
        0.005,
        "  ".join(
            (
                "samples=" + str(len(samples)),
                "duration=" + "{:.2f}".format(durationSec) + "s",
                "final theta error=" + "{:+.2f}".format(theta[-1] - thetaRef[-1]) + " deg",
                "final status=" + samples[-1].status,
            )
        ),
        fontsize=10,
        color="dimgray",
    )

    fig.tight_layout(rect=(0.0, 0.02, 1.0, 0.96))
    outputPath = Path(outputPath)
    outputPath.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(outputPath, dpi=160)
    print(f"plot written to  : {outputPath}")
    if showPlot:
        plt.show()
    else:
        plt.close(fig)
    return 0


def Main() -> int:
    from GenerateHubDriveSmokeRuntime import GenerateHubDriveSmokeRuntime
    from LegoBalance.RobotConfig import DEFAULT_CONFIG_PATH

    args = ParseArgs()
    argStatus = ValidateArgs(args)
    if argStatus != 0:
        return argStatus

    if args.log is not None:
        try:
            samples = CollectFromLogFile(args.log)
        except FileNotFoundError as exc:
            print(str(exc), file=sys.stderr)
            return 1
    elif args.stdin:
        samples = CollectSamples(sys.stdin)
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
            samples = CollectFromPybricksdev(args)
        except FileNotFoundError:
            print(
                "Could not find pybricksdev. Install it with: pip install -e .[hub], "
                "or pipe a saved log via --log / --stdin.",
                file=sys.stderr,
            )
            return 1

    return Plot(samples, outputPath=args.output, showPlot=not args.no_show)


if __name__ == "__main__":
    raise SystemExit(Main())
