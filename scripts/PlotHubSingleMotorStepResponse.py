"""Plot a single-motor angle/speed step-response run.

This is the desktop-side companion to ``hub/HubSingleMotorStepResponseF.py``.
It launches the hub script through ``pybricksdev`` (or reads a saved log) and
plots reference versus measured angle and speed.

Usage:

    python scripts/PlotHubSingleMotorStepResponse.py
    python scripts/PlotHubSingleMotorStepResponse.py --log path/to/run.txt
    pybricksdev run ble hub/HubSingleMotorStepResponseF.py | python scripts/PlotHubSingleMotorStepResponse.py --stdin
"""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

if getattr(getattr(sys, "implementation", None), "name", "") == "micropython":
    raise SystemExit(
        "PlotHubSingleMotorStepResponse.py runs on your laptop with normal "
        "Python, not on the SPIKE hub. Run it on the desktop."
    )

REPO_ROOT = Path(__file__).resolve().parents[1]
MPLCONFIGDIR = Path("/tmp/matplotlib-legobalance")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

DEFAULT_HUB_SCRIPT = REPO_ROOT / "hub" / "HubSingleMotorStepResponseF.py"
DEFAULT_OUTPUT_PATH = REPO_ROOT / "plots" / "HubSingleMotorStepResponse.png"
DATA_PREFIX = "DATA,"


@dataclass(frozen=True)
class StepSample:
    phase: str
    segment: str
    t_s: float
    angle_ref_deg: float
    angle_meas_deg: float
    speed_ref_deg_per_sec: float
    speed_meas_deg_per_sec: float


def ParseStepResponseLine(line: str) -> StepSample | None:
    dataStart = line.find(DATA_PREFIX)
    if dataStart < 0:
        return None
    payload = line[dataStart + len(DATA_PREFIX) :]
    fields = next(csv.reader([payload]))
    if len(fields) < 7:
        return None
    try:
        return StepSample(
            phase=fields[0],
            segment=fields[1],
            t_s=float(fields[2]),
            angle_ref_deg=float(fields[3]),
            angle_meas_deg=float(fields[4]),
            speed_ref_deg_per_sec=float(fields[5]),
            speed_meas_deg_per_sec=float(fields[6]),
        )
    except ValueError:
        return None


def CollectSamples(stream: TextIO) -> list[StepSample]:
    samples: list[StepSample] = []
    for rawLine in stream:
        line = rawLine.rstrip("\r\n")
        sample = ParseStepResponseLine(line)
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


def CollectFromPybricksdev(args: argparse.Namespace) -> list[StepSample]:
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


def CollectFromLogFile(path: Path) -> list[StepSample]:
    if not path.exists():
        raise FileNotFoundError(f"log file not found: {path}")
    with path.open("r", encoding="utf-8") as fh:
        return CollectSamples(fh)


def ContiguousPhaseSpans(samples: list[StepSample]) -> list[tuple[str, float, float]]:
    spans: list[tuple[str, float, float]] = []
    if not samples:
        return spans

    currentPhase = samples[0].phase
    spanStart = samples[0].t_s
    previousTime = samples[0].t_s
    for sample in samples[1:]:
        if sample.phase != currentPhase:
            spans.append((currentPhase, spanStart, previousTime))
            currentPhase = sample.phase
            spanStart = sample.t_s
        previousTime = sample.t_s
    spans.append((currentPhase, spanStart, previousTime))
    return spans


def SegmentBoundaries(samples: list[StepSample]) -> list[tuple[str, str, float]]:
    boundaries: list[tuple[str, str, float]] = []
    previousKey: tuple[str, str] | None = None
    for sample in samples:
        currentKey = (sample.phase, sample.segment)
        if currentKey != previousKey:
            boundaries.append((sample.phase, sample.segment, sample.t_s))
            previousKey = currentKey
    return boundaries


def Plot(
    samples: list[StepSample],
    outputPath: Path,
    showPlot: bool,
    runName: str = "HubSingleMotorStepResponse",
) -> int:
    if not samples:
        print(
            "No DATA rows were collected. Nothing to plot. Confirm the hub "
            "script ran and printed DATA rows.",
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
    angleRef = [s.angle_ref_deg for s in samples]
    angleMeas = [s.angle_meas_deg for s in samples]
    speedRef = [s.speed_ref_deg_per_sec for s in samples]
    speedMeas = [s.speed_meas_deg_per_sec for s in samples]

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(12, 8), num=runName)
    fig.suptitle(
        runName + ": one-motor step response\nreference versus measured angle and speed"
    )
    axAngle, axSpeed = axes

    phaseColors = {
        "velocity": "tab:blue",
        "position": "tab:orange",
    }
    for phase, start, end in ContiguousPhaseSpans(samples):
        color = phaseColors.get(phase, "0.8")
        axAngle.axvspan(start, end, color=color, alpha=0.05)
        axSpeed.axvspan(start, end, color=color, alpha=0.05)

    axAngle.plot(times, angleRef, color="tab:red", linestyle="--", linewidth=1.3, label="angle ref")
    axAngle.plot(times, angleMeas, color="tab:blue", linewidth=1.6, label="angle meas")
    axAngle.set_ylabel("angle (deg)")
    axAngle.grid(True, alpha=0.35)
    axAngle.legend(loc="upper right")

    axSpeed.plot(
        times,
        speedRef,
        color="tab:purple",
        linestyle="--",
        linewidth=1.3,
        label="speed ref",
    )
    axSpeed.plot(times, speedMeas, color="tab:green", linewidth=1.6, label="speed meas")
    axSpeed.set_ylabel("speed (deg/s)")
    axSpeed.set_xlabel("time (s)")
    axSpeed.grid(True, alpha=0.35)
    axSpeed.legend(loc="upper right")

    for phase, segment, timeSec in SegmentBoundaries(samples):
        label = phase + ":" + segment
        axAngle.axvline(timeSec, color="0.7", linestyle=":", linewidth=0.8)
        axSpeed.axvline(timeSec, color="0.7", linestyle=":", linewidth=0.8)
        axSpeed.text(
            timeSec,
            0.98,
            label,
            rotation=90,
            va="top",
            ha="right",
            transform=axSpeed.get_xaxis_transform(),
            fontsize=9,
            color="0.35",
        )

    outputPath.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.subplots_adjust(top=0.90)
    fig.savefig(outputPath, dpi=150)
    print(f"Saved plot to {outputPath}", file=sys.stderr)

    if showPlot:
        plt.show()
    else:
        plt.close(fig)
    return 0


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read hub output from stdin instead of launching pybricksdev",
    )
    parser.add_argument(
        "--log",
        type=Path,
        help="read hub output from a saved file",
    )
    parser.add_argument(
        "--hub-script",
        type=Path,
        default=DEFAULT_HUB_SCRIPT,
        help="HubSingleMotorStepResponseF.py path passed to pybricksdev",
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
            "PlotHubSingleMotorStepResponse.py is the laptop-side plotter, "
            "not the hub program. Use:\n\n"
            "    python scripts/PlotHubSingleMotorStepResponse.py\n\n"
            "or run the hub program directly with:\n\n"
            "    pybricksdev run ble hub/HubSingleMotorStepResponseF.py",
            file=sys.stderr,
        )
        return 1
    return 0


def Main() -> int:
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
            samples = CollectFromPybricksdev(args)
        except FileNotFoundError:
            print(
                "Could not find pybricksdev. Install it with: pip install -e .[hub], "
                "or pipe a saved log via --log / --stdin.",
                file=sys.stderr,
            )
            return 1

    return Plot(
        samples,
        outputPath=args.output,
        showPlot=not args.no_show,
    )


if __name__ == "__main__":
    raise SystemExit(Main())
