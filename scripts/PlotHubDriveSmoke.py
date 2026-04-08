"""Plot the estimator state captured during a HubDriveSmoke run.

The SPIKE hub cannot open matplotlib windows, so this is the desktop side
companion to ``hub/HubDriveSmoke.py``. It collects every ``DATA,...`` row
the hub script prints during the run, and once the run finishes it renders
a static multi panel diagnostic plot of the implemented state vector

    [theta, thetaDot, phi, phiDot]

plus the issued left/right wheel command and the safety gate status. Use
this to eyeball whether the body tilt stayed inside the drive gate, what
``phi``/``phiDot`` actually did, and how the issued command lined up with
the schedule.

Usage:

    # Spawn pybricksdev, run HubDriveSmoke on the hub, plot when it ends.
    python scripts/PlotHubDriveSmoke.py

    # Do not upload this plotter to the hub with pybricksdev. It is a
    # laptop-side program that launches hub/HubDriveSmoke.py for you.

    # Read a previously captured log file (one DATA row per line).
    python scripts/PlotHubDriveSmoke.py --log path/to/run.txt

    # Pipe stdout from an external pybricksdev invocation.
    pybricksdev run ble hub/HubDriveSmoke.py | python scripts/PlotHubDriveSmoke.py --stdin
"""

from __future__ import annotations

import sys

if getattr(getattr(sys, "implementation", None), "name", "") == "micropython":
    raise SystemExit(
        "PlotHubDriveSmoke.py runs on your laptop with normal Python, not on the "
        "SPIKE hub. Upload/run hub/HubDriveSmoke.py on the hub, then run this "
        "on your laptop: python scripts/PlotHubDriveSmoke.py"
    )

import argparse
import csv
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HUB_SCRIPT = REPO_ROOT / "hub" / "HubDriveSmoke.py"
DATA_PREFIX = "DATA,"
# Mirror the constant on the hub side. The plotter draws this as a
# horizontal envelope on the theta panel so the operator can see when the
# tilt is approaching the drive gate.
DEFAULT_MAX_TILT_FOR_MOTION_DEG = 50.0  # ~0.87 rad from configs/Default.yaml


@dataclass(frozen=True)
class DriveSample:
    """One ``DATA,...`` row from HubDriveSmoke."""

    t_s: float
    leg: str
    theta_deg: float
    theta_dot_deg_per_sec: float
    phi_deg: float
    phi_dot_deg_per_sec: float
    left_cmd_deg_per_sec: float
    right_cmd_deg_per_sec: float
    gate: str  # "SAFE" or "GATED"

    @property
    def gated(self) -> bool:
        return self.gate.upper() == "GATED"


def ParseDriveSmokeLine(line: str) -> DriveSample | None:
    """Return a :class:`DriveSample` for one HubDriveSmoke ``DATA`` row.

    Returns ``None`` for any line that is not a ``DATA,...`` telemetry row
    so non telemetry print output (banners, leg headers, gate transition
    messages, summary) is ignored without raising.
    """
    dataStart = line.find(DATA_PREFIX)
    if dataStart < 0:
        return None
    payload = line[dataStart + len(DATA_PREFIX) :]
    fields = next(csv.reader([payload]))
    if len(fields) < 9:
        return None
    try:
        return DriveSample(
            t_s=float(fields[0]),
            leg=fields[1],
            theta_deg=float(fields[2]),
            theta_dot_deg_per_sec=float(fields[3]),
            phi_deg=float(fields[4]),
            phi_dot_deg_per_sec=float(fields[5]),
            left_cmd_deg_per_sec=float(fields[6]),
            right_cmd_deg_per_sec=float(fields[7]),
            gate=fields[8],
        )
    except ValueError:
        return None


def CollectSamples(stream: TextIO) -> list[DriveSample]:
    """Drain ``stream`` until EOF and return all parsed samples in order.

    Non telemetry lines are echoed to stderr so the operator still sees
    the hub side banner, leg headers, gate transition messages and the
    final summary while the run is in progress.
    """
    samples: list[DriveSample] = []
    for rawLine in stream:
        line = rawLine.rstrip("\r\n")
        sample = ParseDriveSmokeLine(line)
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


def CollectFromPybricksdev(args: argparse.Namespace) -> list[DriveSample]:
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


def CollectFromLogFile(path: Path) -> list[DriveSample]:
    if not path.exists():
        raise FileNotFoundError(f"log file not found: {path}")
    with path.open("r", encoding="utf-8") as fh:
        return CollectSamples(fh)


def ContiguousGatedSpans(samples: list[DriveSample]) -> list[tuple[float, float]]:
    """Return ``(t_start, t_end)`` spans where the gate was active.

    Used to shade the gated regions of the theta panel so the operator can
    immediately see when the body went over the drive tilt limit.
    """
    spans: list[tuple[float, float]] = []
    spanStart: float | None = None
    lastTime: float = 0.0
    for sample in samples:
        if sample.gated and spanStart is None:
            spanStart = sample.t_s
        elif not sample.gated and spanStart is not None:
            spans.append((spanStart, sample.t_s))
            spanStart = None
        lastTime = sample.t_s
    if spanStart is not None:
        spans.append((spanStart, lastTime))
    return spans


def LegBoundaries(samples: list[DriveSample]) -> list[tuple[str, float]]:
    """Return ``(legLabel, t_start)`` for each leg transition."""
    boundaries: list[tuple[str, float]] = []
    previousLeg: str | None = None
    for sample in samples:
        if sample.leg != previousLeg:
            boundaries.append((sample.leg, sample.t_s))
            previousLeg = sample.leg
    return boundaries


def Plot(samples: list[DriveSample], maxTiltDeg: float, runName: str = "HubDriveSmoke") -> int:
    if not samples:
        print(
            "No DATA rows were collected. Nothing to plot. Confirm " + runName + " "
            "ran and printed DATA rows.",
            file=sys.stderr,
        )
        return 1

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            "matplotlib is required for plotting. Install it with: pip install matplotlib",
            file=sys.stderr,
        )
        return 1

    times = [s.t_s for s in samples]
    theta = [s.theta_deg for s in samples]
    thetaDot = [s.theta_dot_deg_per_sec for s in samples]
    phi = [s.phi_deg for s in samples]
    phiDot = [s.phi_dot_deg_per_sec for s in samples]
    leftCmd = [s.left_cmd_deg_per_sec for s in samples]
    rightCmd = [s.right_cmd_deg_per_sec for s in samples]

    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(10, 9), num=runName + " run")
    fig.suptitle(
        runName + ": estimator state and drive command\n"
        "implemented state [theta, thetaDot, phi, phiDot]"
    )

    axTheta, axThetaDot, axPhi, axPhiDot, axCmd = axes

    # ----- theta panel with the drive gate envelope shaded. -----
    axTheta.plot(times, theta, color="tab:blue", linewidth=1.6, label="theta")
    axTheta.axhline(+maxTiltDeg, color="tab:red", linestyle="--", linewidth=1.0,
                    label="max tilt for motion (+/-)")
    axTheta.axhline(-maxTiltDeg, color="tab:red", linestyle="--", linewidth=1.0)
    axTheta.set_ylabel("theta (deg)")
    axTheta.grid(True, alpha=0.35)
    axTheta.legend(loc="upper right", fontsize=8)

    # Shade the contiguous regions where the hub side gate was active.
    spans = ContiguousGatedSpans(samples)
    for spanStart, spanEnd in spans:
        for ax in axes:
            ax.axvspan(spanStart, spanEnd, color="tab:red", alpha=0.10)

    axThetaDot.plot(times, thetaDot, color="tab:orange", linewidth=1.4, label="thetaDot")
    axThetaDot.set_ylabel("thetaDot (deg/s)")
    axThetaDot.grid(True, alpha=0.35)
    axThetaDot.legend(loc="upper right", fontsize=8)

    axPhi.plot(times, phi, color="tab:green", linewidth=1.6, label="phi")
    axPhi.set_ylabel("phi (deg)")
    axPhi.grid(True, alpha=0.35)
    axPhi.legend(loc="upper right", fontsize=8)

    axPhiDot.plot(times, phiDot, color="tab:purple", linewidth=1.4, label="phiDot")
    axPhiDot.set_ylabel("phiDot (deg/s)")
    axPhiDot.grid(True, alpha=0.35)
    axPhiDot.legend(loc="upper right", fontsize=8)

    axCmd.plot(times, leftCmd, color="tab:brown", linewidth=1.4, label="left cmd")
    axCmd.plot(times, rightCmd, color="tab:cyan", linewidth=1.4, linestyle="--",
               label="right cmd")
    axCmd.set_ylabel("wheel cmd (deg/s)")
    axCmd.set_xlabel("time (s)")
    axCmd.grid(True, alpha=0.35)
    axCmd.legend(loc="upper right", fontsize=8)

    # Mark each leg transition with a thin vertical line and a top label.
    for legLabel, legStart in LegBoundaries(samples):
        for ax in axes:
            ax.axvline(legStart, color="black", alpha=0.25, linewidth=0.8)
        axTheta.text(
            legStart,
            axTheta.get_ylim()[1],
            " " + legLabel,
            verticalalignment="top",
            horizontalalignment="left",
            fontsize=8,
            color="black",
            alpha=0.6,
        )

    # Operator readable summary line on the bottom panel.
    safeCount = sum(1 for s in samples if not s.gated)
    gatedCount = len(samples) - safeCount
    durationSec = times[-1] - times[0] if len(times) >= 2 else 0.0
    durationText = "{:.2f}".format(durationSec)  # noqa: UP032 - keeps mpy-cross parseable.
    summary = "  ".join(
        (
            "samples=" + str(len(samples)),
            "duration=" + durationText + "s",
            "safe=" + str(safeCount),
            "gated=" + str(gatedCount),
        )
    )
    fig.text(0.01, 0.005, summary, fontsize=8, color="dimgray")

    fig.tight_layout(rect=(0.0, 0.02, 1.0, 0.96))
    plt.show()
    return 0


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read HubDriveSmoke output from stdin instead of launching pybricksdev",
    )
    parser.add_argument(
        "--log",
        type=Path,
        help="read HubDriveSmoke output from a saved file",
    )
    parser.add_argument(
        "--hub-script",
        type=Path,
        default=DEFAULT_HUB_SCRIPT,
        help="HubDriveSmoke.py path passed to pybricksdev",
    )
    parser.add_argument(
        "--name",
        help='optional hub name for pybricksdev, for example "Pybricks Hub"',
    )
    parser.add_argument(
        "--max-tilt-deg",
        type=float,
        default=DEFAULT_MAX_TILT_FOR_MOTION_DEG,
        help="value of MAX_TILT_FOR_MOTION_DEG used by HubDriveSmoke; drawn as the gate envelope",
    )
    return parser.parse_args()


def ValidateArgs(args: argparse.Namespace) -> int:
    if Path(args.hub_script).resolve() == Path(__file__).resolve():
        print(
            "PlotHubDriveSmoke.py is the laptop-side plotter, not the hub program. "
            "Use the default command instead:\n\n"
            "    python scripts/PlotHubDriveSmoke.py\n\n"
            "or run the hub program directly with:\n\n"
            "    pybricksdev run ble hub/HubDriveSmoke.py",
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

    return Plot(samples, maxTiltDeg=args.max_tilt_deg)


if __name__ == "__main__":
    raise SystemExit(Main())
