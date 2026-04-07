"""Live plot state telemetry streamed by hub/HubMain.py.

The SPIKE hub cannot open desktop plot windows, so HubMain.py prints CSV-like
``DATA`` rows and this laptop-side script renders the state vector

    x = [theta, theta_dot, p, p_dot]^T

Run and plot HubMain directly from the laptop:

    python scripts/PlotHubMainLive.py

Or plot an already-running stream:

    pybricksdev run ble hub/HubMain.py | python scripts/PlotHubMainLive.py --stdin
"""

import sys

if getattr(getattr(sys, "implementation", None), "name", "") == "micropython":
    raise SystemExit(
        "PlotHubMainLive.py runs on your laptop with normal Python, not on the SPIKE hub. "
        "Upload/run hub/HubMain.py on the hub, then run this on your laptop: "
        "python scripts/PlotHubMainLive.py"
    )

import argparse
import csv
import os
import queue
import subprocess
import threading
from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HUB_SCRIPT = REPO_ROOT / "hub" / "HubMain.py"
DATA_PREFIX = "DATA,"
DEG_TO_RAD = 0.017453292519943295
REQUIRED_FIELDS = [
    "t_s",
    "theta_deg",
    "theta_dot_deg_per_sec",
    "p_m",
    "p_dot_m_per_sec",
]
EXTRA_FIELDS = [
    "left_angle_deg",
    "left_rate_deg_per_sec",
    "right_angle_deg",
    "right_rate_deg_per_sec",
]


@dataclass(frozen=True)
class HubSample:
    t_s: float
    theta_deg: float
    theta_dot_deg_per_sec: float
    p_m: float
    p_dot_m_per_sec: float
    left_angle_deg: float = 0.0
    left_rate_deg_per_sec: float = 0.0
    right_angle_deg: float = 0.0
    right_rate_deg_per_sec: float = 0.0


@dataclass(frozen=True)
class PlotSpec:
    field: str
    label: str
    ylabel: str
    color: str


@dataclass
class LineBinding:
    spec: PlotSpec
    axis: Any
    line: Any


class LiveSampleBuffer:
    def __init__(self, windowSec: float) -> None:
        if windowSec <= 0.0:
            raise ValueError("windowSec must be positive")
        self._windowSec = windowSec
        self._samples: deque[HubSample] = deque()

    def Append(self, sample: HubSample) -> None:
        self._samples.append(sample)
        cutoff = sample.t_s - self._windowSec
        while self._samples and self._samples[0].t_s < cutoff:
            self._samples.popleft()

    def HasSamples(self) -> bool:
        return bool(self._samples)

    def TimeBounds(self) -> tuple[float, float]:
        if not self._samples:
            return (0.0, self._windowSec)
        end = max(self._samples[-1].t_s, self._windowSec)
        return (end - self._windowSec, end)

    def Series(self, field: str) -> tuple[list[float], list[float]]:
        times = [sample.t_s for sample in self._samples]
        values = [float(getattr(sample, field)) for sample in self._samples]
        return times, values


def ParseHubMainLine(line: str) -> HubSample | None:
    dataStart = line.find(DATA_PREFIX)
    if dataStart < 0:
        return None

    payload = line[dataStart + len(DATA_PREFIX) :]
    values = next(csv.reader([payload]))
    if len(values) >= 16:
        # Compatibility with the earlier HubMain telemetry layout, which
        # emitted degrees plus a few unused IMU fields.
        return HubSample(
            t_s=float(values[0]),
            theta_deg=float(values[1]),
            theta_dot_deg_per_sec=float(values[2]),
            p_m=float(values[3]),
            p_dot_m_per_sec=float(values[4]),
            left_angle_deg=float(values[5]),
            left_rate_deg_per_sec=float(values[6]),
            right_angle_deg=float(values[7]),
            right_rate_deg_per_sec=float(values[8]),
        )

    if len(values) < len(REQUIRED_FIELDS):
        raise ValueError(
            "HubMain DATA row has "
            f"{len(values)} values; expected at least {len(REQUIRED_FIELDS)}"
        )

    fieldNames = REQUIRED_FIELDS + EXTRA_FIELDS[: max(0, len(values) - len(REQUIRED_FIELDS))]
    row = dict(zip(fieldNames, values[: len(fieldNames)], strict=True))
    try:
        numeric = {field: float(value) for field, value in row.items()}
    except ValueError as exc:
        raise ValueError(f"HubMain DATA row contains a non-numeric value: {payload}") from exc
    return HubSample(**numeric)


def ReadTelemetry(
    stream: TextIO,
    samples: queue.Queue[HubSample],
    stopEvent: threading.Event,
) -> None:
    for rawLine in stream:
        if stopEvent.is_set():
            break

        line = rawLine.strip()
        if not line:
            continue

        try:
            sample = ParseHubMainLine(line)
        except ValueError as exc:
            print(f"Ignored telemetry line: {exc}", file=sys.stderr)
            continue

        if sample is None:
            print(line, file=sys.stderr)
            continue

        samples.put(sample)

    stopEvent.set()


def BuildPybricksCommand(args: argparse.Namespace) -> list[str]:
    command = ["pybricksdev", "run", "ble"]
    if args.name:
        command.extend(["--name", args.name])
    command.append(str(args.hub_script))
    return command


def StartReader(
    args: argparse.Namespace,
    samples: queue.Queue[HubSample],
    stopEvent: threading.Event,
) -> tuple[subprocess.Popen[str] | None, threading.Thread]:
    if args.stdin:
        thread = threading.Thread(
            target=ReadTelemetry,
            args=(sys.stdin, samples, stopEvent),
            daemon=True,
        )
        thread.start()
        return None, thread

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

    thread = threading.Thread(
        target=ReadTelemetry,
        args=(process.stdout, samples, stopEvent),
        daemon=True,
    )
    thread.start()
    return process, thread


def MakeFigure(plt: Any, title: str, specs: Iterable[PlotSpec]) -> list[LineBinding]:
    specs = list(specs)
    fig, axes = plt.subplots(len(specs), 1, sharex=True, num=title)
    fig.suptitle(title)
    bindings: list[LineBinding] = []

    if len(specs) == 1:
        axes = [axes]

    for axis, spec in zip(axes, specs, strict=True):
        (line,) = axis.plot([], [], color=spec.color, linewidth=1.6, label=spec.label)
        axis.set_ylabel(spec.ylabel)
        axis.grid(True, alpha=0.35)
        axis.legend(loc="upper right")
        axis.margins(x=0.0)
        bindings.append(LineBinding(spec=spec, axis=axis, line=line))

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    return bindings


def BuildFigures(plt: Any, showMotors: bool = False) -> list[LineBinding]:
    bindings: list[LineBinding] = []
    bindings.extend(
        MakeFigure(
            plt,
            "HubMain State Vector x(t)",
            [
                PlotSpec("theta_deg", "theta", "deg", "tab:blue"),
                PlotSpec("theta_dot_deg_per_sec", "theta_dot", "deg/s", "tab:orange"),
                PlotSpec("p_m", "p", "m", "tab:green"),
                PlotSpec("p_dot_m_per_sec", "p_dot", "m/s", "tab:red"),
            ],
        )
    )
    if showMotors:
        bindings.extend(
            MakeFigure(
                plt,
                "HubMain Motor Encoders",
                [
                    PlotSpec("left_angle_deg", "left angle", "deg", "tab:purple"),
                    PlotSpec("left_rate_deg_per_sec", "left rate", "deg/s", "tab:brown"),
                    PlotSpec("right_angle_deg", "right angle", "deg", "tab:cyan"),
                    PlotSpec("right_rate_deg_per_sec", "right rate", "deg/s", "tab:pink"),
                ],
            )
        )
    return bindings


def DrainSamples(samples: queue.Queue[HubSample], buffer: LiveSampleBuffer) -> bool:
    changed = False
    while True:
        try:
            sample = samples.get_nowait()
        except queue.Empty:
            return changed
        buffer.Append(sample)
        changed = True


def UpdatePlots(buffer: LiveSampleBuffer, bindings: list[LineBinding]) -> None:
    if not buffer.HasSamples():
        return

    xmin, xmax = buffer.TimeBounds()
    axes = set()
    for binding in bindings:
        times, values = buffer.Series(binding.spec.field)
        binding.line.set_data(times, values)
        binding.axis.set_xlim(xmin, xmax)
        binding.axis.relim()
        binding.axis.autoscale_view(scalex=False, scaley=True)
        axes.add(binding.axis)

    for axis in axes:
        axis.figure.canvas.draw_idle()


def RunPlotLoop(
    args: argparse.Namespace,
    samples: queue.Queue[HubSample],
    stopEvent: threading.Event,
) -> None:
    import matplotlib.pyplot as plt

    buffer = LiveSampleBuffer(windowSec=args.window_sec)
    bindings = BuildFigures(plt, showMotors=args.show_motors)
    plt.show(block=False)

    while plt.get_fignums():
        if DrainSamples(samples, buffer):
            UpdatePlots(buffer, bindings)
        plt.pause(args.interval_ms / 1000.0)

    while DrainSamples(samples, buffer):
        UpdatePlots(buffer, bindings)
        plt.pause(args.interval_ms / 1000.0)


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read HubMain output from stdin instead of launching pybricksdev",
    )
    parser.add_argument(
        "--hub-script",
        type=Path,
        default=DEFAULT_HUB_SCRIPT,
        help="HubMain.py path passed to pybricksdev",
    )
    parser.add_argument(
        "--name",
        help='optional hub name for pybricksdev, for example "Pybricks Hub"',
    )
    parser.add_argument(
        "--window-sec",
        type=float,
        default=10.0,
        help="rolling plot window length in seconds",
    )
    parser.add_argument(
        "--interval-ms",
        type=float,
        default=40.0,
        help="plot refresh interval in milliseconds",
    )
    parser.add_argument(
        "--show-motors",
        action="store_true",
        help="also show raw left/right motor encoder angle and rate",
    )
    return parser.parse_args()


def StopProcess(process: subprocess.Popen[str] | None) -> None:
    if process is None or process.poll() is not None:
        return

    process.terminate()
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3.0)


def Main() -> int:
    args = ParseArgs()
    samples: queue.Queue[HubSample] = queue.Queue()
    stopEvent = threading.Event()

    try:
        process, thread = StartReader(args, samples, stopEvent)
    except FileNotFoundError:
        print(
            "Could not find pybricksdev. Install it with: pip install -e .[hub]",
            file=sys.stderr,
        )
        return 1

    try:
        RunPlotLoop(args, samples, stopEvent)
    except KeyboardInterrupt:
        stopEvent.set()
    finally:
        stopEvent.set()
        StopProcess(process)
        thread.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
