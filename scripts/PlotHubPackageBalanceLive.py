"""Live-plot a real package-backed hub balance run.

This laptop-side script does three things in one flow:

1. regenerates ``LegoBalance.HubDriveSmokeRuntime`` from the selected desktop YAML,
2. launches ``src/HubPackageBalance.py`` through ``pybricksdev``,
3. streams the real hub telemetry into live matplotlib plots.

The live plots show:

- tilt reference versus measured tilt,
- the implemented state ``[theta, thetaDot, phi, phiDot]``,
- raw versus applied wheel velocity command.

Usage:

    python scripts/PlotHubPackageBalanceLive.py
    python scripts/PlotHubPackageBalanceLive.py --name "Pybricks Hub"
    pybricksdev run ble src/HubPackageBalance.py | python scripts/PlotHubPackageBalanceLive.py --stdin
"""

import argparse
import csv
import os
import queue
import subprocess
import sys
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

if getattr(getattr(sys, "implementation", None), "name", "") == "micropython":
    raise SystemExit(
        "PlotHubPackageBalanceLive.py runs on your laptop with normal Python, not on "
        "the SPIKE hub. Run: python scripts/PlotHubPackageBalanceLive.py"
    )

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))
MPLCONFIGDIR = Path("/tmp/matplotlib-legobalance")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

DEFAULT_HUB_SCRIPT = SRC_ROOT / "HubPackageBalance.py"
DEFAULT_OUTPUT_PATH = REPO_ROOT / "plots" / "HubPackageBalanceLive.png"
DATA_PREFIX = "DATA,"


def RelativeToRepo(path: Path) -> str:
    try:
        return str(Path(path).resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


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


@dataclass(frozen=True)
class PlotSpec:
    field: str
    label: str
    ylabel: str
    color: str
    axisIndex: int
    linestyle: str = "-"


@dataclass
class LineBinding:
    spec: PlotSpec
    axis: Any
    line: Any


PLOT_SPECS = (
    PlotSpec("theta_deg", "theta", "theta (deg)", "tab:blue", 0),
    PlotSpec("theta_ref_deg", "theta ref", "theta (deg)", "tab:red", 0, "--"),
    PlotSpec("theta_dot_deg_per_sec", "thetaDot", "thetaDot (deg/s)", "tab:orange", 1),
    PlotSpec("phi_deg", "phi", "phi (deg)", "tab:green", 2),
    PlotSpec("phi_dot_deg_per_sec", "phiDot", "phiDot (deg/s)", "tab:purple", 3),
    PlotSpec("raw_cmd_deg_per_sec", "raw cmd", "wheel cmd (deg/s)", "tab:brown", 4),
    PlotSpec("safe_cmd_deg_per_sec", "applied cmd", "wheel cmd (deg/s)", "tab:cyan", 4, "--"),
)


class LiveSampleBuffer:
    def __init__(self, windowSec: float) -> None:
        if windowSec <= 0.0:
            raise ValueError("windowSec must be positive")
        self._windowSec = windowSec
        self._samples: deque[BalanceSample] = deque()

    def Append(self, sample: BalanceSample) -> None:
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

    def Latest(self) -> BalanceSample | None:
        if not self._samples:
            return None
        return self._samples[-1]


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


def ReadTelemetry(
    stream: TextIO,
    samples: queue.Queue[BalanceSample],
    stopEvent: threading.Event,
) -> None:
    for rawLine in stream:
        if stopEvent.is_set():
            break

        line = rawLine.strip()
        if not line:
            continue

        sample = ParseBalanceLine(line)
        if sample is None:
            print(line, file=sys.stderr)
            continue
        samples.put(sample)

    stopEvent.set()


def ReadTelemetryFromPath(
    path: Path,
    samples: queue.Queue[BalanceSample],
    stopEvent: threading.Event,
) -> None:
    with Path(path).open("r", encoding="utf-8") as fh:
        ReadTelemetry(fh, samples, stopEvent)


def BuildPybricksCommand(args: argparse.Namespace) -> list[str]:
    command = ["pybricksdev", "run", "ble"]
    if args.name:
        command.extend(["--name", args.name])
    command.append(str(args.hub_script))
    return command


def StartReader(
    args: argparse.Namespace,
    samples: queue.Queue[BalanceSample],
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

    if args.log is not None:
        thread = threading.Thread(
            target=ReadTelemetryFromPath,
            args=(args.log, samples, stopEvent),
            daemon=True,
        )
        thread.start()
        return None, thread

    from GenerateHubDriveSmokeRuntime import GenerateHubDriveSmokeRuntime

    generatedPath = GenerateHubDriveSmokeRuntime(args.config)
    print(
        "Generated "
        + RelativeToRepo(generatedPath)
        + " from "
        + RelativeToRepo(Path(args.config)),
        file=sys.stderr,
    )

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


def MakeFigure(plt: Any) -> tuple[Any, list[LineBinding]]:
    fig, axes = plt.subplots(5, 1, sharex=True, num="HubPackageBalance live", figsize=(12, 10))
    fig.suptitle(
        "HubPackageBalance live telemetry\n"
        "real hub state [theta, thetaDot, phi, phiDot] and wheel commands"
    )

    bindings: list[LineBinding] = []
    for spec in PLOT_SPECS:
        axis = axes[spec.axisIndex]
        line = axis.plot(
            [],
            [],
            color=spec.color,
            linewidth=1.6,
            linestyle=spec.linestyle,
            label=spec.label,
        )[0]
        bindings.append(LineBinding(spec=spec, axis=axis, line=line))

    ylabelByAxis = {}
    for spec in PLOT_SPECS:
        ylabelByAxis[spec.axisIndex] = spec.ylabel

    for axisIndex, axis in enumerate(axes):
        axis.set_ylabel(ylabelByAxis[axisIndex])
        axis.grid(True, alpha=0.35)
        axis.legend(loc="upper right")
        axis.margins(x=0.0)

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout(rect=(0.0, 0.03, 1.0, 0.95))
    return fig, bindings


def DrainSamples(samples: queue.Queue[BalanceSample], buffer: LiveSampleBuffer) -> bool:
    changed = False
    while True:
        try:
            sample = samples.get_nowait()
        except queue.Empty:
            return changed
        buffer.Append(sample)
        changed = True


def UpdatePlots(fig: Any, buffer: LiveSampleBuffer, bindings: list[LineBinding]) -> None:
    if not buffer.HasSamples():
        return

    xmin, xmax = buffer.TimeBounds()
    axes = set()
    for binding in bindings:
        times, values = buffer.Series(binding.spec.field)
        binding.line.set_data(times, values)
        binding.axis.set_xlim(xmin, xmax)
        axes.add(binding.axis)

    for axis in axes:
        axis.relim()
        axis.autoscale_view(scalex=False, scaley=True)

    latest = buffer.Latest()
    if latest is not None:
        fig._suptitle.set_text(  # type: ignore[attr-defined]
            "HubPackageBalance live telemetry\n"
            + "status="
            + latest.status
            + "  t="
            + "{:.2f}".format(latest.t_s)
            + "s  theta error="
            + "{:+.2f}".format(latest.theta_deg - latest.theta_ref_deg)
            + " deg"
        )
    fig.canvas.draw_idle()


def RunPlotLoop(
    args: argparse.Namespace,
    samples: queue.Queue[BalanceSample],
    stopEvent: threading.Event,
) -> None:
    import matplotlib.pyplot as plt

    buffer = LiveSampleBuffer(windowSec=args.window_sec)
    fig, bindings = MakeFigure(plt)
    savedOutput = False
    if "agg" not in plt.get_backend().lower():
        plt.show(block=False)

    while plt.get_fignums():
        if DrainSamples(samples, buffer):
            UpdatePlots(fig, buffer, bindings)

        if stopEvent.is_set() and samples.empty():
            if args.output is not None and buffer.HasSamples() and not savedOutput:
                outputPath = Path(args.output)
                outputPath.parent.mkdir(parents=True, exist_ok=True)
                fig.savefig(outputPath, dpi=160)
                print("plot written to  : " + str(outputPath), file=sys.stderr)
                savedOutput = True
            if args.close_when_done:
                plt.close("all")
                break

        plt.pause(args.interval_ms / 1000.0)


def ParseArgs() -> argparse.Namespace:
    from LegoBalance.RobotConfig import DEFAULT_CONFIG_PATH

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read HubPackageBalance output from stdin instead of launching pybricksdev",
    )
    parser.add_argument(
        "--log",
        type=Path,
        help="read HubPackageBalance output from a saved file instead of launching pybricksdev",
    )
    parser.add_argument(
        "--hub-script",
        type=Path,
        default=DEFAULT_HUB_SCRIPT,
        help="HubPackageBalance.py path passed to pybricksdev",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="YAML config used to regenerate the hub runtime before launch",
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
        "--close-when-done",
        action="store_true",
        help="close the figure automatically when the run finishes",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_PATH,
        help="optional PNG path to save the final live-plot frame",
    )
    return parser.parse_args()


def ValidateArgs(args: argparse.Namespace) -> int:
    if args.stdin and args.log is not None:
        print("--stdin and --log are mutually exclusive", file=sys.stderr)
        return 1
    if not args.stdin and args.log is None and not Path(args.config).exists():
        print(f"--config file not found: {args.config}", file=sys.stderr)
        return 1
    if args.window_sec <= 0.0:
        print("--window-sec must be positive", file=sys.stderr)
        return 1
    if args.interval_ms <= 0.0:
        print("--interval-ms must be positive", file=sys.stderr)
        return 1
    return 0


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
    argStatus = ValidateArgs(args)
    if argStatus != 0:
        return argStatus
    samples: queue.Queue[BalanceSample] = queue.Queue()
    stopEvent = threading.Event()

    try:
        process, thread = StartReader(args, samples, stopEvent)
    except FileNotFoundError:
        print(
            "Could not find pybricksdev. Install it with: pip install -e .[hub], "
            "or use --stdin / --log.",
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
