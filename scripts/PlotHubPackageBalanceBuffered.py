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

import math
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


def LyapunovLevelSet(
    v0: float,
    kWeight: float,
    numPoints: int = 200,
) -> tuple[list[float], list[float]]:
    """Sample the ellipse V = 0.5*thetaDot^2 + k*e^2 = v0 in (e, thetaDot)."""
    if v0 <= 0.0 or kWeight <= 0.0:
        return [], []
    semiE = math.sqrt(v0 / kWeight)
    semiDot = math.sqrt(2.0 * v0)
    xs: list[float] = []
    ys: list[float] = []
    for i in range(numPoints + 1):
        ang = 2.0 * math.pi * i / numPoints
        xs.append(semiE * math.cos(ang))
        ys.append(semiDot * math.sin(ang))
    return xs, ys


def PlotBufferedDiagnostics(
    samples: list["base.BalanceSample"],
    outputPath: Path,
    showPlot: bool,
    runName: str = RUN_NAME,
    toleranceDeg: float = 2.0,
    lyapunovThetaWeight: float = 1.0,
) -> int:
    """Render the high value diagnostic figure for a buffered balance run.

    Panels: tilt tracking error, theta/thetaDot phase portrait, Lyapunov
    like energy V = 0.5*thetaDot^2 + k*e^2, and wheel command saturation.
    """
    if not samples:
        print(
            "No DATA rows were collected. Nothing to plot for diagnostics.",
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
    theta = [s.theta_deg for s in samples]
    thetaRef = [s.theta_ref_deg for s in samples]
    thetaDot = [s.theta_dot_deg_per_sec for s in samples]
    rawCmd = [s.raw_cmd_deg_per_sec for s in samples]
    safeCmd = [s.safe_cmd_deg_per_sec for s in samples]

    tiltError = [t - r for t, r in zip(theta, thetaRef)]
    saturation = [r - s for r, s in zip(rawCmd, safeCmd)]
    lyapunov = [
        0.5 * td * td + lyapunovThetaWeight * e * e
        for td, e in zip(thetaDot, tiltError)
    ]

    vMax = max(lyapunov) if lyapunov else 0.0
    meanTiltError = sum(tiltError) / len(tiltError) if tiltError else 0.0
    meanThetaDot = sum(thetaDot) / len(thetaDot) if thetaDot else 0.0

    trippedIdx = None
    for i, sample in enumerate(samples):
        if sample.tripped:
            trippedIdx = i
            break

    fig, axes = plt.subplots(2, 2, figsize=(12, 9), num=runName + " diagnostics")
    fig.suptitle(
        runName + ": balance diagnostics\n"
        "tilt error, phase portrait, Lyapunov like energy, saturation",
        fontsize=15,
    )

    axErr, axPhase = axes[0]
    axLyap, axSat = axes[1]

    axErr.axhspan(-toleranceDeg, toleranceDeg, color="tab:green", alpha=0.12, label="+/- tol")
    axErr.axhline(0.0, color="black", linewidth=0.8)
    axErr.plot(times, tiltError, color="tab:red", linewidth=1.3, label="theta - theta ref")
    axErr.set_ylabel("tilt error (deg)")
    axErr.set_xlabel("time (s)")
    axErr.grid(True, alpha=0.35)
    axErr.legend(loc="upper right")
    axErr.set_title("tilt tracking error")

    axPhase.plot(tiltError, thetaDot, color="tab:blue", linewidth=0.9, alpha=0.75)
    axPhase.scatter([tiltError[0]], [thetaDot[0]], color="tab:green", s=45, label="start", zorder=5)
    axPhase.scatter([tiltError[-1]], [thetaDot[-1]], color="tab:orange", s=45, label="end", zorder=5)
    if trippedIdx is not None:
        axPhase.scatter(
            [tiltError[trippedIdx]],
            [thetaDot[trippedIdx]],
            color="tab:red",
            s=70,
            marker="x",
            label="tripped",
            zorder=6,
        )
    axPhase.scatter(
        [meanTiltError],
        [meanThetaDot],
        color="tab:purple",
        s=140,
        marker="*",
        label="centroid",
        zorder=7,
    )
    vTol = lyapunovThetaWeight * toleranceDeg * toleranceDeg
    xsTol, ysTol = LyapunovLevelSet(vTol, lyapunovThetaWeight)
    if xsTol:
        axPhase.plot(
            xsTol,
            ysTol,
            color="tab:green",
            linestyle="--",
            linewidth=1.2,
            label="V = k*tol^2",
        )
    xsVmax, ysVmax = LyapunovLevelSet(vMax, lyapunovThetaWeight)
    if xsVmax:
        axPhase.plot(
            xsVmax,
            ysVmax,
            color="tab:red",
            linestyle=":",
            linewidth=1.2,
            label="V = V_max",
        )
    axPhase.axhline(0.0, color="black", linewidth=0.6, alpha=0.5)
    axPhase.axvline(0.0, color="black", linewidth=0.6, alpha=0.5)
    axPhase.set_xlabel("theta - theta ref (deg)")
    axPhase.set_ylabel("thetaDot (deg/s)")
    axPhase.grid(True, alpha=0.35)
    axPhase.legend(loc="upper right")
    axPhase.set_title("phase portrait")
    eMin, eMax = min(tiltError), max(tiltError)
    dotMin, dotMax = min(thetaDot), max(thetaDot)
    eMargin = max(0.25, 0.08 * (eMax - eMin))
    dotMargin = max(0.25, 0.08 * (dotMax - dotMin))
    axPhase.set_xlim(eMin - eMargin, eMax + eMargin)
    axPhase.set_ylim(dotMin - dotMargin, dotMax + dotMargin)

    axLyap.plot(times, lyapunov, color="tab:purple", linewidth=1.3, label="V")
    axLyap.set_ylabel("V = 0.5 thetaDot^2 + k e^2")
    axLyap.set_xlabel("time (s)")
    axLyap.grid(True, alpha=0.35)
    axLyap.legend(loc="upper right")
    axLyap.set_title("Lyapunov like energy (k=" + "{:.2f}".format(lyapunovThetaWeight) + ")")

    axSat.plot(times, rawCmd, color="tab:brown", linewidth=0.9, alpha=0.75, label="raw")
    axSat.plot(times, safeCmd, color="tab:cyan", linewidth=0.9, linestyle="--", alpha=0.9, label="applied")
    axSat.fill_between(times, rawCmd, safeCmd, color="tab:red", alpha=0.25, label="clamped")
    axSat.axhline(0.0, color="black", linewidth=0.6)
    axSat.set_ylabel("wheel cmd (deg/s)")
    axSat.set_xlabel("time (s)")
    axSat.grid(True, alpha=0.35)
    axSat.legend(loc="upper right")
    axSat.set_title("control saturation")

    for sample in samples:
        if sample.tripped:
            for ax in (axErr, axLyap, axSat):
                ax.axvline(sample.t_s, color="tab:red", alpha=0.18, linewidth=0.9)

    numSaturated = sum(1 for s in saturation if abs(s) > 0.5)
    saturationPct = 100.0 * numSaturated / max(len(saturation), 1)
    rmsError = math.sqrt(sum(e * e for e in tiltError) / max(len(tiltError), 1))
    fig.text(
        0.01,
        0.005,
        "  ".join(
            (
                "samples=" + str(len(samples)),
                "rms tilt error=" + "{:.2f}".format(rmsError) + " deg",
                "saturation=" + "{:.1f}".format(saturationPct) + "%",
                "V max=" + "{:.2f}".format(vMax),
                "final status=" + samples[-1].status,
            )
        ),
        fontsize=10,
        color="dimgray",
    )

    fig.tight_layout(rect=(0.0, 0.02, 1.0, 0.94))
    outputPath = Path(outputPath)
    outputPath.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(outputPath, dpi=160)
    print(f"diagnostics plot written to: {outputPath}")
    if showPlot:
        plt.show()
    else:
        plt.close(fig)
    return 0


def PlotBufferedSeparatePanels(
    samples: list["base.BalanceSample"],
    outputDir: Path,
    filePrefix: str,
    toleranceDeg: float = 2.0,
    lyapunovThetaWeight: float = 1.0,
) -> int:
    """Export each panel as its own PNG into outputDir.

    Saves nine files named ``{filePrefix}_{PanelName}.png`` covering the
    five time series from the base plot (theta, thetaDot, phi, phiDot,
    wheel cmd) and the four diagnostic panels (tilt error, phase portrait,
    Lyapunov like energy, control saturation). Individual panels are
    written to disk without opening interactive windows, so this runs
    alongside the combined figures rather than replacing them.
    """
    if not samples:
        print(
            "No DATA rows were collected. Nothing to export as separate panels.",
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
    thetaRef = [s.theta_ref_deg for s in samples]
    thetaDot = [s.theta_dot_deg_per_sec for s in samples]
    phi = [s.phi_deg for s in samples]
    phiDot = [s.phi_dot_deg_per_sec for s in samples]
    rawCmd = [s.raw_cmd_deg_per_sec for s in samples]
    safeCmd = [s.safe_cmd_deg_per_sec for s in samples]

    tiltError = [t - r for t, r in zip(theta, thetaRef)]
    lyapunov = [
        0.5 * td * td + lyapunovThetaWeight * e * e
        for td, e in zip(thetaDot, tiltError)
    ]
    vMax = max(lyapunov) if lyapunov else 0.0
    meanTiltError = sum(tiltError) / len(tiltError) if tiltError else 0.0
    meanThetaDot = sum(thetaDot) / len(thetaDot) if thetaDot else 0.0

    trippedTimes = [s.t_s for s in samples if s.tripped]
    trippedIdx = None
    for i, sample in enumerate(samples):
        if sample.tripped:
            trippedIdx = i
            break

    outputDir = Path(outputDir)
    outputDir.mkdir(parents=True, exist_ok=True)

    def MarkTrips(ax) -> None:
        for tt in trippedTimes:
            ax.axvline(tt, color="tab:red", alpha=0.18, linewidth=0.9)

    def SavePanel(fig, name: str) -> None:
        fig.tight_layout()
        path = outputDir / f"{filePrefix}_{name}.png"
        fig.savefig(path, dpi=160)
        print(f"panel written to   : {path}")
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, theta, color="tab:blue", linewidth=1.6, label="theta")
    ax.plot(times, thetaRef, color="tab:red", linestyle="--", linewidth=1.1, label="theta ref")
    ax.set_title(filePrefix + ": tilt theta vs reference")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("theta (deg)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    MarkTrips(ax)
    SavePanel(fig, "Theta")

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, thetaDot, color="tab:orange", linewidth=1.4, label="thetaDot")
    ax.set_title(filePrefix + ": tilt rate thetaDot")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("thetaDot (deg/s)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    MarkTrips(ax)
    SavePanel(fig, "ThetaDot")

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, phi, color="tab:green", linewidth=1.6, label="phi")
    ax.set_title(filePrefix + ": wheel angle phi")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("phi (deg)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    MarkTrips(ax)
    SavePanel(fig, "Phi")

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, phiDot, color="tab:purple", linewidth=1.4, label="phiDot")
    ax.set_title(filePrefix + ": wheel rate phiDot")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("phiDot (deg/s)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    MarkTrips(ax)
    SavePanel(fig, "PhiDot")

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, rawCmd, color="tab:brown", linewidth=1.4, label="raw cmd")
    ax.plot(times, safeCmd, color="tab:cyan", linestyle="--", linewidth=1.4, label="applied cmd")
    ax.set_title(filePrefix + ": wheel velocity command")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("wheel cmd (deg/s)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    MarkTrips(ax)
    SavePanel(fig, "WheelCmd")

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.axhspan(-toleranceDeg, toleranceDeg, color="tab:green", alpha=0.12, label="+/- tol")
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.plot(times, tiltError, color="tab:red", linewidth=1.3, label="theta - theta ref")
    ax.set_title(filePrefix + ": tilt tracking error")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("tilt error (deg)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    MarkTrips(ax)
    SavePanel(fig, "TiltError")

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(tiltError, thetaDot, color="tab:blue", linewidth=0.9, alpha=0.75)
    ax.scatter([tiltError[0]], [thetaDot[0]], color="tab:green", s=45, label="start", zorder=5)
    ax.scatter([tiltError[-1]], [thetaDot[-1]], color="tab:orange", s=45, label="end", zorder=5)
    if trippedIdx is not None:
        ax.scatter(
            [tiltError[trippedIdx]],
            [thetaDot[trippedIdx]],
            color="tab:red",
            s=70,
            marker="x",
            label="tripped",
            zorder=6,
        )
    ax.scatter(
        [meanTiltError],
        [meanThetaDot],
        color="tab:purple",
        s=140,
        marker="*",
        label="centroid",
        zorder=7,
    )
    vTol = lyapunovThetaWeight * toleranceDeg * toleranceDeg
    xsTol, ysTol = LyapunovLevelSet(vTol, lyapunovThetaWeight)
    if xsTol:
        ax.plot(xsTol, ysTol, color="tab:green", linestyle="--", linewidth=1.2, label="V = k*tol^2")
    xsVmax, ysVmax = LyapunovLevelSet(vMax, lyapunovThetaWeight)
    if xsVmax:
        ax.plot(xsVmax, ysVmax, color="tab:red", linestyle=":", linewidth=1.2, label="V = V_max")
    ax.axhline(0.0, color="black", linewidth=0.6, alpha=0.5)
    ax.axvline(0.0, color="black", linewidth=0.6, alpha=0.5)
    ax.set_title(filePrefix + ": phase portrait")
    ax.set_xlabel("theta - theta ref (deg)")
    ax.set_ylabel("thetaDot (deg/s)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    eMin, eMax = min(tiltError), max(tiltError)
    dotMin, dotMax = min(thetaDot), max(thetaDot)
    eMargin = max(0.25, 0.08 * (eMax - eMin))
    dotMargin = max(0.25, 0.08 * (dotMax - dotMin))
    ax.set_xlim(eMin - eMargin, eMax + eMargin)
    ax.set_ylim(dotMin - dotMargin, dotMax + dotMargin)
    SavePanel(fig, "PhasePortrait")

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, lyapunov, color="tab:purple", linewidth=1.3, label="V")
    ax.set_title(
        filePrefix + ": Lyapunov like energy (k=" + "{:.2f}".format(lyapunovThetaWeight) + ")"
    )
    ax.set_xlabel("time (s)")
    ax.set_ylabel("V = 0.5 thetaDot^2 + k e^2")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    MarkTrips(ax)
    SavePanel(fig, "LyapunovEnergy")

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, rawCmd, color="tab:brown", linewidth=0.9, alpha=0.75, label="raw")
    ax.plot(times, safeCmd, color="tab:cyan", linewidth=0.9, linestyle="--", alpha=0.9, label="applied")
    ax.fill_between(times, rawCmd, safeCmd, color="tab:red", alpha=0.25, label="clamped")
    ax.axhline(0.0, color="black", linewidth=0.6)
    ax.set_title(filePrefix + ": control saturation")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("wheel cmd (deg/s)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    MarkTrips(ax)
    SavePanel(fig, "ControlSaturation")

    return 0


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

    baseStatus = base.Plot(
        samples,
        outputPath=args.output,
        showPlot=not args.no_show,
        runName=RUN_NAME,
    )
    diagnosticsPath = args.output.with_name(
        args.output.stem + "Diagnostics" + args.output.suffix
    )
    diagnosticsStatus = PlotBufferedDiagnostics(
        samples,
        outputPath=diagnosticsPath,
        showPlot=not args.no_show,
    )
    panelsDir = args.output.parent / (args.output.stem + "Panels")
    panelsStatus = PlotBufferedSeparatePanels(
        samples,
        outputDir=panelsDir,
        filePrefix=args.output.stem,
    )
    return baseStatus or diagnosticsStatus or panelsStatus


if __name__ == "__main__":
    raise SystemExit(Main())
