"""Plot a desktop closed-loop balance run using the shared LegoBalance package.

This is the laptop-side companion to ``examples/ClosedLoopSimulation.py``.
Instead of only printing a short textual summary, it runs the package-backed
estimator/controller/safety/mock-hub loop and then renders diagnostic plots for:

- the implemented state ``[theta, thetaDot, phi, phiDot]``,
- tilt reference versus measured tilt,
- raw versus applied wheel command.

The script loads physical parameters and limits from the normal YAML config via
``LegoBalance.LoadConfig``. The default target is upright balance
(``theta_ref = 0 rad``), which matches the current controller design.

Usage:

    python scripts/PlotClosedLoopBalance.py
    python scripts/PlotClosedLoopBalance.py --initial-tilt-rad 0.08
    python scripts/PlotClosedLoopBalance.py --tilt-ref-rad 0.0 --no-show
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

if getattr(getattr(sys, "implementation", None), "name", "") == "micropython":
    raise SystemExit(
        "PlotClosedLoopBalance.py runs on your laptop with normal Python, not on the "
        "SPIKE hub. Run: python scripts/PlotClosedLoopBalance.py"
    )

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))
MPLCONFIGDIR = Path("/tmp/matplotlib-legobalance")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

from LegoBalance.BalanceState import BalanceState
from LegoBalance.ControlInterfaces import Measurement
from LegoBalance.DataLogger import DataLogger
from LegoBalance.MockAdapters import MockHub
from LegoBalance.NonLinearController import NonLinearController
from LegoBalance.RobotConfig import DEFAULT_CONFIG_PATH, LoadConfig, RobotConfig
from LegoBalance.SafetyMonitor import SafetyMonitor
from LegoBalance.StateEstimator import StateEstimator

DEFAULT_DURATION_SEC = 5.0
DEFAULT_INITIAL_TILT_RAD = 0.02
DEFAULT_TILT_REFERENCE_RAD = 0.0
DEFAULT_OUTPUT_PATH = REPO_ROOT / "plots" / "ClosedLoopBalance.png"
DEFAULT_CSV_PATH = REPO_ROOT / "logs" / "ClosedLoopBalance.csv"


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="YAML config path loaded through LegoBalance.LoadConfig",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=DEFAULT_DURATION_SEC,
        help="simulation duration in seconds",
    )
    parser.add_argument(
        "--dt-sec",
        type=float,
        help="simulation time step in seconds; defaults to 1 / control.loopRate",
    )
    parser.add_argument(
        "--initial-tilt-rad",
        type=float,
        default=DEFAULT_INITIAL_TILT_RAD,
        help="initial body tilt in radians; small nonzero values reveal the response",
    )
    parser.add_argument(
        "--tilt-ref-rad",
        type=float,
        default=DEFAULT_TILT_REFERENCE_RAD,
        help="constant tilt reference in radians; 0 keeps the upright target",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_PATH,
        help="path to save the PNG plot",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_CSV_PATH,
        help="path to save the simulation log as CSV",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="save the plot without opening a matplotlib window",
    )
    return parser.parse_args()


def ValidateArgs(args: argparse.Namespace) -> int:
    if args.duration_sec <= 0.0:
        print("--duration-sec must be positive", file=sys.stderr)
        return 1
    if args.dt_sec is not None and args.dt_sec <= 0.0:
        print("--dt-sec must be positive when provided", file=sys.stderr)
        return 1
    return 0


def BuildMeasurement(hub: MockHub, config: RobotConfig, currentTimeSec: float) -> Measurement:
    """Build the estimator input from the mock hub using the real config signs."""
    rawLeftAngle = config.motors.leftEncoderSign * config.motors.forwardSign * hub.LeftMotor.Angle()
    rawRightAngle = (
        config.motors.rightEncoderSign * config.motors.forwardSign * hub.RightMotor.Angle()
    )
    rawLeftRate = config.motors.leftEncoderSign * config.motors.forwardSign * hub.LeftMotor.Velocity()
    rawRightRate = (
        config.motors.rightEncoderSign * config.motors.forwardSign * hub.RightMotor.Velocity()
    )
    return Measurement(
        tiltAngle=config.imu.tiltSign * (hub.Imu.TiltAngleRadians() - config.imu.zeroOffset),
        tiltRate=config.imu.tiltSign * (hub.Imu.TiltRateRadiansPerSec() + config.imu.gyroBias),
        leftWheelAngle=rawLeftAngle,
        rightWheelAngle=rawRightAngle,
        leftWheelRate=rawLeftRate,
        rightWheelRate=rawRightRate,
        timestamp=currentTimeSec,
        valid=True,
    )


def BuildControllerState(state: BalanceState, tiltReferenceRad: float) -> BalanceState:
    """Return the state presented to the controller.

    The controller itself is designed around the upright equilibrium
    ``theta = 0``. This helper keeps the package boundary untouched while
    allowing the plotting script to visualize constant-reference runs by
    shifting the tilt state before it enters the controller.
    """
    if tiltReferenceRad == 0.0:
        return state
    return BalanceState(
        tilt=state.tilt - tiltReferenceRad,
        tiltRate=state.tiltRate,
        phi=state.phi,
        phiDot=state.phiDot,
        timestamp=state.timestamp,
        valid=state.valid,
    )


def RunClosedLoopSimulation(
    config: RobotConfig,
    durationSec: float,
    dtSec: float,
    initialTiltRad: float,
    tiltReferenceRad: float,
) -> tuple[DataLogger, SafetyMonitor]:
    """Run the package-backed desktop closed loop and return the log."""
    hub = MockHub()
    estimator = StateEstimator(config)
    controller = NonLinearController(config)
    safety = SafetyMonitor(config)
    logger = DataLogger(bufferSize=int(math.ceil(durationSec / dtSec)) + 16)

    hub.SetInitialTilt(initialTiltRad)
    safety.Arm(currentTime=0.0)

    steps = int(round(durationSec / dtSec))
    timeSec = 0.0
    for _ in range(steps):
        measurement = BuildMeasurement(hub, config, timeSec)
        actualState = estimator.Update(measurement)
        controllerState = BuildControllerState(actualState, tiltReferenceRad)
        rawCommand = controller.Compute(controllerState)
        safeCommand = safety.Check(actualState, rawCommand, currentTime=timeSec)

        logger.Record(
            actualState,
            safeCommand,
            thetaReference=tiltReferenceRad,
            thetaError=actualState.tilt - tiltReferenceRad,
            rawLeftCommand=rawCommand.leftCommand,
            rawRightCommand=rawCommand.rightCommand,
        )

        hub.LeftMotor.RunVelocity(safeCommand.leftCommand)
        hub.RightMotor.RunVelocity(safeCommand.rightCommand)
        hub.Step(dtSec)
        timeSec += dtSec

        if safety.status.tripped:
            break

    return logger, safety


def PlotSimulation(
    logger: DataLogger,
    safety: SafetyMonitor,
    config: RobotConfig,
    outputPath: Path,
    showPlot: bool,
) -> int:
    records = logger.Records()
    if not records:
        print("Simulation produced no records. Nothing to plot.", file=sys.stderr)
        return 1

    import matplotlib

    if not showPlot:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    outputPath = Path(outputPath)
    outputPath.parent.mkdir(parents=True, exist_ok=True)

    times = [record.timestamp for record in records]
    theta = [record.state.tilt for record in records]
    thetaRef = [float(record.extras["thetaReference"]) for record in records]
    thetaDot = [record.state.tiltRate for record in records]
    phi = [record.state.phi for record in records]
    phiDot = [record.state.phiDot for record in records]
    rawCommand = [float(record.extras["rawLeftCommand"]) for record in records]
    safeCommand = [record.controlOutput.leftCommand for record in records]

    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(12, 11), num="ClosedLoopBalance")
    fig.suptitle(
        "ClosedLoopBalance: package-backed controller run\n"
        "state [theta, thetaDot, phi, phiDot] and wheel velocity command",
        fontsize=16,
    )

    axTheta, axThetaDot, axPhi, axPhiDot, axCmd = axes

    axTheta.plot(times, theta, color="tab:blue", linewidth=1.8, label="theta")
    axTheta.plot(times, thetaRef, color="tab:red", linestyle="--", linewidth=1.2, label="theta ref")
    axTheta.axhline(config.control.maxTilt, color="tab:red", linestyle=":", linewidth=1.0)
    axTheta.axhline(-config.control.maxTilt, color="tab:red", linestyle=":", linewidth=1.0)
    axTheta.set_ylabel("theta (rad)")
    axTheta.grid(True, alpha=0.35)
    axTheta.legend(loc="upper right")

    axThetaDot.plot(times, thetaDot, color="tab:orange", linewidth=1.5, label="thetaDot")
    axThetaDot.axhline(0.0, color="black", alpha=0.25, linewidth=0.8)
    axThetaDot.set_ylabel("thetaDot (rad/s)")
    axThetaDot.grid(True, alpha=0.35)
    axThetaDot.legend(loc="upper right")

    axPhi.plot(times, phi, color="tab:green", linewidth=1.5, label="phi")
    axPhi.axhline(0.0, color="black", alpha=0.25, linewidth=0.8)
    axPhi.set_ylabel("phi (rad)")
    axPhi.grid(True, alpha=0.35)
    axPhi.legend(loc="upper right")

    axPhiDot.plot(times, phiDot, color="tab:purple", linewidth=1.5, label="phiDot")
    axPhiDot.axhline(0.0, color="black", alpha=0.25, linewidth=0.8)
    axPhiDot.set_ylabel("phiDot (rad/s)")
    axPhiDot.grid(True, alpha=0.35)
    axPhiDot.legend(loc="upper right")

    axCmd.plot(times, rawCommand, color="tab:brown", linewidth=1.5, label="raw command")
    axCmd.plot(times, safeCommand, color="tab:cyan", linestyle="--", linewidth=1.5, label="applied command")
    axCmd.axhline(config.control.maxWheelRate, color="tab:red", linestyle=":", linewidth=1.0)
    axCmd.axhline(-config.control.maxWheelRate, color="tab:red", linestyle=":", linewidth=1.0)
    axCmd.set_ylabel("wheel cmd (rad/s)")
    axCmd.set_xlabel("time (s)")
    axCmd.grid(True, alpha=0.35)
    axCmd.legend(loc="upper right")

    if safety.status.tripped:
        tripTime = safety.status.lastUpdateTime
        for ax in axes:
            ax.axvline(tripTime, color="tab:red", alpha=0.35, linewidth=1.0)

    durationSec = times[-1] - times[0] if len(times) >= 2 else 0.0
    finalThetaError = theta[-1] - thetaRef[-1]
    summary = "  ".join(
        (
            "samples=" + str(len(records)),
            "duration=" + "{:.2f}s".format(durationSec),
            "tripped=" + str(safety.status.tripped),
            "final theta error=" + "{:+.4f} rad".format(finalThetaError),
        )
    )
    if safety.status.reasons:
        summary += "  reasons=" + ", ".join(safety.status.reasons)
    fig.text(0.01, 0.005, summary, fontsize=10, color="dimgray")

    fig.tight_layout(rect=(0.0, 0.03, 1.0, 0.96))
    fig.savefig(outputPath, dpi=160)
    print(f"plot written to  : {outputPath}")

    if showPlot:
        plt.show()
    else:
        plt.close(fig)
    return 0


def Main() -> int:
    args = ParseArgs()
    argStatus = ValidateArgs(args)
    if argStatus != 0:
        return argStatus

    config = LoadConfig(path=args.config, applyLocalOverride=False)
    dtSec = 1.0 / config.control.loopRate if args.dt_sec is None else args.dt_sec

    logger, safety = RunClosedLoopSimulation(
        config=config,
        durationSec=args.duration_sec,
        dtSec=dtSec,
        initialTiltRad=args.initial_tilt_rad,
        tiltReferenceRad=args.tilt_ref_rad,
    )
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    logger.WriteCsv(args.csv)
    print("LegoBalance ClosedLoopBalance")
    print("-" * 60)
    print(f"config          : {Path(args.config).resolve()}")
    print(f"duration        : {args.duration_sec:.2f} s")
    print(f"dt              : {dtSec:.4f} s")
    print(f"initial tilt    : {args.initial_tilt_rad:+.4f} rad")
    print(f"tilt reference  : {args.tilt_ref_rad:+.4f} rad")
    print(f"records         : {len(logger)}")
    print(f"safety tripped  : {safety.status.tripped}")
    if safety.status.reasons:
        print(f"safety reasons  : {safety.status.reasons}")
    print(f"log written to  : {args.csv}")

    return PlotSimulation(
        logger=logger,
        safety=safety,
        config=config,
        outputPath=args.output,
        showPlot=not args.no_show,
    )


if __name__ == "__main__":
    raise SystemExit(Main())
