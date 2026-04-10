"""HubSingleMotorStepResponseF.py

Single-motor velocity step-response sweep for the motor on Port F.

This script runs one staircase-style velocity experiment on one motor:

    0 -> 200 -> 0 -> 500 -> 0 -> 750 -> 0 -> 1000 -> 0 deg/s

Each plateau is held for a fixed duration so the desktop plotter can overlay
the reference speed and the measured speed. The measured angle is also logged
to show how the shaft position evolves during the sweep.

For telemetry compatibility with the desktop plotter, the script still prints:

- phase (always ``velocity`` in this script),
- segment label (``zero_start``, ``step_200``, ``zero_after_200``,
  ``step_500``, ``zero_after_500``, ``step_750``, ``zero_after_750``,
  ``step_1000``, ``zero_end``),
- time,
- angle reference and measured angle,
- speed reference and measured speed.

The angle reference is always emitted as ``nan`` because this experiment is
velocity-driven only.

Use the desktop-side plotter to visualize reference versus measurement:

    python scripts/PlotHubSingleMotorStepResponse.py

SAFETY: lift the motor or hold it before running. The shaft will move through
the full velocity sweep.
"""

# ruff: noqa: UP032

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

MOTOR_PORT = Port.F
MOTOR_PORT_LABEL = "F"

START_DELAY_MS = 1500
LOOP_PERIOD_MS = 20

VELOCITY_ZERO_HOLD_MS = 800
VELOCITY_STEP_MS = 1600
VELOCITY_STEP_LEVELS_DPS = (200, 500, 750, 1000)


def CenterPressed(hub):
    return Button.CENTER in hub.buttons.pressed()


def FormatScalar(value):
    if value is None:
        return "nan"
    return "{:+.3f}".format(value)


def EmitData(phase, segment, timeSec, angleRefDeg, angleMeasDeg, speedRefDps, speedMeasDps):
    print(
        "DATA,{},{},{:.3f},{},{},{},{}".format(
            phase,
            segment,
            timeSec,
            FormatScalar(angleRefDeg),
            FormatScalar(angleMeasDeg),
            FormatScalar(speedRefDps),
            FormatScalar(speedMeasDps),
        )
    )


def WaitUntilTick(stopwatch, nextTickMs):
    while stopwatch.time() < nextTickMs:
        wait(1)
    return nextTickMs + LOOP_PERIOD_MS


def RunVelocitySegment(hub, motor, stopwatch, nextTickMs, segment, durationMs, speedRefDps):
    segmentEndMs = stopwatch.time() + durationMs
    while stopwatch.time() < segmentEndMs:
        if CenterPressed(hub):
            return nextTickMs, False
        motor.run(speedRefDps)
        EmitData(
            "velocity",
            segment,
            stopwatch.time() / 1000.0,
            None,
            motor.angle(),
            speedRefDps,
            motor.speed(),
        )
        nextTickMs = WaitUntilTick(stopwatch, nextTickMs)
    return nextTickMs, True


def Main():
    hub = PrimeHub()
    motor = Motor(MOTOR_PORT)

    motor.reset_angle(0)

    print("============================================================")
    print(" HubSingleMotorStepResponseF")
    print(" Single-motor velocity sweep on Port {}.".format(MOTOR_PORT_LABEL))
    print(" Runs a staircase command: 0 -> 200 -> 0 -> 500 -> 0 -> 750 -> 0 -> 1000 -> 0 deg/s.")
    print(" Press the center button to stop cleanly.")
    print("============================================================")
    print(" Velocity holds : {} ms per plateau".format(VELOCITY_STEP_MS))
    print(" Zero holds     : {} ms per zero segment".format(VELOCITY_ZERO_HOLD_MS))
    print(" Loop period    : {} ms".format(LOOP_PERIOD_MS))
    print(" DATA columns:")
    print(
        " DATA,phase,segment,t_s,angle_ref_deg,angle_meas_deg,"
        "speed_ref_deg_per_sec,speed_meas_deg_per_sec"
    )
    print(" Lift the motor first. Starting in {:.1f} s.".format(START_DELAY_MS / 1000.0))
    wait(START_DELAY_MS)

    stopwatch = StopWatch()
    nextTickMs = LOOP_PERIOD_MS

    velocitySchedule = [("zero_start", VELOCITY_ZERO_HOLD_MS, 0)]
    for index, speedDps in enumerate(VELOCITY_STEP_LEVELS_DPS):
        velocitySchedule.append(("step_{}".format(speedDps), VELOCITY_STEP_MS, speedDps))
        zeroLabel = "zero_end"
        if index < len(VELOCITY_STEP_LEVELS_DPS) - 1:
            zeroLabel = "zero_after_{}".format(speedDps)
        velocitySchedule.append((zeroLabel, VELOCITY_ZERO_HOLD_MS, 0))

    ok = True
    for segment, durationMs, speedRefDps in velocitySchedule:
        nextTickMs, ok = RunVelocitySegment(
            hub,
            motor,
            stopwatch,
            nextTickMs,
            segment,
            durationMs,
            speedRefDps,
        )
        if not ok:
            break

    if not ok:
        motor.stop()
        motor.brake()
        print("Aborted during segment '{}'.".format(segment))
        return

    motor.stop()
    motor.brake()

    print("Done. final angle = {:+.1f} deg, final speed = {:+.1f} deg/s".format(
        motor.angle(),
        motor.speed(),
    ))


Main()
