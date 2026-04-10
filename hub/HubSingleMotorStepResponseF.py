"""HubSingleMotorStepResponseF.py

Single-motor step-response experiment for the motor on Port F.

This script runs two short experiments on one motor:

1. a velocity step, where the reference speed jumps from 0 to a fixed value
   and back to 0,
2. a position step, where the reference angle jumps from 0 to a fixed target
   and then returns to 0.

For each loop it prints a ``DATA,...`` telemetry row containing:

- experiment phase (``velocity`` or ``position``),
- segment label (``pre``, ``step``, ``post``, ``return``),
- time,
- angle reference and measured angle,
- speed reference and measured speed.

Use the desktop-side plotter to visualize reference versus measurement:

    python scripts/PlotHubSingleMotorStepResponse.py

SAFETY: lift the motor or hold the robot before running. The velocity step and
the position step both move the shaft.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

MOTOR_PORT = Port.F
MOTOR_PORT_LABEL = "F"

START_DELAY_MS = 1500
LOOP_PERIOD_MS = 20

VELOCITY_PRE_MS = 600
VELOCITY_STEP_MS = 1800
VELOCITY_POST_MS = 1200
VELOCITY_STEP_DPS = 360

POSITION_PRE_MS = 600
POSITION_STEP_MS = 1800
POSITION_RETURN_MS = 1800
POSITION_STEP_DEG = 180


def CenterPressed(hub):
    return Button.CENTER in hub.buttons.pressed()


def FormatScalar(value):
    if value is None:
        return "nan"
    return f"{value:+.3f}"


def EmitData(phase, segment, timeSec, angleRefDeg, angleMeasDeg, speedRefDps, speedMeasDps):
    print(
        f"DATA,{phase},{segment},{timeSec:.3f},{FormatScalar(angleRefDeg)},"
        f"{FormatScalar(angleMeasDeg)},{FormatScalar(speedRefDps)},"
        f"{FormatScalar(speedMeasDps)}"
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


def RunPositionSegment(hub, motor, stopwatch, nextTickMs, segment, durationMs, angleRefDeg):
    segmentEndMs = stopwatch.time() + durationMs
    while stopwatch.time() < segmentEndMs:
        if CenterPressed(hub):
            return nextTickMs, False
        motor.track_target(angleRefDeg)
        EmitData(
            "position",
            segment,
            stopwatch.time() / 1000.0,
            angleRefDeg,
            motor.angle(),
            None,
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
    print(f" Single-motor step response on Port {MOTOR_PORT_LABEL}.")
    print(" Runs both a velocity step and a position step.")
    print(" Press the center button to stop cleanly.")
    print("============================================================")
    print(f" Velocity step  : 0 -> {VELOCITY_STEP_DPS} deg/s -> 0")
    print(f" Position step  : 0 -> {POSITION_STEP_DEG} deg -> 0")
    print(f" Loop period    : {LOOP_PERIOD_MS} ms")
    print(" DATA columns:")
    print(
        " DATA,phase,segment,t_s,angle_ref_deg,angle_meas_deg,"
        "speed_ref_deg_per_sec,speed_meas_deg_per_sec"
    )
    print(f" Lift the motor first. Starting in {START_DELAY_MS / 1000.0:.1f} s.")
    wait(START_DELAY_MS)

    stopwatch = StopWatch()
    nextTickMs = LOOP_PERIOD_MS

    nextTickMs, ok = RunVelocitySegment(
        hub,
        motor,
        stopwatch,
        nextTickMs,
        "pre",
        VELOCITY_PRE_MS,
        0,
    )
    if not ok:
        motor.stop()
        motor.brake()
        print("Aborted during velocity pre-step.")
        return

    nextTickMs, ok = RunVelocitySegment(
        hub,
        motor,
        stopwatch,
        nextTickMs,
        "step",
        VELOCITY_STEP_MS,
        VELOCITY_STEP_DPS,
    )
    if not ok:
        motor.stop()
        motor.brake()
        print("Aborted during velocity step.")
        return

    nextTickMs, ok = RunVelocitySegment(
        hub,
        motor,
        stopwatch,
        nextTickMs,
        "post",
        VELOCITY_POST_MS,
        0,
    )
    if not ok:
        motor.stop()
        motor.brake()
        print("Aborted during velocity post-step.")
        return

    motor.stop()
    wait(300)
    motor.reset_angle(0)

    nextTickMs, ok = RunPositionSegment(
        hub,
        motor,
        stopwatch,
        nextTickMs,
        "pre",
        POSITION_PRE_MS,
        0,
    )
    if not ok:
        motor.stop()
        motor.brake()
        print("Aborted during position pre-step.")
        return

    nextTickMs, ok = RunPositionSegment(
        hub,
        motor,
        stopwatch,
        nextTickMs,
        "step",
        POSITION_STEP_MS,
        POSITION_STEP_DEG,
    )
    if not ok:
        motor.stop()
        motor.brake()
        print("Aborted during position step.")
        return

    nextTickMs, ok = RunPositionSegment(
        hub,
        motor,
        stopwatch,
        nextTickMs,
        "return",
        POSITION_RETURN_MS,
        0,
    )
    if not ok:
        motor.stop()
        motor.brake()
        print("Aborted during position return.")
        return

    motor.stop()
    motor.brake()

    print(f"Done. final angle = {motor.angle():+.1f} deg, final speed = {motor.speed():+.1f} deg/s")


Main()
