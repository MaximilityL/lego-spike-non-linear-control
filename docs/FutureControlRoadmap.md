# Future Control Roadmap

This roadmap starts from the current state of the repository, where a
geometry aware robust nonlinear balance controller is already in place
and has been validated in live and buffered hub runs.

## 1. What Is Already In Place

The current control stack already includes:

- measured sign and offset conventions in `configs/Default.yaml`,
- a minimal estimator that produces `[theta, thetaDot, phi, phiDot]`,
- a geometry aware robust nonlinear balance controller
  (`NonLinearController`) with a slow outer recentering loop, a
  Lyapunov style fast inner loop, and a first order actuator lag
  inversion,
- a PID baseline controller kept behind the same factory interface
  for comparison runs,
- a safety monitor with watchdog and tilt limits,
- desktop closed loop simulation,
- package backed balance runs on the real hub through both
  `src/HubPackageBalance.py` (live telemetry) and
  `src/HubPackageBalanceBuffered.py` (post run dump for full 100 Hz
  capture),
- plotting tools for post run analysis.

That means the next work is mostly about improving controller quality,
hardware validation, and parameter identification, not building the
pipeline from scratch.

## 2. Near Term Priorities

### 2.1. Systematic hardware tuning of the nonlinear law

The geometry aware controller is implemented, but most of its tuning
still needs to be done on real hardware rather than in simulation. The
highest value tuning task is now to sweep the new gains cleanly:

- log multiple real balance runs using the buffered path so the full
  100 Hz trace is available for post run analysis,
- sweep `innerNaturalFrequency`, `innerDampingRatio`,
  `robustGain`, and `boundaryLayerWidth` systematically and record
  which combinations recover from a push,
- then carefully raise `outerPositionGain` and `outerVelocityGain`
  until the recentering loop begins to interfere with the fast
  anti fall law, and back off from there,
- compare the nonlinear and PID controllers under the same chassis,
  battery, and floor conditions.

### 2.2. Better experiment logging

The buffered telemetry path already captures every control iteration
without decimation. The next step is to make those traces easier to
compare across runs:

- save metadata with each run (gains, battery state, chassis
  configuration),
- keep a run log with controller gains and which YAML revision was
  active,
- build side by side plots for different tuning sets,
- keep both the live and buffered plots comparable so regressions are
  obvious from glance.

### 2.3. Tighter actuator identification

The current `actuatorTau = 0.25 s` is a rough estimate inferred from
a single motor step response plot, not a least squares fit on raw
telemetry, and the test was done with the wheels lifted. A cleaner
identification would:

- fit `tau` from the raw single motor step response log rather than
  reading rise times off a PNG,
- repeat the identification under load on the full robot, not with
  the wheels lifted,
- characterize how `tau` changes with operating point and battery
  state, and decide whether a single nominal value is still
  acceptable.

Because the robust term in the controller is what absorbs uncertainty
in `tau`, a better identification will also let us lower
`robustGain` and reduce the cost of the robust correction in quiet
conditions.

### 2.4. Measured chassis pitch inertia

The configured `chassis.bodyInertia` is a nominal guess. The
rectangular body fallback is documented as an estimate. A direct
measurement (for example a bifilar pendulum test) would let us remove
one of the main sources of uncertainty the robust term has to cover,
and would tighten the nominal `alphaHat` and `betaHat` values the
Lyapunov style inner loop uses.

### 2.5. Cleaner hub runtime packaging

The generated helper `LegoBalance.HubDriveSmokeRuntime` now serves
both the package backed drive smoke path and the package backed
balance path (live and buffered). Renaming or generalizing that
helper, and clarifying the regeneration workflow after every YAML
edit, would make the architecture clearer and reduce the chance of
stale hub runs.

## 3. Medium Term Control Improvements

### 3.1. Stronger state estimation

The estimator is deliberately minimal. Future improvements could
include:

- complementary fusion between accelerometer tilt and gyro integration,
- explicit gyro bias estimation,
- simple filtering on wheel rate estimates,
- robustness to wheel slip.

These changes should preserve the same `BalanceState` boundary so the
controller does not need to change.

### 3.2. Smarter control policies

Possible control upgrades include:

- gain scheduling between small angle and recovery regions (for
  example widen the boundary layer near upright and shrink it during
  recovery),
- a dedicated recovery mode for larger disturbances,
- an integral action on the tilt error to auto trim
  `targetTilt`, with anti wind up,
- exploring an extended state observer to estimate and cancel the
  lumped disturbance term $\Delta_\theta$ from the reduced model,
- online identification of `tau`, `bodyInertia`, or both.

### 3.3. Command mode experiments

The current default is wheel velocity control because it maps
directly to Pybricks `Motor.run(...)`. Another path is to evaluate
duty cycle control through `Motor.dc(...)`, which would give a more
torque like interface at the cost of losing Pybricks' built in speed
regulation. Because the new controller reasons in chassis
acceleration, switching to `Motor.dc` would change the actuator
inversion but not the nominal law: it would replace the first order
velocity lag with a different, torque like mapping.

## 4. Longer Term System Goals

Once basic balance is reliable on the new law, the next larger
features are:

- yaw and steering control,
- trajectory tracking,
- stand up and recovery routines,
- online identification of mechanical parameters, feeding back into
  the nominal reduced model,
- a more realistic simulation beyond the current toy plant, for
  example one that includes the first order wheel velocity lag and
  simple contact/friction effects.

## 5. Recommended Sequence

The highest leverage order from here is:

1. stabilize and document a repeatable hardware tuning workflow for
   the new controller, using the buffered telemetry path,
2. tighten the actuator lag and pitch inertia identification so the
   nominal `alphaHat`, `betaHat`, and `tau` stop being educated
   guesses,
3. compare the nonlinear and PID controllers on the real robot under
   the same conditions,
4. strengthen the estimator only where logs show it is limiting
   control quality,
5. explore more advanced control improvements (gain scheduling,
   disturbance observer, recovery mode) once the measurement
   pipeline is trusted.
