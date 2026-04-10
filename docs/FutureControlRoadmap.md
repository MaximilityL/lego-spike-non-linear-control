# Future Control Roadmap

This roadmap starts from the current state of the repository, where a real
balance controller already exists.

## 1. What Is Already In Place

The current control stack already includes:

- measured sign and offset conventions in `configs/Default.yaml`,
- a minimal estimator that produces `[theta, thetaDot, phi, phiDot]`,
- two balance controllers behind one factory interface,
- a safety monitor with watchdog and tilt limits,
- desktop closed-loop simulation,
- package-backed balance and drive runs on the real hub,
- plotting tools for post-run analysis.

That means the next work is mostly about improving controller quality and
hardware validation, not building the pipeline from scratch.

## 2. Near-Term Priorities

### 2.1. Systematic hardware tuning

The tanh controller is implemented, but hardware tuning is still empirical.
The next high-value task is a repeatable tuning procedure:

- log multiple real balance runs,
- compare successful and failed runs,
- tune `targetTilt`, `kTheta`, `kThetaDot`, and `sScale` systematically,
- compare the nonlinear and PID controllers under the same conditions.

### 2.2. Better experiment logging

The project already logs state and command traces. The next step is to make
those traces easier to compare across runs:

- save metadata with each run,
- keep a run log with controller gains and battery state,
- build side-by-side plots for different tuning sets.

### 2.3. Cleaner hub-runtime packaging

The generated helper file `LegoBalance.HubDriveSmokeRuntime` now serves both the
package-backed drive smoke path and the package-backed balance path. Renaming or
generalizing that helper would make the architecture clearer.

## 3. Medium-Term Control Improvements

### 3.1. Stronger state estimation

The estimator is deliberately minimal. Future improvements could include:

- complementary fusion between angle and gyro integration,
- explicit bias estimation,
- simple filtering on wheel-rate estimates,
- robustness to wheel slip.

These changes should preserve the same `BalanceState` boundary if possible.

### 3.2. Smarter control policies

Possible control upgrades include:

- gain scheduling between small-angle and recovery regions,
- a recovery mode for larger disturbances,
- a more explicitly model-based nonlinear law once hardware parameters are
  measured with confidence,
- better anti-drift logic once long-run balance is stable.

### 3.3. Command-mode experiments

The current default is wheel velocity control because it maps directly to
Pybricks `Motor.run(...)`. Another path is to evaluate duty-cycle control
through `Motor.dc(...)`, but that requires additional characterization because
the relationship to torque is only approximate.

## 4. Longer-Term System Goals

Once basic balance is reliable, the next larger features are:

- yaw and steering control,
- trajectory tracking,
- stand-up and recovery routines,
- online identification of mechanical parameters,
- more realistic simulation beyond the current toy plant.

## 5. Recommended Sequence

The highest-leverage order from here is:

1. stabilize and document repeatable hardware tuning,
2. compare nonlinear and PID controllers on the real robot,
3. improve logging and experiment bookkeeping,
4. strengthen the estimator only where logs show it is limiting control quality,
5. explore more advanced controllers after the measurement pipeline is trusted.
