# Changelog

All notable changes to this project will be documented in this file.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.7.5] - 2026-04-11

### Added
- Added a second post-run analysis pass for buffered balance logs. After the
  standard time-series figure, the desktop plotter now computes the inner-loop
  tracking error `e = theta - thetaRef`, a Lyapunov-like energy
  `V = 0.5*thetaDot^2 + k*e^2`, and the gap between the raw and safety-clamped
  wheel commands, then renders those signals as a dedicated diagnostic figure.
  The point of the new view is to answer the logic questions the base plot does
  not answer well: whether the run stayed near the intended operating region,
  whether the phase trajectory stayed compact around upright, and how often the
  controller was being saved by saturation rather than by nominal balance
  action.
- Added per-panel PNG export for the buffered plot workflow. Each run now also
  emits standalone files for `theta`, `thetaDot`, `phi`, `phiDot`, wheel
  command, tilt error, phase portrait, Lyapunov-like energy, and control
  saturation so report writing and side-by-side comparisons no longer require
  manual cropping from the combined figure.

### Changed
- Retuned the hub face eye hysteresis to require a larger wheel-angle excursion
  before the eyes commit to the left or right bucket. `effects.eyesPhiDeadbandDeg`
  was raised from `45 deg` to `50 deg`, which makes the display react more to
  sustained displacement and less to transient drift around the center region.
- Regenerated the hub runtime mirror and refreshed the committed buffered plot
  artifacts so the checked-in outputs match the new diagnostics path and the
  updated eye-deadband tuning.

## [1.7.4] - 2026-04-11

### Added
- Added an `effects` section to `configs/Default.yaml` and a matching
  `EffectsConfig` dataclass so the sound and light feedback on the buffered
  balance run is configuration-driven. Every sub feature has its own toggle
  on top of a master switch so the robot can run silent, with only the
  button LED, with only the face, or with the full personality.
- The buffered hub runtime now draws a face on the 5x5 display that is
  driven off the smooth state the controller already computes, instead of
  the raw tilt-rate or command signals that are too noisy to watch
  directly:
  - Eyes track `phi` through three buckets (left, center, right). The
    bucket logic uses a hysteretic deadband so `phi` has to cross
    `eyesPhiDeadbandDeg` outward and then rebound to 40 percent of that
    value inward before the eyes switch direction, which prevents
    chatter at the boundaries. The bucket sign is the opposite of `phi`
    so the eyes look back at where the robot rolled from instead of
    where it is rolling to.
  - Mouth reflects balance quality: smile when the rolling RMS tilt
    error is at or below `qualityGreenThresholdDeg`, frown when it is
    at or above `qualityRedThresholdDeg`, flat bar in between. Eye
    bucket changes and mouth category changes share a single redraw
    path so the face always reflects the latest bucket plus the latest
    quality category.
- The center hub button LED now mirrors the same RMS tilt error as the
  mouth, but as a continuous red-to-green gradient. The hue is computed
  from a rolling window of squared theta error samples of length
  `qualityRmsWindowSamples`, mapped linearly between the green and red
  thresholds, and quantized to 10 degree hue steps so the LED does not
  twitch on every control iteration when the error is slowly drifting.
- Added an upright chime: a short beep every time `theta` crosses zero,
  debounced by `uprightChimeMinIntervalMs` so successive crossings do not
  chatter. Pybricks `Speaker.beep` is synchronous and exposes no stop
  method on the current firmware, so the chime is implemented as a short
  blocking beep whose duration is kept under the control loop period to
  avoid disturbing the balancer.

### Changed
- Extended `scripts/GenerateHubDriveSmokeRuntime.py` and regenerated
  `LegoBalance.HubDriveSmokeRuntime` so the hub side mirror carries every
  effects field from YAML. The buffered balance runtime reads its effects
  toggles and thresholds from that mirror just like the rest of the
  control stack, so the same YAML edit reaches both the desktop loader
  and the hub.

## [1.7.3] - 2026-04-11

### Added
- Added a continuous on-hub calibration sweep for `imu.zeroOffset` or
  `imu.gyroBias`. The balancer now stays running while it mutates one IMU
  calibration field between fixed-length windows, gives each candidate a
  settle period before scoring, and ranks windows by a weighted combination
  of mean absolute tilt error and wheel-position drift relative to that
  window's starting `phi` so later tests are not biased by earlier encoder
  drift.
- The sweep search itself is now logic-driven rather than a blind fixed grid:
  it measures the baseline first, probes `+k` and `-k` in alternating
  bracket pairs, commits to a direction only after two consecutive pairs
  agree that one side beats baseline while the other does not, and then
  climbs only in that confirmed direction until the phase cap or a safety
  trip stops the run.
- Added a laptop launcher for that sweep which regenerates the hub-safe
  runtime from `configs/Default.yaml` before calling `pybricksdev`, then
  streams the hub's per-window summaries and final best-value report to the
  terminal (with optional tee-to-log support) instead of trying to postprocess
  full telemetry.

### Changed
- Retuned the shipped IMU upright reference after the new sweep identified a
  slightly better balance point on the current chassis. `imu.zeroOffset` was
  moved from `+1.133 deg` to `+0.933 deg`, reducing the steady bias the
  controller has to fight around the nominal upright.
- Regenerated the package-backed hub runtime and refreshed the buffered
  balance diagnostic plot so the committed runtime mirror and the checked-in
  visual artifact match the new default calibration.

## [1.7.2] - 2026-04-11

### Changed
- Rewrote docs/NonLinearControllerReview.md from scratch so it reviews the
  geometry aware robust nonlinear controller that is actually running, instead
  of the composite variable tanh law that was removed in 1.7.0. The review now
  derives the reduced sagittal pendulum plant, explains the first order
  actuator lag inversion, builds the Lyapunov style nominal law and the smooth
  sliding mode robust correction from first principles, walks through the slow
  outer recentering loop and the algebraic loop fix, maps every equation to
  the corresponding source line, performs a linearized stability check and a
  region of attraction estimate against the current config, and lists the
  honest open questions the controller does not try to resolve.
- Updated docs/ImplementationReportGuide.md so the equations worth citing are
  the reduced tilt dynamics, the actuator model, the outer/inner loop
  equations, and the command mapping instead of the legacy composite variable
  expression. The report-friendly design decisions and limitations sections
  now reflect why exact feedback linearization was deliberately not chosen and
  that the pitch inertia and actuator time constant are still nominal values.
- Refreshed docs/FutureControlRoadmap.md so it starts from the current
  controller rather than the old tanh law. Near term priorities now target
  sweeping the new inner/outer loop gains on real hardware through the
  buffered telemetry path, tightening the actuator lag identification on the
  loaded two motor chassis, and measuring the pitch inertia directly so the
  robust term can shrink.
- Updated docs/ArchitectureOverview.md so the controller selection section
  describes the geometry aware law (with `tanh` kept as a legacy alias) and
  the hub entrypoint section mentions both the live telemetry path and the
  new buffered post run variant.
- Rewrote the "How It Is Used" section of docs/MotorActuatorLagIdentification.md
  so it reflects how the geometry aware controller consumes `tau`: the
  inversion turns a desired chassis acceleration into a wheel velocity command
  and the outer loop estimates the previous step acceleration through the
  same model to break the reference derivative's algebraic loop.

## [1.7.1] - 2026-04-11

### Added
- New buffered telemetry path for the balance run. The control loop is no
  longer coupled to the BLE print rate: a new hub program captures every
  control iteration into preallocated packed float32 buffers (via
  ustruct.pack_into on bytearrays, since Pybricks MicroPython does not ship
  the array module), then dumps every sample over BLE once the run ends.
  This removes the per iteration print blocking that previously forced
  telemetry decimation to keep the 100 Hz loop honest, and gives the post
  run analysis the full unthinned state and command trace.
- New laptop side plotter that regenerates the hub runtime from
  configs/Default.yaml, launches the buffered hub program through
  pybricksdev, collects the post run DATA rows, and produces a diagnostic
  PNG in the same layout as the live variant so buffered and live runs
  remain visually comparable.

### Changed
- Shortened the default run duration on the buffered path to 25 s so the
  full 100 Hz capture fits in hub RAM without decimating the sample stream.
  The live run path is untouched.
- Retuned the balance base point after the buffered runs exposed a residual
  static bias: the IMU zero offset was recalibrated around the current
  mechanical upright, targetTilt was brought back to zero so the outer loop
  recentering term is the only authority over the steady lean, and the tilt
  rate filter was raised to soften the inner loop response to gyro noise.
  The regenerated hub runtime mirror is bumped to match.

## [1.7.0] - 2026-04-11

### Changed
- Replaced the legacy tanh composite-variable balancer with a geometry-aware
  robust nonlinear controller. The new law separates a slow wheel-position
  recentering reference loop from the fast tilt stabilizer, derives the
  reduced tilt model from chassis mass/CoM/inertia, and maps desired chassis
  acceleration through the identified first-order actuator lag before issuing
  wheel-speed commands.
- Extended the shared chassis/controller config and the generated hub runtime
  with the geometry and tuning fields the new law needs, including the body
  dimensions used for the inertia fallback, the inner/outer loop gains, the
  smooth robust-term settings, and the bounded reference-tilt offset.
- Updated the hub balance banner, the nonlinear-controller design/review
  docs, and the refreshed live-balance plot to reflect the new control
  structure instead of the old tanh gain set.

### Tests
- Reworked controller/config coverage around the new logic: geometry-derived
  dynamics, inertia fallback behavior, outer-loop saturation, actuator
  mapping, filter/reset state handling, and legacy alias compatibility.

## [1.6.0] - 2026-04-10

### Changed
- Marked this as the first somewhat working integrated version of the current
  robot build. The shared software stack now matches the real drivetrain
  layout instead of assuming a simpler two-motor bench configuration: left
  wheel on `B`, primary right-side motor on `F`, and an auxiliary right-side
  axle motor on `D`.
- Extended the shared motor config with `rightAuxPort` and
  `rightAuxEncoderSign`, and tightened validation so port names are normalized
  and duplicate assignments are rejected. The third motor is now part of the
  explicit hardware contract instead of an informal wiring detail.
- Updated the package-backed hub balance and drive-smoke entrypoints so the
  D-port motor is reset, stopped, and commanded together with the existing
  right-side motor, and so the implemented right-wheel measurement is built
  from the averaged signed encoders of `F` and `D`. The controller and
  estimator still run on the same two-wheel state `[theta, thetaDot, phi,
  phiDot]`, but that state now reflects the real axle mechanics more closely.
- Aligned the standalone hub bring-up, encoder, and motor smoke scripts with
  the same B/F/D wiring and telemetry assumptions, so quick bench checks and
  package-backed runs no longer disagree about which motors belong to the
  drive axle.

### Tests
- Expanded config and generated-runtime parity coverage for the auxiliary
  axle-motor fields and reran the desktop validation suite on the integrated
  three-motor configuration.

## [1.5.0] - 2026-04-10

### Added
- Added `docs/MotorActuatorLagIdentification.md`, which records the Port `F`
  single-motor velocity-step experiment, the observed first-order-like speed
  response, and the practical inference `tau_eff ≈ 0.20 s` used for control
  design.

### Changed
- Reworked `hub/HubSingleMotorStepResponseF.py` into a velocity-only sweep with
  explicit zero-speed reset segments:
  `0 -> 200 -> 0 -> 500 -> 0 -> 750 -> 0 -> 1000 -> 0 deg/s`. This makes each
  step easier to inspect independently and gives cleaner visual separation in
  the measured-speed trace.
- Extended the nonlinear controller config with `controller.actuatorTau` and
  propagated it through the desktop loader, generated hub-safe runtime, and the
  parity tests so actuator-lag tuning is part of the normal shared config path
  instead of a hidden controller constant.
- Updated `NonLinearController` to account for command lag in the wheel-state
  feedback terms. The controller now computes a provisional command, predicts
  `phi` and `phiDot` one outer-loop step ahead under a first-order actuator
  model parameterised by `actuatorTau`, and then recomputes the tanh law using
  those predicted wheel states.
- Adjusted the single-motor desktop plotter and top-level docs to match the
  velocity-only sweep and to point report writing toward the new actuator-lag
  identification note.

### Tests
- Added config validation and controller behavior checks for the new
  `actuatorTau` parameter, including parity coverage for the generated
  hub-safe runtime and nonlinear-controller tests that prove the lag
  compensation is active only when the wheel-state terms are active.

## [1.4.2] - 2026-04-10

### Fixed
- Reworked `hub/HubSingleMotorStepResponseF.py` to use a more conservative
  MicroPython-safe string formatting style after the first `1.4.1` revision
  failed during the `mpy-cross` compile step inside `pybricksdev run`. The
  one-motor step-response experiment now compiles on the SPIKE toolchain
  instead of dying before upload.

## [1.4.1] - 2026-04-10

### Added
- Added a single-motor step-response workflow for the currently wired Port `F`
  motor. `hub/HubSingleMotorStepResponseF.py` now runs both a velocity step
  and a position step on one motor and prints telemetry rows containing the
  requested reference-versus-measurement traces for angle and speed.
- Added `scripts/PlotHubSingleMotorStepResponse.py` as the desktop-side
  companion that launches the hub script through `pybricksdev` or reads a
  saved log, then overlays reference and measured angle/speed so one-motor
  actuator behavior can be inspected quickly before or alongside full-balance
  experiments.

### Changed
- Switched the shipped default balance profile back to the nonlinear/tanh path
  and retuned the current YAML/runtime defaults around the latest hardware
  iteration: a more negative `targetTilt`, stronger tilt authority, stronger
  wheel-state centering, a wider `sScale`, and zeroed deadband/filter terms so
  the nonlinear controller starts from the current real-robot operating point
  rather than the previous PID branch defaults.
- Reduced the default drive-smoke telemetry print cadence from every `20`
  iterations to every `50` iterations so the shared hub runs spend less time
  printing over Bluetooth while still leaving enough data for post-run plots.

### Tests
- Added parser/span tests for the new single-motor step-response plotter so the
  reference-versus-measurement telemetry contract is checked on desktop before
  running it on the hub.

## [1.4.0] - 2026-04-10

### Added
- Added a config-selected balance-controller path so the real hub runner, the
  desktop closed-loop simulation, and the plotting scripts all instantiate the
  same balancing law from YAML instead of hard-coding the tanh controller in
  each entrypoint.
- Added `PidController` as a discrete velocity-command balance law that follows
  the referenced LEGO pattern closely inside the repo's controller boundary:
  tilt error around `targetTilt`, a simple accumulated integral term, a
  difference-based derivative term, and a wheel-position correction term.

### Changed
- Reframed this branch's balancing setup around controller interchangeability
  rather than a single control law. `controller.algorithm` now selects between
  `pid` and the existing `tanh`/`nonlinear` path while preserving the same
  robot, the same telemetry columns, and the same post-run plots.
- Made the balance reference part of the shared config contract end to end.
  The desktop plot/simulation path and the package-backed hub path now both
  treat `control.targetTilt` as the operating-point reference instead of
  shifting the state in an entrypoint-specific way.
- Switched the generated hub-safe runtime, typed config defaults, and parity
  tests to carry the same controller-selection fields and PID parameters as
  the desktop YAML loader, so tuning changes propagate through the whole stack
  without silent drift.

### Fixed
- Removed a hub-runtime startup bug in the package-backed balance script where
  the banner relied on `type(...).__name__`, which is not dependable on the
  SPIKE MicroPython runtime.
- Added a quiet-zone to the PID tilt path by honoring `thetaDeadband`, so a
  near-upright P-only experiment does not immediately turn small IMU jitter
  into wheel twitching.
- Closed a configuration-consistency gap where dataclass defaults could drift
  away from `configs/Default.yaml` and cause helper/runtime paths that bypass
  YAML to behave differently from the main balance flow.

### Tests
- Added dedicated PID tests covering factory selection, command symmetry,
  proportional/integral/derivative behavior, saturation, reset semantics, the
  wheel-position term, and the new deadband behavior near upright.
- Updated the config, generated-runtime, and end-to-end pipeline tests so they
  validate the config-selected controller path rather than assuming the
  nonlinear balance controller is always active.

## [1.3.2] - 2026-04-10

### Changed
- Reframed the balancing logic around a model-light tanh composite variable
  instead of a model-dependent gravity-cancellation law. The controller now
  treats tilt and tilt-rate as the primary stabilizing signals, keeps wheel
  position and wheel-rate as optional weak drift terms, and exposes the
  nonlinear knee explicitly through `sScale`.
- Made the balance-point logic explicit. `targetTilt` now represents the
  actual upright operating point tracked by the shared controller, rather than
  being partially applied in the hub script and partially implied by the IMU
  calibration.
- Tuned the hardware-facing defaults toward a calmer real-hub experiment loop:
  `100 Hz` balance timing, stronger tilt authority, softer and more filtered
  tilt-rate damping, wider quiet zones near upright, and lower default
  telemetry pressure over Bluetooth.

### Fixed
- Removed a hub-runtime logic bug where the package-backed balance script
  effectively applied `targetTilt` twice before sending the state into the
  controller, biasing the operating point during real balancing runs.
- Replaced the old "work then sleep" timing behavior in the package-backed
  balance loop with a deadline-based cadence so the real hub loop better
  matches the configured control rate under logging and BLE jitter.
- Corrected the package-backed and standalone drive-smoke paths so they follow
  the configured tilt axis and current verified hardware mapping instead of
  silently assuming the older pitch-axis bring-up layout.
- Hardened the estimator/safety boundary: measurement validity now propagates
  through `StateEstimator`, and `SafetyMonitor` now trips on non-finite state
  or command values instead of letting bad telemetry leak into the motor path.

### Tests
- Expanded the controller tests around the new tanh law, target-tilt handling,
  filter state, and config sensitivity so the release checks the implemented
  balancing logic rather than only the controller interface shape.
- Updated config, safety, estimator, and generated-runtime parity tests so the
  desktop loader, generated hub runtime, and real hub entrypoints all agree on
  the same current control assumptions.

## [1.2.2] - 2026-04-08

### Added
- Controler Config to `config/Default.yaml`.

## [1.2.1] - 2026-04-08

### Changed
- Aligned the standalone `hub/HubMain.py` IMU zero-offset with the current
  hardware calibration so the direct hub telemetry/readout path matches the
  latest measured upright reference instead of reporting against the older
  `-60 deg` offset.
- Kept this as a small follow-up release on top of `1.2.0`: the package-backed
  real balance path remains runnable and observable, while the standalone
  `HubMain.py` sensor bring-up path now reflects the same current calibration
  intent used during ongoing balance tuning.

## [1.2.0] - 2026-04-08

### Added
- Added a real package-backed hub balancing entrypoint in
  `src/HubPackageBalance.py` that runs the shared `StateEstimator`,
  `NonLinearController`, and `SafetyMonitor` on the SPIKE hub instead of the
  desktop mock plant.
- Added a laptop-side post-run telemetry plotter for the real balance run and
  a live plotting wrapper that regenerates the hub-safe runtime config from
  YAML, launches the hub program, and streams real telemetry into live plots
  of reference vs tilt, the implemented state, and wheel command.
- Added a desktop closed-loop balance plotter so the same shared controller
  path can be exercised and visualized off-hardware when needed.

### Changed
- Extended the control config with a YAML-driven `targetTilt` reference so the
  package-backed balance run can request an explicit tilt target without
  moving the controller boundary or hard-coding the reference inside the hub
  script.
- Made the shared `NonLinearController` import path friendlier to the
  package-backed Pybricks runtime by removing the desktop-only `__future__`
  import and other avoidable runtime dependencies that blocked hub execution.
- Reached the first real hardware integration milestone for balancing: the
  package-backed hub path now runs the real estimator, controller, safety
  monitor, telemetry, and plotting flow end to end, but the current control
  law is not yet physically stabilizing the robot on hardware. This release is
  about making the real loop observable and tunable, not claiming closed-loop
  stability.

## [1.1.1] - 2026-04-08

### Changed
- Replaced the balancing-controller placeholder with a real sagittal
  balancing law in `NonLinearController`: a velocity-mode,
  sliding-mode-inspired outer-loop controller acting on the implemented state
  `[theta, thetaDot, phi, phiDot]`.
- Defined the controller logic around a composite sliding variable
  `sigma = thetaDot + lambdaTheta*theta + lambdaPhiDot*phiDot + lambdaPhi*phi`
  and a bounded command that combines smooth state feedback with a
  boundary-layer saturation term instead of a discontinuous `sign(...)`
  switch, reducing jitter on sampled LEGO hardware.
- Locked the controller contract around practical runtime behavior: invalid
  states stop immediately, valid commands preserve timestamps, left and right
  wheel commands remain symmetric, positive forward lean commands positive
  forward wheel motion, and controller output is intentionally clamped to
  `control.maxWheelRate` before reaching the safety monitor.
- Upgraded the controller tests from placeholder checks to real logic checks,
  covering invalid-state stop behavior, symmetry, output saturation, reset
  semantics, placeholder status, and forward-lean sign sanity.
- Brought the typed config defaults, shipped YAML, generated
  `HubDriveSmokeRuntime`, and related tests back into agreement on the
  hardware-validated chassis values, wheel-rate limit, drive test speed, and
  pre-balancing motion gate so the full desktop test suite reflects the
  current repo truth.

## [1.1.0] - 2026-04-08

### Added
- Added `NonLinearController` as the canonical balancing-controller module and
  preserved `LyapunovController` as a compatibility alias so existing imports
  still work during the transition.
- Added `docs/NonLinearControllerDesignGuide.md` as a controller handoff
  document describing the control boundary, the expected state estimate,
  command conventions, and how the controller is meant to interact with the
  estimator, safety monitor, and motor path.

### Changed
- Reframed the balancing-controller API around the implemented runtime logic:
  `Measurement -> StateEstimator.Update -> NonLinearController.Compute ->
  SafetyMonitor.Check -> motor command`.
- Updated the architecture, roadmap, test-strategy, hardware, Pybricks, and
  top-level docs so they describe the actual implemented state
  `[theta, thetaDot, phi, phiDot]`, the derived `p` / `pDot` view, and the
  controller/chassis sign conventions instead of leaving the controller as an
  abstract future placeholder.
- Switched the desktop closed-loop simulation and controller-facing tests to
  the new `NonLinearController` name while keeping behavior unchanged: the
  controller remains an explicit placeholder that returns zero commands until a
  real balancing law is implemented.

## [1.0.4] - 2026-04-08

### Changed
- Made the package-backed hub smoke flow configuration-driven end to end. The
  package runtime generator now copies the values from `configs/Default.yaml`
  directly into `LegoBalance.HubDriveSmokeRuntime`, instead of rejecting local
  edits that differ from a previously hard-coded smoke-test profile.
- Moved the package smoke timing knobs into the `drive` config section so the
  YAML now controls the hub loop period, telemetry cadence, stop leg duration,
  forward/backward leg duration, test speed, and motion gate without editing
  `src/HubPackageDriveSmoke.py`.
- Updated the typed config defaults and parity tests to follow the current YAML
  values, keeping the desktop loader, generated hub runtime, and package smoke
  path aligned around the same source of truth.
- Increased the shared drive-smoke plot typography and figure size so the popup
  plots are easier to read for both the standalone and package-backed smoke
  plotters.

## [1.0.3] - 2026-04-08

### Added
- Added a package-backed hardware smoke path that exercises the shared
  `LegoBalance.StateEstimator`, `DriveCommandController`, and `SafetyMonitor` modules on
  the SPIKE hub instead of duplicating that logic inside a standalone hub script.
- Added a laptop plotter for the package-backed smoke run that reuses the existing
  drive-smoke telemetry parser while loading the plotted drive gate from the desktop
  `LegoBalance` config.
- Added a hub-safe default config helper and parity tests that keep its hardware-validated
  signs/constants aligned with the desktop config while proving the shared estimator,
  controller, and safety monitor accept it.
- Added a generator for the hub-safe drive-smoke config so `configs/Default.yaml` remains
  the source of truth even though the SPIKE hub cannot parse YAML directly.

### Changed
- Kept the standalone `hub/HubDriveSmoke.py` as the hardware-verified reference path,
  while documenting the new package-backed path as the deliberate multi-file Pybricks
  exception for testing reusable package logic on the real robot.
- Generalized the drive-smoke plotting title so both standalone and package-backed runs
  can share the same diagnostic plot logic without changing the telemetry contract.
- Removed desktop-only dataclass, enum, and abstract-base-class runtime dependencies from
  the shared state/controller boundary modules so Pybricks can import the same estimator
  and drive controller used on the desktop.
- Made the package-backed plotter regenerate `LegoBalance.HubDriveSmokeRuntime` from
  `configs/Default.yaml` before live upload, and fail early if the config no longer
  matches the hardware-verified smoke constants.
- Kept the hub import path MicroPython-safe by removing unsupported `__future__`,
  `property`, `staticmethod`, relative import, and `math` dependencies from the modules
  uploaded by `src/HubPackageDriveSmoke.py`.

## [1.0.2] - 2026-04-07

### Changed
- Promoted the hardware-verified drive smoke convention into the desktop defaults: left motor on port `B`, right motor on port `A`, `motors.forwardSign = -1`, `1000 deg/s` as `17.4532925199 rad/s`, and a `50 deg` pre-balancing drive gate.
- Locked the package contract around the implemented estimator state `x = [theta, thetaDot, phi, phiDot]`, where `phi` and `phiDot` are mean wheel rotation states and the linear `p` / `pDot` view is derived only when a wheel radius is needed.
- Defined `ControlOutput` in the controller/chassis frame: positive velocity means forward wheel-base motion and increasing `phi`; raw motor sign flips remain at the hardware adapter or hub-script boundary.
- Updated mock measurement conversion so desktop simulations and smoke flows undo the hardware signs before feeding `StateEstimator`, preserving the same state direction as the real hub run.
- Aligned `HubMain.py`, `HubDriveSmoke.py`, `PlotHubDriveSmoke.py`, config docs, and tests with the same hardware-validated signs, limits, and state notation.

### Added
- Added `DriveCommandController` as the non-balancing forward / backward / stop command path used to exercise the estimator + safety + motor pipeline before the upright controller is implemented.
- Added desktop and hub drive-smoke flows plus a post-run diagnostic plot for `[theta, thetaDot, phi, phiDot]`, wheel command, and drive-gate status.
- Added pre-balancing flow tests that prove forward commands produce increasing `phi`, drive-gate violations stop motion without tripping the absolute safety monitor, and the future controller boundary remains shape-compatible.

## [1.0.1] - 2026-04-07

### Changed
- Updated the state estimator to apply `imu.tiltSign`, `imu.zeroOffset`, and `imu.gyroBias` so tilt and tilt rate are corrected by configuration.
- Wheel encoder measurements now use `motors.leftEncoderSign` and `motors.rightEncoderSign`, average the signed motor readings, and convert from wheel angle to linear forward position/velocity using `chassis.wheelRadius` and `motors.forwardSign`.
- Added configuration validation for `motors.forwardSign`, `motors.leftEncoderSign`, and `motors.rightEncoderSign` values.
- Clarified the balance state and controller documentation to use the `[theta, thetaDot, p, pDot]` state representation.

## [1.0.0] - 2026-04-07

### Added
- Initial scaffold for the LEGO SPIKE inverted pendulum project targeting Pybricks.
- Top level project files: `README.md`, `VERSION`, `CHANGELOG.md`, `.gitignore`, `pyproject.toml`.
- Configuration system with a default YAML config under `configs/Default.yaml` and a `RobotConfig` loader.
- Desktop side Python package `LegoBalance` containing modular interfaces:
  - `HubInterface`, `MotorInterface`, `ImuInterface` abstract base classes.
  - `RobotConfig` dataclass and loader.
  - `DataLogger` for in memory and CSV friendly logging.
  - `SafetyMonitor` with tilt, current, and watchdog checks.
  - `StateEstimator` complementary filter scaffold.
  - `BalanceState` dataclass for the future control vector.
  - `ControlInterfaces` and `ControllerBase`.
  - `LyapunovController` placeholder with documented control objective.
  - `ConnectionDiagnostics` and `HardwareTestRunner` skeletons.
  - `Saturation` and `Units` utilities.
  - Mock hub, mock motor, and mock IMU adapters used by tests and the simulation stub.
- Hub side Pybricks programs in `hub/`:
  - `HubMain.py` minimal hub bring up program.
  - `HubMotorTest.py` safe motor smoke test.
  - `HubImuTest.py` IMU streaming program.
  - `HubEncoderTest.py` encoder echo program.
- Diagnostics and bootstrap scripts in `scripts/`:
  - `BootstrapEnv.sh` desktop environment setup helper.
  - `DetectHub.py` USB and Bluetooth presence check helper.
  - `RunDiagnostics.py` desktop side connection diagnostics runner.
- Examples in `examples/`:
  - `ClosedLoopSimulation.py` simulation stub that exercises the controller API without hardware.
  - `MotorSmokeTest.py` desktop side dry run of the motor smoke routine using mocks.
  - `ReadSensors.py` mock sensor read example.
- Test suite in `tests/` covering pure Python logic with `pytest`:
  - Config loading, saturation, units, state estimator shapes, controller interface,
    safety monitor, balance state, and connection diagnostics with mocks.
- Documentation in `docs/`:
  - `ArchitectureOverview.md`, `HardwareAssumptions.md`, `FutureControlRoadmap.md`,
    `TestStrategy.md`, `PybricksNotes.md`.

### Notes
- This release is a scaffold. The Lyapunov controller is intentionally a placeholder.
- No hardware behavior is claimed beyond what the listed Pybricks APIs document.
