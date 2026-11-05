# detect2: IMU detector, synthetic data and implementation plan

Implement the detector in **Consumer INS Coupled with Carrier Phase Measurements
for GNSS Spoofing Detection**, arXiv:2502.03870v1, inside this directory.

Reference: [local paper](../../2502.03870v1.pdf),
[version 1 online](https://arxiv.org/html/2502.03870v1).

Status: geometry, synthetic observations, the windowed statistical detector and
short-window IMU displacement estimation are implemented. IMU mode uses specific
force and supplied attitude, removes gravity, integrates acceleration and adds
the rotating antenna offset. Predictions use no truth displacement or initial
velocity. Oracle mode remains available as a separate reference. Controlled
accelerometer, supplied-attitude and timing sensitivity tests are available.
An offline recorded PX4 attitude/IMU adapter and July-flight inspection are
available; measurement-clock verification, measured noise validation, another
flight and live integration remain later work.

Current priority: validate our own IMU integration. The additional motion-quality
check propagates configured accelerometer noise into displacement uncertainty.
A PX4 fused-displacement input remains deferred.

For another computer, see [the checkout/report handover](HANDOVER.md).
The next hardware review is [the CubeOrange PPS input research](PPS_INPUT_RESEARCH.md).

For the drone, follow the [short run-and-fly guide](FLIGHT_QUICKSTART.md), or see
the [detailed runtime recording guide](acquisition/RUNTIME.md):

```bash
cd /home/nss/SpoofingDetection
./start_recording --timesync
```

It verifies/configures the receiver at 20 Hz/2 Mbaud and starts native PX4 logging
with a 128 KiB runtime buffer before recording. Stop with Ctrl+C; if the aircraft
is armed, native collection waits until disarm. No startup-file or receiver
flash/BBR changes are made. The accepted native bench rate is 189.6 Hz.
Pi/PX4 TIMESYNC estimation is implemented and bench-tested; see the
[Stage 5A timing review](results/acquisition_stage5a/timesync_20261105T142326Z/report.html).
The flag records clock exchanges and produces offset/RTT plots and bounded
per-segment mappings. GNSS epoch alignment and independently verified timing
accuracy remain later steps. Without the flag, recording stays passive.

The intended real-data path uses **attitude supplied by PX4** and our own
accelerometer integration to estimate antenna displacement. The synthetic
quaternions currently stand in for that supplied attitude. A custom gyro-based
attitude estimator is an optional experiment, not a prerequisite or the next
required stage. See [the PX4 attitude input plan](IMU_ESTIMATION.md#px4-attitude-input-plan).

## Run the IMU-displacement detector

For the first real-data inspection, open the
[July flight HTML report](results/recorded_july07/report.html), or see the
[recorded-data method and reproduction command](RECORDED_FLIGHT.md). It uses
the confirmed 2 cm forward antenna offset and supplied PX4 attitude. ULog GNSS
position is used only for the user-authorized fixed satellite-geometry origin.
Estimated timing and conditional quality checks do not establish a conclusive
field detector result.

```bash
python3 -m SpoofingDetection.detect2.replay --imu
```

Open the [IMU detector report](results/imu/report.html),
[interactive motion/oracle comparison](results/imu/motion_report.html) or
[all figures as PDF](results/imu/detector_results.pdf). The comparison shows raw
and quadratic-removed displacement, motion-induced phase, gamma and decisions.
Truth and labels are loaded only after predictions have been saved.

For prediction without any truth access:

```bash
python3 -m SpoofingDetection.detect2.replay --imu --no-evaluate \
  --output-dir /tmp/detect2-imu-inputs-only
```

For the existing zero-phase-noise dataset, which retains unknown slow trends:

```bash
python3 -m SpoofingDetection.detect2.replay --imu \
  --data-dir SpoofingDetection/detect2/synthetic/data/carrier_clean \
  --output-dir SpoofingDetection/detect2/results/imu_clean
```

Open its [detector report](results/imu_clean/report.html) or
[motion comparison](results/imu_clean/motion_report.html).
[IMU_ESTIMATION.md](IMU_ESTIMATION.md) documents integration, initialization,
timing and assumptions. These datasets have ideal IMU measurements and exact
supplied attitude; the noisy dataset adds carrier noise only.

## Test our integration with input errors

```bash
python3 -m SpoofingDetection.detect2.imu_robustness
```

Open the [gated IMU sensitivity report](results/imu_robustness_gated/report.html) or
[all sensitivity figures as PDF](results/imu_robustness_gated/imu_robustness.pdf).
Twenty conditions vary accelerometer noise, body-frame bias, supplied-attitude
error and IMU clock offset, then combine them once. Each uses identical carrier
measurements, detector settings and interior windows. Separate condition reports
show raw/observable displacement, motion phase, gamma and decisions. The actual
perturbed IMU inputs are exported, and all predictions precede truth evaluation.

These are engineering stress settings with one seed, not a calibrated hardware
model. See [IMU_ROBUSTNESS.md](IMU_ROBUSTNESS.md) for exact levels, pairing,
timing conventions and remaining work. Ideal baseline outputs remain separate.
The [original ungated sweep](results/imu_robustness/report.html) is preserved.
It exposed false classifications from noise-induced motion in weak controls.
The new noise check requires motion reliably above its propagated uncertainty,
and shows the resulting loss or retention of conclusive coverage.

```bash
python3 -m SpoofingDetection.detect2.imu_noise_validation
```

Open the [paired 10-seed comparison](results/imu_noise_validation/report.html).
It holds carrier observations fixed and compares the same fits before/after
the extra quality check. See [IMU_UNCERTAINTY.md](IMU_UNCERTAINTY.md) for the
covariance derivation, conservative bound, assumptions and saved diagnostics.

## Run the oracle-displacement detector

```bash
python3 -m SpoofingDetection.detect2.replay --oracle
```

Open the [noisy-data detector report](results/oracle/report.html) or
[all detector figures as PDF](results/oracle/detector_results.pdf). The interactive
report selects motion, input case, window and signal. It shows raw/fitted phase,
phase after the candidate's estimated trend removal, and residuals. Static plots
also show the two costs, gamma, predictions and separately loaded labels.

For a noiseless validation that still contains unknown slow trends:

```bash
python3 -m SpoofingDetection.detect2.synthetic.generate_carrier \
  --noise-sigma-cycles 0 --no-plots \
  --output-dir SpoofingDetection/detect2/synthetic/data/carrier_clean
python3 -m SpoofingDetection.detect2.replay --oracle \
  --data-dir SpoofingDetection/detect2/synthetic/data/carrier_clean \
  --output-dir SpoofingDetection/detect2/results/oracle_clean
```

Open the [noiseless detector report](results/oracle_clean/report.html).
Read [DETECTOR.md](DETECTOR.md) for the equation mapping, quality policy,
optimization limits, covariance handling and input/truth boundary.

The first detector requires sufficiently excited full-rank 3D motion *after*
quadratic removal. It conservatively leaves stationary, constant-velocity,
one-axis and planar/coning controls inconclusive. The three-axis, combined and
nonplanar rotation-only controls exercise the complete fit. This restriction
is an implementation policy, not a claim that all planar motion is undetectable.

## Generate the paired carrier observations

```bash
python3 -m SpoofingDetection.detect2.synthetic.generate_carrier
```

Open the [carrier comparison report](synthetic/data/carrier_phase/report.html) or
[all carrier figures as PDF](synthetic/data/carrier_phase/carrier_observations.pdf).
Defaults: eight motions, six seconds, eight illustrative GPS L1 signals, 20 Hz
carrier observations, 200 Hz ideal IMU, 0.01-cycle phase noise and random seed 42.
Each motion has both benign and single-transmitter spoofed observations.

The raw carrier CSVs retain their full observed phases. The report's
trend-removed plots use the exact generated trend for visualization; this is
separate from the detector's fitted trends. Input and truth are separated. See
[CARRIER_DATA.md](synthetic/CARRIER_DATA.md) for schema, model and controls.

## Run the geometry and motion examples

From `/home/borna/thesis`:

```bash
python3 -m SpoofingDetection.detect2.synthetic.generate
python3 -m pytest SpoofingDetection/detect2/tests -q
```

Open the generated [interactive report](synthetic/data/motion_geometry/report.html)
or [all figures as PDF](synthetic/data/motion_geometry/motion_geometry.pdf).
Each motion also has PNG figures, a `truth.csv`, geometric projections and a
configuration file. Read [CONVENTIONS.md](CONVENTIONS.md) for all units, frame
directions, quaternion definitions, equations, assumptions and primary sources.

Eight analytic controls are supplied: stationary, constant East velocity,
East oscillation, horizontal circle, circle with coning orientation, three-axis
translation, rotation about a fixed IMU origin, and combined translation/rotation.
Defaults are six seconds
at 200 Hz with an antenna offset of `[0.10, 0.05, -0.20] m` in FRD body axes.

For example, compare rotation with an antenna directly at the IMU:

```bash
python3 -m SpoofingDetection.detect2.synthetic.generate \
  --scenario rotation_only --lever-arm 0 0 0 \
  --output-dir /tmp/detect2-zero-offset
```

Other CLI options set the WGS-84 origin, duration, sample rate and translation
amplitude/frequency scales. Run `--help` for details. This motion-only report
shows truth-derived accelerometer and geometric phase checks. The carrier
generator exports separate inputs consumed by IMU replay. Implementation is
from scratch; the older `torej`
implementations are not a dependency or design reference.

For a circle with a rigidly mounted antenna whose pointing direction sweeps a
45° cone from vertical:

```bash
python3 -m SpoofingDetection.detect2.synthetic.generate \
  --scenario circle_coning --cone-tilt-deg 45 \
  --output-dir /tmp/detect2-circle-coning
```

The antenna pointing axis is negative body Down. It leans radially inward as the
body follows the circle, staying perpendicular to the body's velocity. The
interactive report has separate motion and shared-origin pointing-vector views;
`orientation_cone.png` shows both the actual orbit and the cone, plus direction
components and angle checks. Tilt is prescribed kinematically, not calculated
from a force-balanced flight model. Changing `--cone-tilt-deg` changes the cone
half-angle; 45° is a tilt, whereas perpendicular directions have a 90° separation.

## Layout

```text
detect2/
  README.md                 implementation plan and equation mapping
  CONVENTIONS.md             implemented units, frame definitions and equations
  requirements.txt          Python dependencies (including test dependencies)
  schema.py                 measured-input validation and explicit oracle loading
  geometry.py               implemented coordinate transforms and signal geometry
  detector.py               QR reduction, constrained fits and decision
  motion.py                 supplied-attitude IMU integration and antenna offset
  motion_uncertainty.py     integration covariance and conservative noise gate
  imu_errors.py             repeatable measured-input errors and input export
  imu_robustness.py          matched error sweeps with frozen predictions
  imu_robustness_report.py   motion/phase/gamma sensitivity plots and report
  imu_noise_validation.py    paired uncertainty-gate validation across seeds
  imu_noise_validation_report.py  coverage, error and noise-floor plots
  replay.py                 IMU/oracle replay and prediction export
  evaluation.py             held-out labels, read after prediction
  motion_evaluation.py       held-out displacement/phase and oracle comparison
  motion_plots.py            integration, phase-error and decision comparison plots
  motion_report.py           interactive motion comparison report
  detector_plots.py          fit, residual, cost, gamma and decision figures
  detector_report.py         interactive window/signal comparison report
  DETECTOR.md               implemented mathematics, limits and verification
  IMU_ESTIMATION.md          integration, initialization and truth boundary
  IMU_ROBUSTNESS.md          input-error experiment definitions and limits
  IMU_UNCERTAINTY.md         noise propagation, additional gate and validation
  results/imu/              ideal IMU with noisy-carrier results
  results/imu_clean/        ideal IMU with zero-phase-noise results
  results/imu_robustness/   twenty controlled IMU error conditions
  results/imu_robustness_gated/  error conditions with uncertainty checks
  results/imu_noise_validation/ paired before/after results across IMU seeds
  results/oracle/           noisy-data oracle results
  results/oracle_clean/     noiseless-data oracle results
  synthetic/
    README.md               implemented outputs and remaining generation plan
    motions.py              analytic body and antenna kinematics
    generate.py             motion truth, coordinate checks and plot generation
    plots.py                scientific figures
    report.py               standalone interactive HTML report
    report_template.html    local report viewer (no network dependencies)
    carrier.py              paired phase model, named RNG streams and noise
    generate_carrier.py     observation/ideal-IMU export and comparison report
    carrier_plots.py        observations, components, differences and noise plots
    carrier_report.py       interactive paired phase traces and plot gallery
    CARRIER_DATA.md          measured-input schema, truth boundary and controls
    data/motion_geometry/   eight generated scenarios, figures and reports
    data/carrier_phase/     paired carrier observations, ideal IMU and truth
  tests/                    independent geometry, kinematics and export checks
```

The previous source review is retained as an archival note. It is not part of
the current implementation plan.

## 1. Define inputs, coordinate frames and provenance

- Define a shared relative measurement time in seconds. Preserve original clock
  timestamps and document how each source maps onto this timeline. Resampling
  must not hide offsets, gaps or different acquisition rates.
- Carrier records: measurement time, constellation/satellite/signal identity,
  carrier phase in cycles, wavelength in metres, phase uncertainty, validity and
  lock/continuity information. Group by signal identity, never channel index alone.
- Geometry records: satellite line-of-sight unit vectors in a declared reference
  frame. Initially supply known synthetic directions. Later derive directions
  from simulator output or navigation ephemerides with documented reference
  position/time assumptions. Satellite IDs alone are insufficient.
- Inertial records: measurement time, body-frame accelerometer specific force in
  m/s², angular rate in rad/s and attitude quaternion when available. Declare
  quaternion order, rotation direction, axis convention and whether gravity is
  included. Do not treat Euler angles as translation measurements.
- Calibration/configuration: sensor-to-body rotation, antenna offset from the IMU,
  gravity convention, filter/integration settings, window duration, clock mapping,
  noise assumptions and quality thresholds.
- Keep simulated truth and classification labels separate from measured inputs.
  The explicit known-displacement development mode in stage 2 is an oracle mode,
  not evidence that inertial estimation works.
- Start with multiple GPS L1 signals and one local Cartesian frame. Retain
  wavelength per signal so additional frequencies can be added correctly.

Completion: validate one small example dataset and reject ambiguous units,
invalid geometry, non-monotonic timestamps and missing required fields explicitly.

## 2. Implement the paper's detector with known antenna displacement

First isolate the statistical model from inertial estimation errors. Provide
known, time-aligned synthetic displacement and carrier phase to the detector.

| Paper component | Oracle-mode implementation |
| --- | --- |
| Eq. 3 | Per-signal constant, linear and quadratic nuisance phase terms |
| Eqs. 4–5 | Motion-induced phase scaled by wavelength; separate satellite directions versus one shared spoofer direction |
| Eqs. 8–12 | Noise normalization and QR reduction of the windowed phase/motion system |
| Eqs. 13–15 | Constrained fit of the common frame transformation for the benign model and a shared unit direction for the spoofed model |
| Eq. 16 | `gamma = J_spoofed - J_benign` with the paper's zero threshold |

- Preserve a common rigid rotation in the benign fit and a common direction in
  the spoofed fit; do not fit unconstrained independent directions per satellite.
- Verify phase/displacement signs with a simple distance-change example before
  testing general geometry.
- Define the phase-noise weights and their units explicitly. Use identical
  measurement assumptions when comparing the two model costs.
- Use raw continuous phase windows with nuisance terms fitted in the model.
  The old moving-average detrending is not a substitute for this reduction.
- Record optimizer convergence and compare QR costs with an independent direct
  least-squares calculation on small examples.
- Check motion excitation and matrix rank/conditioning. Return an explicit
  inconclusive result when the chosen model is unidentifiable, phase continuity
  fails or optimization is unreliable. A reduced-dimensional motion model, if
  added later, must be documented separately.
- Output window times, signal count, both costs, gamma, classification and quality
  diagnostics. Positive gamma favors benign; negative gamma favors spoofing.
  Specify numerical tie handling separately from the paper's zero threshold.

Completion: well-conditioned ideal benign and single-transmitter datasets favor
their respective models. Verify invariance to an allowed common frame rotation
and to the modeled quadratic phase trends. Stationary/degenerate inputs must not
produce an unjustified confident classification.

## 3. Estimate antenna displacement from IMU inputs

The supplied-attitude baseline is implemented; see
[IMU_ESTIMATION.md](IMU_ESTIMATION.md). It rotates specific force into ENU,
removes gravity, integrates per window with zero initial position/velocity and
adds antenna translation from the rotating lever arm. Exact measurement times
are preserved; missing coverage and IMU gaps produce inconclusive windows.
No additional high-pass filter is applied. The detector's existing polynomial
projection removes low-order drift, including unknown initial velocity.

Saved comparisons distinguish raw integration drift from the displacement and
phase remaining after quadratic removal. Oracle fits and labels are evaluated
only after IMU predictions are exported. The ideal baseline agrees with the
oracle's decisions, including its conservative inconclusive controls.

Controlled accelerometer noise/bias, supplied-attitude error and timestamp
offset sweeps are implemented in [IMU_ROBUSTNESS.md](IMU_ROBUSTNESS.md).
The detector is not retuned for these experiments. Remaining work:

- Extend the implemented white-noise covariance to justified real sensor models
  and assess systematic attitude, bias and timing errors. The extra uncertainty
  gate is documented in [IMU_UNCERTAINTY.md](IMU_UNCERTAINTY.md).
- Extend sensitivity checks across noise seeds, body axes and measured sensor
  characteristics. Measure motion error, gamma changes and conclusive coverage
  rather than only accuracy on usable windows. Test errors and latency in the
  supplied attitude; gyro measurements remain available for diagnostics.
- Validate the implemented PX4 recording adapter's supplied-attitude timing and
  the empirical carrier epoch alignment. FRD-to-NED quaternions are converted
  into FRD-to-ENU, and reset/gap windows are rejected.
- Evaluate any additional filtering with documented cutoff, transients, resets
  and phase delay; do not silently change the present baseline.
- Extend the recorded-data inspection after validating sensor timing and noise.
  Do not infer missing high-rate phase by interpolating low-rate GNSS data.

Next milestone: independently verify the recorded clock alignment and measured
noise/quality model, and assess the motion needed for conclusive coverage.

## 4. Generate synthetic datasets and verify the complete offline pipeline

Details: [synthetic/README.md](synthetic/README.md).

- Generate antenna/body movement, matching IMU measurements, satellite geometry
  and carrier phase from one physical scenario. Save truth and random seed.
- Begin with ideal multi-axis motion and clean benign/spoofed cases. Add sensor
  noise/bias, phase noise, colored noise, timing errors, gaps and cycle slips
  progressively. Include stationary and weak/one-axis movement.
- Use configurable rates; initial engineering defaults are 20 Hz GNSS, 200 Hz
  IMU and approximately one-second detector windows. Start with smooth motion
  in the paper's 1–5 Hz evaluation range. Verify actual generated frequency and
  amplitude, not only scenario names.
- Include benign-to-spoofed transitions. Treat mixed legitimate/spoofed subsets
  as a limitation/stress case of the initial all-benign versus all-spoofed model,
  not as an already-supported per-satellite classifier.
- Check the generator against analytic derivatives and finite-difference
  physical checks. Do not rely only on the same fitting code to generate and
  validate observations. Add model mismatch after the ideal cases.
- Save per-window output and evaluate false alarms, missed detections,
  inconclusive coverage and transition delay across seeds/noise levels. Separate
  accuracy on conclusive windows from the proportion of windows that are usable.
- Document every departure from the paper, including engineering defaults and
  quality/tie handling. Synthetic success verifies an implementation under its
  stated assumptions; it does not reproduce the paper's real-world accuracy.

First milestone: one command generates labeled benign, spoofed and stationary
datasets; a second command runs offline replay and writes results against labels.
No simulator hardware, receiver or new flight is required for this milestone.

## Later: real acquisition and live operation

The [next-flight acquisition guide](ACQUISITION.md) includes a read-only SSH
inspection of the Pi, measured MAVLink streams, confirmed X20 UART saturation,
and proposed receiver/PX4 logging profiles. No remote settings were changed.

Stage 1 acquisition tools are implemented locally: independent raw capture,
offline auditing, replay, optional TIMESYNC, and reviewed RAM-only receiver
transactions. Open the [Stage 1 manual review](results/acquisition_stage1/report.html)
or [command guide](acquisition/README.md). Pi deployment waits for Checkpoint 1.

An [audit of the stored flight recordings](results/recording_audit/report.html)
identifies external-GPS session `20260707_152258` and PX4 `log_170` as the first
candidate pair. It includes raw GPS L5 carrier cycles and recorded IMU/attitude,
but receive-time pairing accumulates about 30 seconds of delay over 72 seconds
of GPS epochs. Recover and validate measurement-time alignment before detector
replay. Satellite geometry and carrier quality metadata also need attention.
See the [full findings](results/recording_audit/report.md) and
[recording inventory](results/recording_audit/recordings.csv). This audit reads
no GNSS/position measurements from the ULogs and produces no flight detector
decisions.

Before another flight, inspect the actual receiver output and logger settings.
Measure the saved GNSS rate rather than assuming a requested rate is achieved.
Plan for high-rate continuous raw carrier measurements, high-rate inertial data,
attitude, quality/lock fields, navigation ephemerides or satellite geometry,
sensor/antenna calibration and a documented common timebase. Save raw receiver
messages as well as decoded data where possible.

After recorded timing and quality are validated, extend the offline adapter to
live streams using the same detector. New field recordings and any Skydel hardware
experiment are deferred until the synthetic pipeline and acquisition requirements
are ready.
