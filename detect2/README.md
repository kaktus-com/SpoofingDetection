# detect2: geometry foundation and detector plan

Implement the detector in **Consumer INS Coupled with Carrier Phase Measurements
for GNSS Spoofing Detection**, arXiv:2502.03870v1, inside this directory.

Reference: [local paper](../../2502.03870v1.pdf),
[version 1 online](https://arxiv.org/html/2502.03870v1).

Status: the geometry and analytic-motion foundation is implemented. The current
pass covers motion definitions, coordinate conversions and plots. The statistical
detector, noisy measurement generation and inertial displacement estimation are
still planned in stages 1–4 below. Another flight and live integration are later.

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
amplitude/frequency scales. Run `--help` for details. The ideal accelerometer
and geometric phase contributions are truth-derived checks, not measured inputs
to a completed detector. Implementation is from scratch; the older `torej`
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
  schema.py                 [planned] measured-input validation and stream definitions
  geometry.py               implemented coordinate transforms and signal geometry
  detector.py               [planned] QR reduction, constrained fits and decision
  motion.py                 [planned] inertial estimation of antenna displacement
  replay.py                 [planned] offline detector execution
  synthetic/
    README.md               implemented outputs and remaining generation plan
    motions.py              analytic body and antenna kinematics
    generate.py             motion truth, coordinate checks and plot generation
    plots.py                scientific figures
    report.py               standalone interactive HTML report
    report_template.html    local report viewer (no network dependencies)
    data/motion_geometry/   eight generated scenarios, figures and reports
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

| Paper component | Planned implementation |
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

## 3. Add inertial estimation of antenna displacement

- Transform measured specific force using attitude, remove gravity and compensate
  for sensor/antenna geometry. Include antenna translation caused by rotation
  around a nonzero antenna offset.
- Estimate attitude from gyro/accelerometer measurements when required, with
  initial attitude and heading assumptions stated. Existing attitude can be used
  as a separate development input whose errors are evaluated.
- Implement the high-frequency separation and short integration needed to
  estimate antenna displacement. Document filter order, cutoff, initialization,
  transients, integration resets and phase delay; the paper does not fix every
  implementation setting.
- Preserve accelerometer/gyro sample timing and resample displacement to GNSS
  measurement epochs. Account for filtering latency in that alignment.
- Compare estimated displacement and classification with the known-displacement
  mode. Report error attributable to IMU estimation separately from model fitting.
- Add recording adapters later, after validating sensor frames and timing. Do
  not infer missing high-rate phase from interpolated low-rate GNSS data.

Completion: the synthetic inertial stream reconstructs the relevant antenna
motion within documented tolerances, and its detector results can be compared
with the oracle mode under increasing sensor and timing errors.

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

Before another flight, inspect the actual receiver output and logger settings.
Measure the saved GNSS rate rather than assuming a requested rate is achieved.
Plan for high-rate continuous raw carrier measurements, high-rate inertial data,
attitude, quality/lock fields, navigation ephemerides or satellite geometry,
sensor/antenna calibration and a documented common timebase. Save raw receiver
messages as well as decoded data where possible.

After the offline pipeline passes, add adapters for recordings and live streams,
using the same validated detector. New field recordings and any Skydel hardware
experiment are deferred until the synthetic pipeline and acquisition requirements
are ready.
