# Windowed carrier-phase detector

This implements the windowed statistical stage in
[arXiv:2502.03870v1, Section IV](https://arxiv.org/html/2502.03870v1#S4).
It reads raw carrier phase and nominal satellite geometry, with antenna
displacement estimated from IMU inputs or supplied explicitly in oracle mode.
Both modes use the same statistical fit. The supplied-attitude IMU baseline is
documented in [IMU_ESTIMATION.md](IMU_ESTIMATION.md); the paper's field experiments
have not been reproduced.

## Run and inspect

```bash
python3 -m SpoofingDetection.detect2.replay --oracle
python3 -m SpoofingDetection.detect2.replay --imu
python3 -m pytest SpoofingDetection/detect2/tests -q
```

Default input: `synthetic/data/carrier_phase/`. Oracle output: `results/oracle/`.
Open [the noisy-data report](results/oracle/report.html),
[its PDF](results/oracle/detector_results.pdf), or the separate
[noiseless report](results/oracle_clean/report.html). The clean data still has
unknown slow trends; its trends are fitted rather than supplied to the detector.
IMU output is under `results/imu/`, with an additional
[held-out motion comparison](results/imu/motion_report.html).

For overlapping windows on selected observations:

```bash
python3 -m SpoofingDetection.detect2.replay --oracle \
  --scenario three_axis --case benign spoofed --window-s 1 --step-s 0.5 \
  --output-dir /tmp/detect2-overlapping-windows
```

`--no-plots` still writes decisions, fitted parameters, CSVs and an interactive
trace report. `--no-evaluate` skips all post-prediction truth access; oracle
prediction still requires its explicitly permitted displacement. Observation datasets and
detector outputs must be separate. No receiver or external hardware is required.

## Input and truth boundary

Both modes read the observation manifest, selected `carrier.csv` and nominal
`inputs/satellite_geometry.csv`. IMU mode additionally reads `inputs/imu.csv`
and `inputs/frame.json`, and no files under `truth/`. Oracle mode instead reads
the explicitly permitted displacement columns in `truth/motion_gnss.csv`.
Oracle epochs must match measurement epochs; missing displacement is not
interpolated.

Prediction does not read generated trend coefficients, phase components, actual
propagation directions, generation settings or labels. Filenames are not features.
Every prediction is exported before the separate evaluation module loads
`truth/labels.csv`. Tests change labels and confirm unchanged predictions, and
also remove unnecessary truth files and rename an input case.

Exactly one of `--imu` and `--oracle` is required. In IMU mode, optional held-out
motion and oracle-fit comparisons run after all predictions are saved, and can
be skipped with `--no-oracle-compare`. `--no-evaluate` also skips these comparisons.

## Phase model, sign and frames

```text
phase_j(k) = polynomial_j(k) - d(k)^T v_j / wavelength_j + noise_j(k)

benign:  v_j = A u_j,   A^T A = I and det(A)=+1
spoofed: v_j = v_sp,    |v_sp|=1, common to every signal
```

`u_j` is the nominal receiver-to-satellite direction. `A` maps nominal geometry
to the displacement frame. Synthetic frames are aligned, but the fit still
estimates A. Tests rotate the displacement frame and preserve model preference.
Physical right-handed rotations use SO(3); reflections are not fitted.

Moving toward a source reduces range-observable phase. The paper's motion term
is expressed with this explicit direction/displacement sign. A real receiver's
beat-phase sign requires an adapter rather than an undocumented sign change.

## Nuisance fitting and QR: Eqs. 3–12

Each window uses its actual sample times and a centered, scaled quadratic basis:

```text
tau = (time - sample_midpoint) / sample_time_span
P = [1, tau, 0.5*tau²]
M_j = -displacement / wavelength_j
```

This spans the paper's quadratic nuisance terms. Displacement is referenced to
its first window sample. Each channel's first **observed** phase is subtracted
to avoid manipulating the large constant baseline. Both changes affect only
free nuisance constants. No generated trend or pre-detrended observation is used.

Compute full QR of the whitened design `[P, M_j]` and transform centered phase
by the same Q. The first three rows determine nuisance coefficients; the next
three give the informative residual `B_j v_j - z_j`. Rows after six contribute
a common tail cost independent of hypothesis:

```text
J_reduced = 0.5 * sum_j |B_j v_j - z_j|²
J_total   = J_reduced + common_tail_cost
```

Nuisance coefficients are recovered by triangular solve after fitting directions.
Raw fitted phase and residuals are exported. Stored polynomial coefficients
refer to centered phase and the normalized-time basis; the observed first phase
restores the raw constant. Each window records its time center and scale.

Every reconstruction checks QR cost against directly evaluated whitened full
residual cost. Independent tests use covariance Cholesky factors and ordinary
least squares to verify this equality, including colored-noise cases.

## Constrained fits: Eqs. 13–15

The benign model fits one common proper rotation, not independent directions.
It uses an SVD/Wahba seed and all 24 proper cube rotations as deterministic
starts, with an analytic SO(3) Jacobian. At least two converged starts must agree
on the best found cost. Start counts, agreement, determinant and convergence
diagnostics are exported. This multistart local solver does not certify a global
minimum.

The spoofed cost is quadratic in one common unit vector. An eigenvalue/secular
equation solver handles ordinary and hard cases. Its global optimality
certificate checks stationarity, unit norm and positive semidefiniteness of the
shifted quadratic matrix. The true spoofer direction is never supplied.

## Weighting and decision: Eq. 16

Each observation's supplied phase sigma sets its weight. Zero or very small
sigma is floored to 0.001 cycles by default and flagged. This permits noiseless
controls; the floor is not a noise estimate or probability calibration.

For known stationary AR(1) noise, marginal-sigma-normalized samples are whitened
with the correct first-sample variance and subsequent innovations. Rho comes
from the public input manifest, optionally overridden by `--noise-rho`. This is
an implementation extension; rho=0 gives independent Gaussian weighting.

```text
gamma = J_spoofed_reduced - J_benign_reduced
gamma > 0: benign preference
gamma < 0: spoofed preference
```

The identical tail cancels in gamma; full costs are also exported. The paper's
zero threshold is used, with a numerical tie band of
`1e-8 + 1e-8*(1 + J_spoofed + J_benign)` producing inconclusive.

## Quality policy and reproduction limits

The initial general-3D implementation conservatively requires:

- At least 10 samples and 4 continuous signals per window.
- No internal gap greater than 1.5 nominal sample intervals.
- Finite valid phase, one arc per signal and no locktime decrease. Discontinuous
  signals are excluded for the window rather than repaired or interpolated.
- Nominal direction condition below 1000.
- Displacement **after quadratic projection** of rank 3, RMS at least 1 mm,
  weakest-axis RMS at least 0.1 mm and condition below 1000.
- Reliable optimization, QR/direct consistency, and best-model residual RMS
  below 3 supplied-noise standard deviations with an approximate fitted
  degree-of-freedom correction.

IMU replay additionally propagates configured white accelerometer noise through
integration and polynomial removal. With nonzero configured sigma, lower bounds
on noise-free motion must meet the same RMS/condition thresholds. Failure is
`imu_motion_below_noise_floor`, before fitting. See
[IMU_UNCERTAINTY.md](IMU_UNCERTAINTY.md) for the conservative model bound and
its exclusions. The likelihood costs and zero gamma threshold are unchanged.

These are configurable engineering defaults, not thresholds claimed from the
paper. The motion gate uses supplied or IMU-estimated displacement excitation,
rather than the paper's IMU acceleration event trigger. Lower-rank detection is
deferred. Planar coning
can be informative under another formulation; it is deliberately inconclusive
here. Constant velocity is absorbed by nuisance trends and is not high-frequency
evidence.

Missing data, weak geometry/motion, unreliable fitting, incompatible observations
or numerical ties also produce inconclusive. No confidence probabilities or
field-performance claims are made.

Default one-second windows are half-open and non-overlapping: 20 samples at
20 Hz. An extra terminal endpoint or incomplete trailing part is not folded into
the last complete window. If there is no complete window, an explicit
incomplete-window result is written. Overlapping replay is optional.

## Outputs and evaluation

```text
results/oracle/
  run.json                     mode, settings and prediction input provenance
  evaluation.json              held-out summary, after prediction
  evaluation.csv               labels versus frozen decisions
  detector_results.pdf
  report.html
  <motion>/decisions.png
  <motion>/<case>/
    windows.csv                decisions, reasons, costs, gamma and quality
    fits.csv                   raw/fitted phase and residuals by window/signal
    window_details.json        parameters and optimization diagnostics
    fits.png                   example window, if fits were attempted
    residuals.png              fitted-window diagnostics
```

The report selects motion, input case, window, signal and plot mode. The motion
view removes each candidate's **estimated** nuisance trend; it does not use
generator truth. No fit is drawn for skipped windows.

Evaluation keeps false alarms, missed detections, inconclusive coverage and
accuracy among conclusive labeled windows separate. Missing/mixed labels are
not forced to binary homogeneous truth. These synthetic results do not establish
field accuracy. Baseline IMU runs use ideal sensors and supplied exact attitude;
controlled input-error runs and their noise assumptions are recorded separately.

Tests cover clean/noisy hypotheses, common frame rotations, nuisance-polynomial
invariance, independent Jacobian/covariance calculations, unit-sphere
certificates, continuity, exact epoch alignment, half-open windows and label
isolation. IMU tests additionally cover independent integration checks, antenna
rotation, noncommensurate sampling, gap rejection, unknown initial velocity and
prediction with the entire truth directory removed. Controlled accelerometer,
supplied-attitude and timing sensitivity tests are documented in
[IMU_ROBUSTNESS.md](IMU_ROBUSTNESS.md). Those tests expose false classifications
from noise-induced apparent motion in weak controls. The additional white-noise
quality gate and paired multi-seed validation are documented in
[IMU_UNCERTAINTY.md](IMU_UNCERTAINTY.md). Systematic-error modelling, real sensor
characterization and attitude estimation remain further work.
