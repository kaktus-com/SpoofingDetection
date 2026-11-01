# Supplied-attitude IMU displacement baseline

This stage replaces oracle displacement with an estimate from the exported
specific force and body-to-ENU attitude. The QR reduction, constrained fits,
zero gamma threshold and full-3D quality policy in [DETECTOR.md](DETECTOR.md)
are unchanged. This is a short-window baseline, not a full navigation system
or a reproduction of the paper's complete INS implementation.

## Run and compare

```bash
python3 -m SpoofingDetection.detect2.replay --imu
python3 -m pytest SpoofingDetection/detect2/tests -q
```

Open the [detector report](results/imu/report.html),
[motion comparison](results/imu/motion_report.html) or
[multipage PDF](results/imu/detector_results.pdf).
The separate [clean-carrier comparison](results/imu_clean/motion_report.html)
retains unknown quadratic phase trends but has zero carrier noise. All of these
runs use ideal IMU measurements and exact supplied attitude.

## Inputs and prediction boundary

IMU prediction reads only the dataset manifests, selected carrier observations,
nominal satellite geometry, `inputs/imu.csv` and `inputs/frame.json`. It requires:

- Specific force in calibrated body FRD axes, in m/s².
- Hamilton wxyz quaternions actively rotating body vectors into local ENU.
- The fixed antenna offset from IMU to antenna, expressed in body axes.
- Gravity in ENU, here `[0, 0, -9.80665] m/s²`.
- A shared timestamp in seconds and the nominal IMU rate for gap checks.

The IMU CSV also contains angular rates in rad/s. They are validated and retained,
but this baseline uses supplied attitude rather than integrating the gyro or
running an attitude estimator. The force fields already contain body-frame
measurements, so the sensor-to-body calibration is not applied a second time.

No GNSS position, truth displacement, true initial velocity, generated trend,
true propagation direction or label enters IMU prediction. This command skips
all truth access, including evaluation, and works with the entire `truth/`
directory removed:

```bash
python3 -m SpoofingDetection.detect2.replay --imu --no-evaluate \
  --output-dir /tmp/detect2-imu-inputs-only
```

## Acceleration, integration and antenna geometry

At each IMU sample, using supplied rotation `R` and measured specific force `f`:

```text
a_ENU(t) = R_ENU_from_body(t) f_body(t) + g_ENU
```

Every detector window starts at its first carrier epoch with estimated IMU
position and velocity both zero. Acceleration is taken as piecewise linear
between its measured ENU samples. For a segment of duration h with accelerations
a0 and a1, integration uses:

```text
v1 = v0 + h (a0 + a1)/2
p1 = p0 + h v0 + h² (2 a0 + a1)/6
```

These are exact integrals of the piecewise-linear acceleration model. Window
boundaries inside IMU intervals use linearly interpolated ENU acceleration.
Position at a carrier epoch is evaluated from the resulting piecewise cubic;
epochs are not rounded to an IMU index and integrated position is not linearly
interpolated. Supplied attitude at carrier epochs uses shortest-path quaternion
SLERP, including sign-equivalent quaternions.

For body-frame antenna offset l and first carrier epoch t0:

```text
d_antenna(t) = p_IMU_est(t) - p_IMU_est(t0)
               + R(t) l - R(t0) l
```

The rotational term matters even when the IMU origin is stationary. No antenna
centripetal acceleration is added to IMU acceleration: the IMU measurement refers
to the IMU origin, and the antenna's relative position is added geometrically.

Missing temporal coverage is not extrapolated. A source IMU interval greater
than 1.5 nominal sample intervals causes `imu_time_gap`; insufficient coverage
causes `imu_coverage_missing`. Both leave that window inconclusive. Non-finite
inputs, zero quaternions, non-monotonic times and ambiguous frame conventions
are rejected. The gap test covers the source intervals needed for boundary
interpolation as well as the window interior.

## Why raw displacement drifts

Zero initial velocity is intentional. With exact acceleration, the difference
from true relative displacement is:

```text
d_est(t) - d_true(t) = -v_true_IMU(t0) (t - t0)
```

The detector models a constant, linear and quadratic phase term independently
for every signal. With the fixed propagation directions used in these synthetic
datasets, the phase contribution of this linear displacement drift lies in that
nuisance space. It therefore disappears in the QR reduction. This does not
recover initial velocity or make the raw integrated trajectory accurate.

No extra high-pass filter, truth initialization, bias correction or endpoint
constraint is applied. A constant ENU acceleration bias also creates a quadratic
drift within the nuisance space. A constant body-frame bias during rotation,
attitude error or a timing offset need not have that property; robustness to
these errors has not yet been established.

## Held-out evaluation and plots

All IMU window predictions, fitted parameters, estimated displacement and
`run.json` are saved before evaluation opens any truth file. Label evaluation
then checks the frozen decisions. Optional motion evaluation loads true antenna
displacement and reruns the same carrier windows with oracle displacement;
neither reference changes the saved IMU predictions. `--no-oracle-compare`
skips this motion/oracle evaluation while retaining label evaluation.

The motion comparison shows both raw relative displacement and its residual
after projection off `[1, tau, 0.5 tau²]`. These plotted error metrics use an
unweighted Euclidean polynomial projection in physical units; the detector
continues to apply its configured noise whitening. They coincide for constant
independent measurement weights and represent the same nuisance span otherwise.

The phase comparison applies `-dot(direction, displacement)/wavelength` and the
same polynomial removal to both trajectories. Actual propagation directions
are loaded from held-out phase components only for this evaluation. If absent,
the comparison is explicitly marked as a nominal-geometry response. No generated
nuisance trend or phase noise is subtracted from observed phase by this stage.

The reports include:

- True versus estimated displacement, both raw and after quadratic removal.
- True versus estimated motion phase and its error after quadratic removal.
- Gravity-removed acceleration and the rotating antenna-offset contribution.
- IMU versus oracle gamma and decisions, including inconclusive controls.
- Existing raw/fitted carrier, residual, cost and quality plots.

The interactive motion report selects scenario, case, window, axis or signal.
It shows truth and estimate together, with their difference on a separate axis.
Static figures and offline HTML images are embedded; PDF and CSV links require
the accompanying files.

```text
results/imu/
  run.json
  evaluation.json / evaluation.csv
  motion_comparison.json / motion_comparison.csv
  report.html
  motion_report.html
  detector_results.pdf
  <motion>/motion_estimation.png
  <motion>/imu_vs_oracle.png
  <motion>/<case>/
    estimated_displacement.csv   per-window IMU/antenna displacement, velocity,
                                 and rotational offset at carrier epochs
    window_details.json          integration diagnostics and detector fits
    windows.csv / fits.csv       frozen detector predictions and fitted phase
```

## Current checks and remaining work

Tests cover analytic integration on irregular time grids, all eight motion
controls, noncommensurate IMU/GNSS rates, quaternion sign equivalence, rotating
antenna offsets, zero initial velocity, polynomial-removable drift, missing
coverage and gaps. Replay tests remove truth entirely and guard truth access
until prediction files have been written. Clean-carrier tests retain large
unknown phase trends and compare classification/gamma with oracle results.

The default noisy-carrier run agrees with oracle decisions in all 96 windows:
36 are conclusive and correctly classified, while 60 remain inconclusive under
the existing full-3D motion policy. Maximum per-window displacement vector RMSE
after quadratic removal is about 10.7 µm, and maximum phase RMSE is about
0.0000417 cycles. These errors measure numerical reconstruction from ideal IMU
inputs and exact supplied attitude, not real sensor accuracy. Raw displacement
errors can be much larger because initial velocity is deliberately unknown.

Controlled accelerometer noise/bias, supplied-attitude error and timestamp-offset
sweeps are now available in [IMU_ROBUSTNESS.md](IMU_ROBUSTNESS.md). They preserve
this baseline and the existing numeric movement thresholds. Next work is
measured sensor characterization and attitude estimation. The implemented
[white-noise motion check and paired multi-seed validation](IMU_UNCERTAINTY.md)
add covariance diagnostics while preserving the likelihood fits. Our own
integration remains the priority; PX4 fused displacement is deferred.
