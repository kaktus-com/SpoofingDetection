# IMU noise propagation and motion-quality checks

The additional IMU quality check distinguishes motion excitation from integrated
white accelerometer noise. It preserves the existing full-3D policy, likelihood
fits and zero gamma threshold. Original results under `results/imu_robustness/`
remain the ungated reference; new output directories keep comparisons separate.
PX4 displacement is still deferred. Attitude remains supplied.

```bash
python3 -m SpoofingDetection.detect2.imu_robustness
python3 -m SpoofingDetection.detect2.imu_noise_validation
python3 -m pytest SpoofingDetection/detect2/tests -q
```

Open the [gated error sweep](results/imu_robustness_gated/report.html) and the
[paired multi-seed validation](results/imu_noise_validation/report.html).
The sweep includes motion/noise-floor plots and per-window displacement bands;
the validation shows before/after coverage and wrong decisions across seeds.

## Configured noise model

The first model assumes independent zero-mean Gaussian accelerometer errors,
isotropic in body axes, with standard deviation `sigma_a` in m/s² **per axis per
IMU sample**. This is not a spectral density. Rotation by deterministic supplied
attitude preserves isotropy and independence across samples. The model therefore
has one temporal covariance matrix per ENU axis and zero cross-axis covariance.

Sigma is a configured sensor assumption, not estimated from motion or truth.
Synthetic injected errors supply their public configured sigma. An original
input sigma can come from `imu_accel_noise_sigma_m_s2` in the input manifest, or
be overridden with `replay --accel-noise-sigma-m-s2`. Independent injected and
original noise sigmas add in quadrature. Existing ideal synthetic manifests
default to zero. Zero means the configured white component is zero, not that
all real sensor errors are known to vanish. Real recordings need a justified
sensor model rather than this legacy ideal default.

This covariance is conditional on supplied attitude, timestamps, gravity and
antenna geometry. It excludes bias, attitude uncertainty, gyro errors, time
offsets/jitter, coloured noise, uncertain lever arms and numerical integration
error. The sweep still exercises several of those systematic errors separately;
the covariance does not claim to cover them.

## Exact propagation through integration and polynomial removal

The displacement integrator is linear in sampled ENU acceleration. For GNSS
epochs t and original IMU acceleration samples a, write `p = K a`. Each entry of
K integrates one piecewise-linear sample basis function against `(t-s)` over
`[t0,t]`, matching the zero-position/velocity initialization used by normal replay.

The operator retains both weights when an integration boundary lies between
IMU samples. It accounts for their reuse and correlation at subsequent GNSS
epochs. It neither rounds epochs nor treats interpolated acceleration values
as independent new measurements.

```text
C_raw = sigma_a² K Kᵀ
P = [1, tau, 0.5 tau²]
H = I - Q_P Q_Pᵀ
C = sigma_a² (H K) (H K)ᵀ
```

C is the per-axis covariance after the same unweighted quadratic projection
used by the detector's motion-quality checks. The likelihood continues to use
its existing carrier-noise whitening. Integration creates strong temporal
correlations: independent uncertainty at every GNSS epoch would be incorrect.
The rotating lever-arm term adds no covariance in this first model because
attitude and the lever arm are conditioned on as deterministic inputs.

## A conservative noise bound

Let E be the projected noise matrix with three independent `N(0,C)` columns.
For `x = log(1/alpha)`, use:

```text
B² = 3 trace(C) + 2 sqrt(3 trace(C²) x) + 2 lambda_max(C) x
```

The Gaussian model probability that `||E||_F > B` is at most alpha. Its spectral
norm is also at most B on that event. This follows from Proposition 1.1 applied
to the three-axis block covariance in
[Hsu, Kakade and Zhang (2012)](https://www.cs.columbia.edu/~djhsu/papers/quadratic-ecp.pdf).
The use of a Frobenius bound for spectral uncertainty is deliberately conservative.

Default `alpha = 0.001` is an engineering quality setting chosen independently
of seed outcomes. It is a per-window bound conditional on the noise model,
not a spoofing false-alarm probability, classifier confidence or a simultaneous
guarantee across every window. No threshold is fitted to the validation seeds.

For projected estimated displacement D with singular values s, the check uses
Weyl's inequality and the reverse triangle inequality:

```text
axis RMS lower bounds = max(0, s - B) / sqrt(N)
total RMS lower bound = max(0, ||D||_F - B) / sqrt(N)
condition upper bound = (s_max + B)/(s_min - B), if s_min > B
```

These refer to the noise-free reconstruction under the stated model; excluded
systematic and numerical errors can still separate it from physical truth.
The weakest-axis lower bound must exceed the existing 0.1 mm threshold, total
RMS must exceed the existing 1 mm threshold, and condition must remain below
1000. Normal continuity, geometry, optimization and fit checks still apply.
Failure produces `imu_motion_below_noise_floor` before fitting, with no gamma
or fitted hypotheses. Configured zero white noise bypasses the additional check
and preserves the prior baseline path exactly.

This changes the quality policy, not the likelihood. It does not incorporate
displacement uncertainty into the hypothesis costs or provide a complete
errors-in-variables model. Lower-rank motion remains conservatively inconclusive.
At higher noise, rotation-only motion can lose coverage because its weakest
axis is small even while its overall movement is substantial.

## Exports and plots

`window_details.json` saves raw/projected temporal covariance matrices per ENU
axis and the full noise-check diagnostics. `windows.csv` adds the noise bound,
expected noise RMS, weakest-axis lower bound, and gate applied/passed fields.
`estimated_displacement.csv` adds raw/projected per-axis white-noise sigma.
`run.json` records the effective input sigma, gate settings and provenance.

Motion report shading is ±1 sigma for this white-noise component. It is not a
band for full navigation error: unknown initial velocity can dominate raw
displacement, and the other excluded error sources remain unmodelled. The
displacement epochs are correlated. Phase plots intentionally have no band.

The error sweep adds a plot per motion comparing observed weakest-axis and total
RMS against the propagated noise bound plus the original fixed thresholds.
The numerical norm bound applies to the whole window; it is distinct from the
pointwise one-sigma shading.

## Paired multi-seed validation

The default validation uses seeds 42–51 and sigmas 0.05, 0.2 and 0.5 m/s², with
one zero-noise reference. Eight motions, both carrier cases and four identical
interior windows give 1,984 paired windows per mode. Carrier observations and
their noise remain fixed; only IMU noise varies. Both carrier cases share one
IMU realization. These are conditional synthetic results, not independent
field trials or a calibrated detection-probability estimate.

All ungated/gated predictions are exported before any truth is accessed. The
paired runner reuses an ungated fit only when the identical pre-fit quality
check passes. Rejected copies have no fit or gamma. Tests compare that reuse
with direct gated replay and verify that passed fits remain unchanged. Label
evaluation occurs afterward. `--no-evaluate` works without truth directories.

```text
results/imu_noise_validation/
  run.json
  summary.json / summary.csv
  windows.csv                    per-window before/after results and noise floor
  report.html
  imu_noise_validation.pdf
  plots/
  <seed-and-sigma>/
    ungated/                     normal replay with the extra gate disabled
    gated/                       matching predictions after the quality check
```

Use `--seeds`, `--sigmas`, `--scenario`, `--no-plots` or `--no-evaluate` for smaller
runs. The normal replay flag `--no-imu-noise-gate` is available for explicit
ablations. Existing likelihood parameters and fixed movement thresholds are
unchanged. A known-noise synthetic benchmark is the first validation; attitude
estimation, real sensor characterization and timestamp validation remain next.

## Recorded results

For the default 1,984 windows per mode (one ideal reference plus ten seeds at
each nonzero sigma), the saved results are:

| Mode | Conclusive | Correct conclusive | Inconclusive | False alarms | Missed detections |
| --- | ---: | ---: | ---: | ---: | ---: |
| Ungated | 1194 | 1143 | 790 | 42 | 9 |
| Gated | 664 | 664 | 1320 | 0 | 0 |

The zero-noise reference is unchanged. All tested three-axis and combined-motion
windows remain conclusive and correct at every noise level. Rotation-only
windows remain usable through sigma 0.2 m/s² in these seeds; at 0.5 m/s² the
additional check rejects them because of their weak spatial axis. Stationary,
constant-velocity and the other lower-rank controls remain inconclusive.

On the original seed-42 high-noise case, the previous four false alarms and one
missed detection become zero, with 16 correct conclusive and 48 inconclusive
windows. The original ungated artifacts are preserved. These counts apply to
the configured white-noise model and fixed synthetic carrier dataset, not a
claim that real-world error rates vanish.
