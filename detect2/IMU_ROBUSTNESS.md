# Controlled tests of our IMU integration

The current priority is our own short-window integration, described in
[IMU_ESTIMATION.md](IMU_ESTIMATION.md). PX4 fused displacement is deferred.
This stage perturbs measured IMU inputs and reruns the same detector, keeping
the ideal baseline and all quality thresholds unchanged. Attitude is still a
supplied input; no gyro-based attitude estimator is claimed.

```bash
python3 -m SpoofingDetection.detect2.imu_robustness
python3 -m pytest SpoofingDetection/detect2/tests -q
```

Open [the gated sensitivity report](results/imu_robustness_gated/report.html) or
[the multipage PDF](results/imu_robustness_gated/imu_robustness.pdf). The
[original ungated report](results/imu_robustness/report.html) is preserved.
Select a motion
and error type for displacement, phase, decision-count and gamma plots. Every
condition links to its own interactive detector and motion/phase reports.

## What changes

Twenty conditions include the ideal reference and these engineering levels:

| Error | Values | Definition |
| --- | --- | --- |
| Accelerometer noise | 0.01, 0.05, 0.2, 0.5 m/s² | Independent Gaussian sigma per axis per IMU sample; not spectral noise density |
| Constant accelerometer bias | 0.01, 0.05, 0.2, 0.5 m/s² | Added in the body Forward axis before rotation/gravity removal |
| Supplied-attitude error | 0.1, 0.5, 1, 3 degrees | Fixed rotation about body Right, applied as `R_input @ R_error` |
| IMU clock offset | −50, −20, −5, +5, +20, +50 ms | Original force/attitude sample at t is tagged as t+offset; carrier times unchanged |
| Combined | One case | 0.05 m/s² noise, 0.05 m/s² Forward bias, 0.5° Right-axis attitude error, +20 ms clock offset |

These are sensitivity settings, not measured specifications for the drone's
hardware. Positive clock offset does not shift the carrier timeline. Forces
and supplied attitude shift together; attitude-only latency is a separate future
experiment. Angular rates are retained without injected gyro errors because the
current displacement estimator uses supplied attitude, not gyro integration.

The perturbed sensor model is:

```text
f_input = f_original + bias_body + sigma * standard_normal
R_input = R_original @ Exp(attitude_error_body_rotation_vector)
t_input = t_original + clock_offset
```

The normal integrator then rotates force, adds gravity, integrates with zero
initial position/velocity and adds the rotating lever arm. It never receives
true initial velocity or truth-based bias corrections. In particular, a constant
body bias during rotation need not produce only polynomial-removable drift.

## Pairing, windows and truth isolation

Every level uses identical carrier observations, nominal geometry, window epochs,
phase covariance and detector settings. Both carrier cases share the same IMU
realization. Stable SHA-256 scenario names and a seed initialize PCG64 streams;
all noise levels scale the same unit-noise sequence. Different scenarios have
separate sequences. The default seed is 42; repeat with `--seed` and a different
`--output-dir` to examine another realization.

The default six-second records are trimmed by one second at each carrier edge.
Each condition therefore uses four identical interior one-second windows per
case: 64 windows across eight motions and both cases. Full IMU input coverage is
retained. This isolates the timing distortion from missing boundary coverage;
it does not relax the normal estimator's coverage/gap checks. `--trim-s` changes
the trim, and rejects a selection with no complete window. The original full
baseline reports still use six windows per case.

Every condition's predictions and integrated displacements are saved, then the
root `run.json` is written, before any condition reads labels or other truth.
Evaluation compares the frozen estimates with true displacement and with an
oracle fit to the same carrier windows. It cannot alter the predictions. Use
`--no-evaluate` to run with no truth access; `--no-plots` retains CSVs and
interactive per-window reports without static figures.

## Outputs and single-condition replay

```text
results/imu_robustness_gated/
  run.json                      all conditions, seed, trim and assumptions
  summary.json / summary.csv    aggregate and per-motion metrics
  report.html
  imu_robustness.pdf
  plots/                        overview and four sensitivity plots per motion
  <condition>/
    imu_errors.json             replayable ImuErrors settings
    inputs/<motion>/imu.csv     actual perturbed force, gyro, attitude and times
    run.json                    source paths and exact error/seed settings
    evaluation.json / .csv
    motion_comparison.json / .csv
    report.html                 interactive carrier fits and decisions
    motion_report.html          raw/observable displacement and phase comparison
    <motion>/<case>/            estimated displacement and frozen detector outputs
```

The original synthetic measurements are never overwritten. A single condition
can be replayed with the usual plotting path, for example:

```bash
python3 -m SpoofingDetection.detect2.replay --imu \
  --imu-errors SpoofingDetection/detect2/results/imu_robustness_gated/combined_errors/imu_errors.json \
  --imu-seed 42 --trim-s 1 \
  --output-dir /tmp/detect2-combined-imu-errors
```

Metrics retain inconclusive coverage, false alarms and missed detections
separately. “Changed from ideal” includes changes to/from inconclusive. Motion
and phase errors are maxima of per-window RMSE after unweighted quadratic
projection; raw drift remains visible in the per-window report. Relative gamma
changes are reported only when both gammas exist and oracle magnitude exceeds
1e-8. Noise weighting and detector thresholds are unchanged, so severe errors
may cause inconclusive outputs or wrong classifications; experiments report both.
The benign fit can absorb a common constant rotation of the displacement frame.
Consequently, motion error in ENU alone does not determine detection failure;
gamma, fit quality and coverage must be assessed alongside it. Errors during
body rotation can have different effects from the same errors at fixed attitude.

## Original failure and additional uncertainty check

The original ungated noisy-carrier sweep exposes a limitation at 0.5 m/s² per-sample
accelerometer noise: 58 of 64 windows become conclusive, with four false alarms
and one missed detection. The ideal reference has 24 correct conclusive windows
and 40 inconclusive controls. Higher apparent coverage is therefore not evidence
of improvement. All five wrong decisions at this level occur in the stationary
and constant-velocity controls. Added noise can make otherwise weak/degenerate movement pass the
motion-rank gate, while the likelihood still treats estimated displacement as
known. The pipeline is not validated for real-flight detection.

The implemented [uncertainty-aware motion-quality check](IMU_UNCERTAINTY.md)
uses propagated white-noise covariance to require reliable movement. The
gated sweep writes to `results/imu_robustness_gated/`; `--no-imu-noise-gate`
replays the prior quality policy into a separate output directory. The paired
multi-seed command compares both modes on identical measurements. The paper's
likelihood and zero gamma threshold are unchanged. This original sweep remains
evidence of the previous failure.

This is one reproducible sensitivity sweep. More seeds, bias directions,
attitude-error dynamics, timing jitter, sensor noise spectra and real vibration
are needed to characterize robustness. Position/attitude reset handling for a
future recording adapter also remains separate. The planned real-data adapter
uses PX4 attitude with our own acceleration/displacement integration; validate
its timing, frame conversion, errors and reset handling. A custom gyro-based
attitude estimator is optional. PX4 fused displacement remains deferred.
