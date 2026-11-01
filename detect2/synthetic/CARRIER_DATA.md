# Paired carrier-phase datasets

This is step 1: generate detector inputs from the accepted motions. It does not
perform fitting or inertial integration itself. The separate parent-directory
detector implements QR reduction and the two likelihood fits, using either IMU
inputs or oracle displacement; see [DETECTOR.md](../DETECTOR.md) and
[IMU_ESTIMATION.md](../IMU_ESTIMATION.md).

## Run

From the repository root:

```bash
python3 -m SpoofingDetection.detect2.synthetic.generate_carrier
```

Defaults generate all eight motions under `data/carrier_phase/`: six seconds per
motion, eight GPS L1 signals, 20 Hz carrier epochs, 200 Hz ideal inertial inputs,
white phase noise with sigma 0.01 cycles (about 1.9 mm of range) and seed 42.
There are 121 GNSS epochs including both endpoints, 968 long-format carrier rows
per case and 1201 ideal IMU samples per motion. Replay uses half-open one-second
analysis windows to avoid sharing boundary samples.

Open [the comparison report](data/carrier_phase/report.html) or
[the full PDF](data/carrier_phase/carrier_observations.pdf). The report has
interactive phase traces and five static figures per motion: observations,
component decomposition, between-signal differences, movement/sample timing and
noise diagnostics. All report images are embedded for offline viewing.

For example, generate a clean three-axis control in its own directory:

```bash
python3 -m SpoofingDetection.detect2.synthetic.generate_carrier \
  --scenario three_axis --noise-sigma-cycles 0 --no-trend \
  --output-dir /tmp/detect2-clean-carrier
```

For correlated noise on the coning case:

```bash
python3 -m SpoofingDetection.detect2.synthetic.generate_carrier \
  --scenario circle_coning --cone-tilt-deg 45 \
  --noise-sigma-cycles 0.02 --noise-rho 0.7 --seed 123 \
  --output-dir /tmp/detect2-correlated-carrier
```

`--help` lists the other motion, origin, rate and trend settings. `--no-plots`
still exports datasets and the interactive trace report, without PNG/PDF figures.

## Model

For signal j, relative measurement time t in seconds, wavelength lambda and
antenna displacement d(t) from its own initial position:

```text
phi_j(t) = beta0_j + beta1_j*t + 0.5*beta2_j*t²
           - dot(u_j, d(t)) / lambda_j + epsilon_j(t)
```

`u_j` is a fixed receiver-to-source unit vector in ENU. The phase sign is the
range-observable sign: movement toward the source reduces phase. It is not an
assertion about the sign convention of a particular receiver's beat-phase output.
The nuisance polynomial follows the structure of the paper's Eqs. 3–5.

- Benign: the eight signals have different true directions.
- Spoofed: all eight signals have one true direction, default azimuth 230°,
  elevation 15°. This is a physical shared propagation direction, not a fake
  navigation trajectory. All eight signals are spoofed for the entire case.
- Both cases have identical antenna motion and per-signal nuisance coefficients.
- Each case has its own independently seeded noise realization.

The advertised satellite geometry stays identical for both cases. It is not
replaced with the true spoofer direction. All wavelengths are GPS L1 and stored
explicitly per signal. S01–S08 are illustrative identifiers, not real ephemerides.

Default nuisance coefficients are sampled per signal: an integer constant
baseline in `[100000000, 120000000)` cycles, a rate uniform in ±2500 cycles/s,
and acceleration uniform in ±0.2 cycles/s². Coefficients are constant over the
whole recording. CLI options change the rate/acceleration spans; `--no-trend`
zeros all coefficients, including the constant phase baseline.

Noise is stationary Gaussian AR(1):

```text
epsilon_0 ~ N(0, sigma²)
epsilon_k = rho*epsilon_(k-1) + sigma*sqrt(1-rho²)*z_k
z_k ~ N(0,1), independent across signals, times and cases
```

Sigma is the marginal standard deviation in cycles. Default rho=0 gives white
noise. With rho>0, the exported sigma is not an innovation standard deviation
or a claim that measurements are temporally independent. Zero sigma is an ideal
debugging case, not an estimated receiver uncertainty.

Motion is evaluated analytically at each time grid; there is no interpolation,
timestamp rounding, latency or clock offset. IMU data contains ideal specific
force, ideal gyro and exact body-to-ENU attitude. No noisy inertial estimate or
integration is implied.

## Input/truth boundary and schema

```text
carrier_phase/
  manifest.json
  report.html
  carrier_observations.pdf
  <motion>/
    manifest.json
    inputs/
      imu.csv
      satellite_geometry.csv
      frame.json
    benign/carrier.csv
    spoofed/carrier.csv
    truth/
      motion_imu.csv
      motion_gnss.csv
      phase_trends.csv
      benign_components.csv
      spoofed_components.csv
      labels.csv
      generation.json
      metrics.json
    plots/*.png
```

Each motion's manifest names the shared measured inputs and two observation
files. IMU prediction reads the selected carrier file plus `inputs/`.
It does not read phase components, coefficients, true source directions or labels.
An explicit oracle development mode can read `truth/motion_gnss.csv` for known
antenna displacement; that must be kept distinct from inertial estimation.

`carrier.csv` is long format, one row per epoch and signal:

| Field | Meaning |
| --- | --- |
| `time_s` | Common relative measurement time, seconds |
| `signal_id` | Join key to the nominal geometry, S01–S08 |
| `carrier_phase_cycles` | Full observed phase, including nuisance trend and noise |
| `phase_sigma_cycles` | Configured marginal phase standard deviation |
| `carrier_valid` | 1 for every sample in this stage |
| `lock_time_s` | Continuous synthetic lock duration since t=0 |
| `arc_id` | 0 for the single continuous arc in this stage |

There is no spoofing label or latent phase component in this file. Valid zero
phase is allowed in clean controls. `satellite_geometry.csv` provides nominal
ENU directions, constellation/band and wavelength; `frame.json` specifies the
origin, axes, quaternion convention, sensor/body alignment, antenna offset and
gravity. IMU CSV fields explicitly state FRD axis and units; quaternion order is
Hamilton wxyz rotating body vectors into ENU.

All latent data is under `truth/`. The component files align with carrier rows
and store geometric phase, nuisance trend, noise, noiseless phase and actual
directions. `phase_trends.csv` supplies beta0, beta1 and beta2 with units. Labels
are per epoch and case: `is_spoofed=0` or `1`. Generation settings include seed,
noise model, motion parameters, actual spoofer direction and NumPy version.

## Reproducibility and interpretation

RNG streams use explicit PCG64 generators with SeedSequence entropy derived from
the master seed and SHA-256 names containing model version, motion and stream.
Generating one motion or all motions in any order gives identical observations
for that motion with the same settings. Trend and benign/spoofed noise streams
are separate. Runs are reproducible within the recorded software/model version.

Raw spoofed phases need not overlap: per-channel constants, slow trends and noise
still differ. Their *geometric contributions* coincide for equal wavelengths.
The report removes known generated trends only in clearly marked diagnostic
views. This is not a detrending algorithm or detector available from inputs.

Stationary and planar cases are intentionally retained. Their truth labels say
what was transmitted, not that the hypothesis is identifiable from available
motion. Displacement rank and component/noise statistics are truth diagnostics;
they do not constitute classification results or reproduced paper accuracy.

The model does not simulate RF, PLL behavior, real satellite orbits, atmosphere,
antenna gain/polarization, receiver clock jumps, gaps, cycle slips, mixed attacks
or attack transitions. Those are later robustness stages. Separate IMU replay
integrates these ideal inputs with supplied attitude and compares with oracle
displacement only after saving predictions. Both prediction modes fit unknown
nuisance coefficients from observations without reading phase components.
