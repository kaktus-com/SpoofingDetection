# Synthetic motion and carrier observations

This folder implements analytic motion, coordinate conversions and visual checks
for the [detect2 plan](../README.md). The geometry command generates analytic
truth and geometric contributions. A separate carrier command now generates
paired benign/spoofed phase observations with noise and slow trends. The parent
directory provides both an oracle-displacement detector and a supplied-attitude
IMU estimator. Run `python3 -m SpoofingDetection.detect2.replay --imu` to use
specific force and attitude, or replace `--imu` with `--oracle` for the known
displacement reference. See [IMU_ESTIMATION.md](../IMU_ESTIMATION.md).

```bash
python3 -m SpoofingDetection.detect2.synthetic.generate_carrier
```

Open the [carrier report](data/carrier_phase/report.html). Read
[CARRIER_DATA.md](CARRIER_DATA.md) for the measured-input/truth layout, model,
reproducibility and parameters. The motion-only report remains separate below.

## Run and inspect

```bash
python3 -m SpoofingDetection.detect2.synthetic.generate
```

Outputs are written under `data/motion_geometry/`. The self-contained
[report.html](data/motion_geometry/report.html) offers scenario selection, a
3D animation, camera controls, time scrubbing and coordinate/attitude readouts.
All figure images are embedded, so the report works offline as a standalone
file. CSV/PDF links also require their accompanying files. Static PNGs and a
[multipage PDF](data/motion_geometry/motion_geometry.pdf) are also exported.

| Motion | Default definition |
| --- | --- |
| `stationary` | Fixed IMU, fixed attitude |
| `constant_velocity` | 0.25 m/s East, fixed attitude |
| `east_oscillation` | East sinusoid, 5 cm amplitude, 2 Hz |
| `circle` | Horizontal 10 cm radius circle, 1 Hz, fixed attitude |
| `circle_coning` | Same circular body path; antenna axis tilted 45° from vertical toward the circle center, with yaw following the orbit |
| `three_axis` | E/N/U amplitudes 4/3/2 cm, frequencies 1.25/1.75/2.5 Hz |
| `rotation_only` | Fixed IMU; roll/pitch/yaw amplitudes 15/10/20°, frequencies 1.5/1/0.5 Hz |
| `combined` | Three-axis translation plus the rotation profile |

The default antenna offset is 10 cm forward, 5 cm right, 20 cm above the IMU.
Every trajectory starts with the IMU at the local origin. Periodic motion is
sampled from an already-active trajectory; it is not a simulated takeoff/startup
transient. Analytic derivatives include the antenna's tangential and centripetal
acceleration. See [frame and sensor conventions](../CONVENTIONS.md).

Each scenario exports `truth.csv`, `geometric_projections.csv`, `config.json` and
five figures: path, kinematics, attitude/ideal IMU, conversions, and projections.
The `circle_coning` case also exports `orientation_cone.png`, showing the actual
orbit, the shared-origin cone, pointing-vector components and constant-angle
checks. The HTML viewer opens this case in its pointing-direction view when it
is included; switch to the motion view to see the translating body and antenna.
Shared files describe the site, illustrative sources and numerical error checks.
CSV data retains every sample; the interactive display uses at most 601 samples.
There is no randomness in this stage. A seed becomes relevant when noise is added.

## Circular motion with a cone sweep

Define the antenna pointing axis as `−body Down`, rigidly mounted to the body.
For orbit angle `theta = 2*pi*f*time`, circle radius `r` and tilt `beta`:

```text
body position ENU = [r(cos(theta)-1), r*sin(theta), 0]
roll = 0; pitch = beta; NED yaw = pi/2 - theta
antenna pointing ENU = [-sin(beta)*cos(theta), -sin(beta)*sin(theta), cos(beta)]
```

The pointing vector has unit length and constant tilt `beta` from vertical. Its
horizontal component points toward the orbit center. It is perpendicular to the
body's tangential velocity. Relocating all these vectors to one origin shows a
cone with half-angle `beta`; this direction plot is distinct from the position
trajectory of the moving antenna. The antenna's body offset is still applied to
its actual position, velocity and acceleration.

`--cone-tilt-deg` sets `beta` between 0° and 90° (default 45°). Body Forward is
perpendicular to antenna pointing and makes angle `90°-beta` from vertical; at
45°, both directions sweep the same cone surface at opposite azimuths. Truth
CSVs export both unit vectors and the antenna tilt from vertical.

This is a prescribed geometry test, not a force-balanced flight-controller
trajectory. Antenna gain, polarization and phase-center orientation effects
remain outside this foundation.

## Observation generation and remaining work

`generate_carrier` uses the same analytic motion at two independently evaluated
time grids: 20 Hz GNSS and 200 Hz ideal IMU by default. It adds matched per-signal
quadratic trends, independent repeatable phase noise, and two sets of true source
directions. Both cases retain the same advertised satellite geometry.

Every motion, including stationary and planar controls, has both cases. Truth
labels describe the transmitter condition, not whether a future detector has
enough motion to distinguish it. White and stationary AR(1) phase noise are
implemented; use zero sigma and `--no-trend` for clean geometric checks.

The parent directory implements the windowed detector with IMU-estimated or
explicit oracle displacement; see [DETECTOR.md](../DETECTOR.md). The IMU baseline
uses ideal measurements and exact supplied attitude. Separate
[integration sensitivity tests](../IMU_ROBUSTNESS.md) perturb measured force,
supplied attitude and timestamps without modifying these generated datasets.
Attitude estimation, measured sensor models, timing jitter, slips,
mixed/transition attacks and physical model mismatch are later work. The
generators themselves do not classify data;
detector outputs and held-out label evaluation are saved separately under
`detect2/results/`.
