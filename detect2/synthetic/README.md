# Synthetic motion foundation

This folder implements analytic motion, coordinate conversions and visual checks
for the [detect2 plan](../README.md). The first pass generates **truth and geometric
contributions**, not noisy receiver measurements or a completed detector.

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

## Planned measured datasets

Each scenario should store configuration/provenance (including a random seed),
measured inputs and separate ground truth:

| File | Contents |
| --- | --- |
| `config.json` | Rates, duration, frames, calibration, motion parameters, noise parameters and seed |
| `imu.csv` | IMU sample times, body-frame specific force, angular rate, optional measured/estimated attitude |
| `carrier.csv` | GNSS epochs, signal identities, phase in cycles, wavelength, uncertainty and validity/lock information |
| `geometry.csv` | Time-tagged satellite LOS unit vectors in the declared reference frame |
| `truth.csv` | Body/antenna pose, velocity, acceleration/displacement and expected scenario labels |

The table above describes future measured-input exports; these files are not
produced yet except for the foundation's separate truth/configuration outputs.
Final measured-input field names and units are established in stage 1. The
detector must not read scenario truth during normal evaluation. Stage 2's oracle
mode deliberately uses known displacement and must be labeled separately.

## Generation

Start with analytic body position and orientation trajectories. Derive velocity,
acceleration, angular rate and antenna position with its configured body offset.
Use the same physical motion to generate specific force (including gravity in
the declared convention) and carrier-phase changes.

For benign measurements, project antenna motion along the different satellite
directions. For the all-spoofed case, use one common physical transmitter direction
while retaining per-signal wavelength and slow phase terms. A fake navigation
trajectory alone is not a physical single-transmitter phase model.

Begin with noise-free, full-rank multi-axis motion. Then introduce controlled
measurement noise/bias, attitude errors, colored phase noise, clock offset/jitter,
gaps and phase discontinuities. Keep truth labels independent of whether a window
has enough usable information to classify it.

Include benign, all-spoofed, stationary, weak/one-axis motion and transition
scenarios. Mixed legitimate/spoofed signals are an additional stress case with
documented limits of the initial two-hypothesis detector.

For the future measured-input stage, initial configurable defaults are 20 Hz
GNSS, 200 Hz IMU, approximately one-second windows and smooth 1–5 Hz motion.
Current truth and geometric projections are both exported at the motion rate
(200 Hz by default); they do not claim to be 20 Hz receiver measurements. These
are development settings, not a claim that every paper experiment used those
exact rates. Verify actual frequency, metric amplitude and transition smoothness.

## Verification

Check analytic derivatives against numerical checks, geometry/unit transformations,
phase sign and wavelength scaling independently of the fitting routines. Compare
known-displacement and estimated-displacement runs. Report both model costs,
gamma, classifications, inconclusive reasons and performance against labels.

Ideal model-generated data verifies the mathematics under ideal assumptions.
Multiple seeds, model mismatch and later real recordings are needed to assess
robustness; these results must not be presented as reproduced field accuracy.
