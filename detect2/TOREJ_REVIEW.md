# Review of existing `torej` sources and data

Reviewed 2026-10-30 for the planned implementation of arXiv:2502.03870v1.
This is a read-only review of the existing scripts and recordings. Simulator,
serial and RF-control scripts were inspected, not executed.

## Overall finding

The folder makes sense as an experimental workspace: it has motion trajectories,
Skydel scenario control, receiver/raw/RINEX recordings, a bundled RTKLIB tree,
flight ULogs and exploratory phase-processing tools.

The complete likelihood-based detector from Section IV of the paper was not
found in the inspected application sources. In particular, no implementation of
the inertial antenna-displacement estimator, paper-specific QR reduction and
the two constrained likelihood fits was identified. Authorship or equivalence
to the published detector cannot be established from the folder name alone.

The inventory covered application Python/MATLAB sources and notebook code cells,
including the notebook checkpoint. Virtual environments and notebook image
outputs were excluded. The bundled RTKLIB source was also searched for detector
identifiers; ordinary GNSS processing should not be confused with this detector.

## Reuse assessment

| Existing material | What it provides | Use in detect2 |
| --- | --- | --- |
| `gnss_workspace/scripts/create_trajectory.py` | Rest/oscillation/rest trajectory in ECEF | Reference for scenario timing and motion; validate amplitude/units |
| `gnss_workspace/scripts/create_circular_motion.py` | Geographic-coordinate circular/elliptic trajectory | Concept only until angular and distance units are corrected |
| `gnss_workspace/scripts/create_square.py` | Piecewise trajectory | Concept only; current increments accelerate within each side |
| `gnss_workspace/scripts/skydel_december_simulation.py` | Benign/spoofer scenario configuration and recording control | Later hardware experiment reference, with paths/timing revalidated |
| `gnss_workspace/scripts/detrend_carrier.py` | RINEX loading and carrier-phase plots | Exploratory plots, not the statistical detector |
| `gnss_workspace/scripts/ubx_sample_time.py` | Prints local receipt times for RAWX messages | Basic output-rate troubleshooting, not measurement synchronization |
| `gnss_workspace/scripts/HIL_Simulation.py` | Hardware-in-the-loop sketch | Incomplete; requires repairs before execution |
| `gnss_workspace/scripts/GPS-quality.ipynb` | NMEA position scatter/statistics | Position-quality exploration, not carrier/IMU hypothesis testing |
| `kateryna_phase/processing.py` | Phase conversion, differencing and optional high-pass filtering | Inspect data with corrections; do not port as detector logic |
| `kateryna_phase/*.ulg` | Flight inertial/attitude and low-rate phase measurements | Movement-estimation development and importer checks |
| `gnss_workspace/data/` | Historical simulator/receiver recordings, including navigation files | Candidate GNSS replay sources after provenance/continuity checks |

## Processing issues that matter

In `kateryna_phase/processing.py`:

- Lines 38–47 assign phase difference to `phase_rate` without dividing by elapsed
  time. This is metres per sample, not metres per second; subtracting the first
  increment does not turn it into a motion-consistency statistic.
- Each channel column is processed independently. Satellite/signal identity,
  validity flags and locktime resets are not used, despite claims in the README.
  The CSVs do contain changes in identity within a channel and decreases in
  locktime, so a continuous column is not necessarily a continuous phase arc.
- Line 31 uses a 1 Hz cutoff, while the comment says 20 Hz and output fields
  contain `hp20`. For all four inspected flight CSVs, the measured sampling rate
  is approximately 1 Hz: the `fs > 2 * cutoff_hz` check is false and the high-pass
  output stays NaN.
- When filtering is enabled on other inputs, interpolation fills invalid samples
  across the whole series. This must not silently bridge phase slips, satellite
  changes or long gaps. `filtfilt` is an offline operation using future samples;
  its timing behavior cannot be assumed for a live detector.
- A fixed GPS L1 wavelength is used. Limit this operation to compatible signals
  or derive wavelength from constellation/signal/frequency information.
- `main()` hardcodes input/output names. The README advertises CLI options and
  preserved original columns that this implementation does not provide.

In `gnss_workspace/scripts/detrend_carrier.py`, the moving-average subtraction
and plotting are useful inspection tools. They do not implement the paper's
joint polynomial/motion fit. Cached NetCDF loading applies a hardcoded 2021 time
range, whereas loading the original RINEX does not apply that same range.

In `gnss_workspace/scripts/create_circular_motion.py`, `omega` is converted to
degrees/second and then passed to Python's radian-based sine/cosine. The phase
constant also has no declared unit. Equal latitude/longitude degree amplitudes
do not describe a metric circle at the stated latitude.

In `gnss_workspace/scripts/create_square.py`, increments grow with loop index
(`dlon * j` and `dlat * j`) instead of being constant-velocity steps. In
`create_trajectory.py`, the 8 Hz angular frequency formula is correct, but the
comment interpreting longitude degrees as metres needs latitude-aware conversion.
Prefer generating all new motion in local metres before any geographic transform.

In `gnss_workspace/scripts/HIL_Simulation.py`, `elapsedMs` is tested before being
initialized, position/velocity are tuples that are later assigned into, and the
position supplied is a two-element latitude/longitude tuple despite the ECEF API
call. A `testbed` module is imported but was not found in the reviewed application
files. This is a sketch rather than a runnable acquisition reference.

The Skydel script also has machine-specific paths, addresses and scheduled
actions. Its oscillation starts with a position step at 600 seconds; new synthetic
tests should control transition smoothness rather than introduce an accidental
impulse. No Skydel API compatibility or hardware execution was tested.

## What the current recordings contain

All four CSVs have 420 columns, including 32 carrier channels and per-channel
GNSS/satellite/signal IDs, frequency IDs, locktime, flags and uncertainty fields.
They have no acceleration, angular-rate or attitude columns.

| CSV | Rows | Median carrier sample interval |
| --- | ---: | ---: |
| `kateryna_phase/flight.csv` | 114 | approximately 1 s |
| `kateryna_phase/flight2.csv` | 106 | approximately 1 s |
| `kateryna_phase/flight3.csv` | 236 | approximately 1 s |
| `kateryna_phase/flight4.csv` | 157 | approximately 1 s |

Rates were checked from both `time` and `sensor_gps_raw/timestamp_sample`.
The latter is in microseconds. These files can check decoding, grouping and
continuity handling. They cannot resolve the paper's 1–5 Hz phase variations;
upsampling cannot recover those missing measurements.

The six ULogs in `kateryna_phase` were inspected with `pyulog`:

- `sensor_combined` contains acceleration and angular rate, logged at median
  rates of approximately 189–200 Hz.
- `vehicle_attitude` is approximately 20–21 Hz.
- `sensor_gps_raw` is approximately 1 Hz.
- Individual `sensor_accel`/`sensor_gyro` topics are logged at approximately 1 Hz,
  and `vehicle_imu` instances at approximately 2 Hz. Select the stream based on
  its actual recorded rate, not its name.

The additional `SpoofingDetection/bestcase/log_170_2026-7-7-15-25-22.ulg` contains
approximately 190 Hz `sensor_combined` and 21 Hz `vehicle_attitude`. No
`sensor_gps_raw` dataset was returned for that log in the filtered inspection.

High-rate inertial data is therefore already available for movement-estimation
development. Its calibration, acceleration convention, sensor selection and
accelerometer/gyro timing (including `accelerometer_timestamp_relative`) still
need to be checked before integration. Sample counts/rates establish availability,
not sensor accuracy or successful displacement reconstruction.

A representative older GNSS recording,
`gnss_workspace/data/20231218_1309_Clean/Processed/receiver.obs`, has a median
epoch interval of 0.05 s over its first 500 epochs (20 Hz). This is a promising
GNSS source. This check does not establish continuity over the whole file,
scenario ground truth, a synchronized IMU stream or detector performance.

## Consequences for the plan

Proceed with stages 1–4 using synthetic paired measurements, while using the old
recordings for targeted importer and movement checks. Reuse trajectory concepts
and scenario metadata, with unit and timing validation. Do not use the exploratory
phase processing as a reference implementation of the paper's detector.

Keep a future flight as a later validation task. First define the recording
contract and verify the actual GNSS logging rate and clock alignment on the bench.
