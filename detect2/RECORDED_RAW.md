# Detector replay on the November raw recordings

Open [the HTML report](results/november_detector/report.html),
[all figures PDF](results/november_detector/november_detector_inspection.pdf),
or [the machine-readable summary](results/november_detector/summary.json).

This is a **conditional detector replay with unverified timing and field error
models**. It runs the existing detector, including its motion-noise, signal,
rank, geometry and model-fit gates. Validated conclusive decisions remain zero.
The report keeps numerical model preferences separate from validated decisions.
No flight benign/spoofed labels are assumed or used.

## Reproduce on another computer

From the `SpoofingDetection` checkout root, after installing the two requirements
files as described in [HANDOVER.md](HANDOVER.md):

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m detect2.recorded_raw
```

Defaults use the three November recordings already committed under
`detect2/results/recorded_nov05/review_20261105T135817Z/evidence/sessions/`.
Outputs go to `detect2/results/november_detector/`. To preserve an existing
derived run, pass `--output-dir /path/to/new/results`. Source archives are read
only. The adapter checks their recorded fingerprints and index coverage before
replay and hashes them again afterwards. It does not rewrite source timestamps,
repair phase gaps, or resample missing measurements.

Run the focused adapter checks from the default clone directory:

```bash
PYTHONPATH=.. python -m pytest detect2/tests/test_recorded_raw.py -q
```

## Inputs and signal quality

The adapter decodes checksum-valid RAWX and NAV-SAT packets directly from the
original receiver bytes. The host timestamp is the receive chunk that completed
each RAWX frame, rather than an assumed nearest attitude row. Receiver and
MAVLink indexes must cover their binary archives completely.

GPS L1 C/A (`gnssId=0, sigId=0`, 1575.42 MHz) supplies one signal per satellite.
Other constellations/frequencies remain in the source data. Phase is retained in
cycles; carrier and half-cycle validity are required. Invalid uncertainty code
15 is excluded; other RAWX `cpStd` values scale by 0.004 cycles. The detector's
existing 0.001-cycle floor remains in force. Locks, missing epochs, loss of
validity and half-cycle subtraction changes delimit arcs. Lock saturation alone
does not create a break.

NAV-SAT azimuth/elevation is converted to ENU unit vectors. Only declared healthy
GPS satellites using ephemeris-source orbits are accepted, with a preceding sky
epoch at most two seconds old. A future geometry epoch is not substituted.
Missing geometry masks that signal. This is receiver-reported nominal geometry,
not an independent orbit reconstruction or authenticity check. The RAWX/NAV-SAT
version fields and lengths are checked. Field references are the u-blox
[X20 interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-X20-HPG-2.03_InterfaceDescription_UBXDOC-304424225-20728.pdf),
sections 3.15.15 and 3.16.4. That reference describes protocol 50.03; this recorder
identified 50.02, with the checked packet layouts/version 1 used here. No
receiver configuration is changed by replay.

## Motion and provisional timing

Motion uses calibrated `sensor_combined` specific force and supplied
`vehicle_attitude` quaternions, converted from FRD→NED to FRD→ENU. The existing
integrator starts each window at zero velocity/position and adds the confirmed
2 cm forward antenna lever arm. QR removes the quadratic nuisance component.
No gyro attitude estimator, PX4 fused displacement, GNSS motion or onboard
GPS-derived UTC is used. Attitude/calibration transition windows are rejected;
integration and interpolation retain the existing gap checks.

These three recordings predate active TIMESYNC and lack recorded PPS edges.
Their later bench TIMESYNC estimates are not extrapolated backwards. The
provisional anchor is:

```text
anchor = min(RAWX_host_receive - GPS_elapsed)
       - min(HIGHRES_IMU_host_receive - PX4_boot_publication_time)
candidate_PX4_time = anchor + GPS_elapsed + fixed_offset
```

It assumes equal clock rates and leaves unknown differential transport/sensor
latency. Week/TOW differences are computed separately to preserve precision.
The primary offset is zero; fixed ±50 ms offsets probe sensitivity. No offset
is selected by phase fit, detector outcome, GNSS motion or flight label.

## Noise sensitivity and outputs

The primary run uses the existing **1 m/s² per-sample acceleration white-noise
assumption**, not a field calibration. A separate stationary-proxy sensitivity
uses the maximum axis MAD of adjacent landed force differences divided by
0.9538725524. That proxy is computed independently of carrier phase and model
preferences. It is optimistic for airborne coloured vibration and is not the
primary result. The motion-uncertainty gate stays enabled in both cases.

Each configuration exports `windows.csv` with separate conditional and validated
decisions, `window_details.json`, `fits.csv`, and
`estimated_displacement.csv`. `conditional_core_windows.csv` preserves the
existing detector's unwrapped outputs. It must not be consumed as a validated
alarm stream. A gamma can exist even when neither model passes the fit-quality
gate; read the reason and best-fit normalized RMS together.

The primary high-motion example is selected from IMU motion RMS only. Its dashed
phase curves use the nominal LOS directly; they do not replace the common
rotation fitted by the benign hypothesis. Land status is used only to label
ground/airborne summaries after prediction.

See [CONTINUATION.md](CONTINUATION.md) for the remaining PPS software and
validation checkpoints.
