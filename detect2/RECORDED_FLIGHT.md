# Recorded July flight inspection

Open [the interactive HTML report](results/recorded_july07/report.html) or
[all scientific figures as PDF](results/recorded_july07/flight_inspection.pdf).
This is an offline inspection and conditional quality screen, not a validated
field spoofing alarm. Raw receiver packets, lock flags and a measured in-flight
noise model were not saved in the supplied recording.

The sources are external receiver session `20260707_152258` and PX4 `log_170` in
`/home/borna/Documents/stored_flights`. The user identified this as an ordinary
non-spoofed flight and supplied the external antenna offset as **[0.02, 0, 0] m
in body Forward/Right/Down coordinates**. The label is not used to choose the
clock offset, geometry, integration or a detector outcome. All original files
remain unchanged; source hashes are in the generated `run.json`.

## Time and frame handling

The phase recording contains 71.72 seconds of receiver GPS epochs, while the
receive clock advances 102.09 seconds. Reusing the saved nearest-attitude row
would match the wrong physical times. A candidate clock uses actual external
GPS epoch intervals with a constant offset into PX4 boot time. Its initial
anchor is the minimum of:

```
paired_attitude_boot + carrier_host_arrival - attitude_host_arrival - GPS_elapsed
```

The sign of the arrival difference is positive: carrier arrival minus attitude
arrival. It remains a receive-time proxy with unknown message age/latency.
`--estimate-clock` scans offsets and selects the best **discovery** score.
Separate alternating holdout windows check the fitted phase response without
refitting its response vectors. This recording favours +0.10 seconds on the
50 ms search grid in both subsets. This is an empirical alignment estimate,
not an externally verified sensor synchronization or confidence interval.
The scalar clock rate is assumed to be one second per second. User flight
labels are never part of this search.

Specific force and angular rate come from `sensor_combined` in body FRD;
quaternions come directly from `vehicle_attitude.q`. Hamilton FRD→NED rotations
are composed with the fixed NED→ENU rotation. No gyro attitude estimator is
run. Already calibrated body force is not rotated through the board calibration
a second time. Extra EKF acceleration-bias estimates and PX4 fused position
are not used to integrate displacement.

Attitude is interpolated only inside covered intervals, excluding reset
transitions and interpolation gaps above 75 ms. The existing integrator retains
its 1.5 nominal-sample-interval acceleration-gap check. Each window starts with
zero position and velocity, adds the rotating measured antenna offset, and
uses the same quadratic projection as the detector. Absolute navigation drift
and the observable within-window displacement are different quantities.

## GNSS exception authorized by the user

The user subsequently permitted reading the ULog GNSS **only to locate the
flight**. `vehicle_gps_position` supplies one fixed median latitude, longitude
and ellipsoidal height for satellite geometry. Those positions do not enter
displacement, velocity, initial conditions or timing. The ULog GPS-derived UTC
header is excluded. The motion adapter itself reads only `sensor_combined`,
`vehicle_attitude` and `vehicle_land_detected`.

GPS satellite directions are reconstructed from the public NOAA
[`brdc1880.26n.gz` navigation file](https://noaa-cors-pds.s3.amazonaws.com/rinex/2026/188/brdc1880.26n.gz).
The downloaded file is archived under `adapted_inputs/`; its hash and source URL
are retained. See [NOAA's data description](https://www.ngs.noaa.gov/CORS/data.shtml).
The RINEX 2 parser and orbit propagation were implemented from scratch following
[IS-GPS-200N Table 20-IV](https://www.gps.gov/sites/default/files/2025-07/IS-GPS-200N.pdf).
The implementation checks GPS week, health, ephemeris age and transmission time;
it includes iterative signal transit time and Earth rotation. Directions are
calculated at each short window's midpoint. The observed `gnssId=0,sigId=7`
signals are GPS L5_Q, using 1176.45 MHz rather than the synthetic L1 wavelength;
see [IS-GPS-705J](https://www.navcen.uscg.gov/sites/default/files/pdf/gps/IS-GPS-705J.pdf).

The report compares measured phase residuals with nominal ENU LOS projections.
These projections do not fit the paper's common frame rotation and are not
likelihood-ratio decisions. The exploratory time search uses a shared but
unconstrained phase response per signal and does not certify satellite geometry
or benign/spoofed signal origin.

## What the quality results mean

All windows retain missing observations and real epoch gaps. Finite phases are
not treated as certified uninterrupted carrier lock. Missing lock/validity,
half-cycle and per-observation uncertainty fields remain blank in the adapted
CSV; the input pack is marked `inspection_only_not_replay_ready`.

Several per-sample acceleration white-noise assumptions (0.3, 1 and 3 m/s²)
probe the existing uncertainty gate. The grounded adjacent-force difference MAD
is also reported as a rough proxy, not an in-flight calibration. In-flight
vibration and attitude/timing errors are not covered by the white-noise model.
The conditional screen rejects insufficient phase availability, integration
coverage/gaps/resets, and motion below the noise floor before any likelihood
fit. Conclusive decision count and gamma are not manufactured when these
requirements fail. An inconclusive result on an ordinary flight is not proof
that benign signals have been detected correctly.

## Reproduce

The archived navigation file is present after the first completed inspection.
The external source recordings are read only.

```bash
python3 -m SpoofingDetection.detect2.recorded_flight \
  --recording-dir /home/borna/Documents/stored_flights/extra_gps/recordings/20260707_152258 \
  --ulog /home/borna/Documents/stored_flights/flights/log_170_2026-7-7-15-25-22.ulg \
  --lever-arm-body 0.02 0 0 --expected-case benign --estimate-clock \
  --origin-from-ulog \
  --navigation-file SpoofingDetection/detect2/results/recorded_july07/adapted_inputs/broadcast_navigation.rnx.gz
```

`--clock-offset-s` instead of `--estimate-clock` explores a specified offset
from the same provisional anchor. Neither option marks timing verified. The
HTML embeds its charts, images and interactive data; it works without network
access. CSV/JSON/PDF exports are separate linked files.
