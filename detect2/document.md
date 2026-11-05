# Recording requirements for detect2

Implementation sequence and manual approval checkpoints:
[drone recording implementation plan](IMPLEMENTATION_PLAN.md).

## Current July recording

The recording is useful for phase, timing and motion analysis, but has not
produced a reliable clean/spoofed decision. Fifty of 72 windows integrate;
none passes the tested one-second movement/noise checks. The estimated timing
offset is not independently verified, and receiver lock/slip metadata was not
saved. The full-3D movement requirement is a conservative implementation choice;
a future solver supporting planar motion may improve coverage.

A live X20 check also found UART saturation and missing RAWX epochs. This is a
confirmed acquisition problem, but does not establish the cause of every July
recording issue.

## Required recording changes

- **X20 output:** target continuous 20 Hz RAWX, reduce redundant high-rate
  messages, and keep low-rate navigation, satellite and communication-health
  messages. Verify actual throughput and epoch continuity.
- **Original data:** save receiver bytes before filtering or attitude matching.
  Preserve all signal IDs, carrier phase, pseudorange, Doppler, quality/lock/
  half-cycle flags, uncertainty, receiver time/week and clock-reset information.
  The current wide CSV exports only selected GPS L5 signals.
- **PX4:** retain native accelerometer/gyro samples around 200 Hz and supplied
  attitude quaternions at least 100 Hz where supported, with sample timestamps
  and reset information. Proposed logging settings are `SDLOG_PROFILE=17` and
  `SDLOG_MODE=2`; verify them on the installed firmware. Displacement remains
  our own IMU integration.
- **Timing:** record MAVLink TIMESYNC exchanges for Pi-to-PX4 timing and establish
  an external-GNSS clock bridge, preferably using timestamped PPS/timepulse
  events. Nearby packet-arrival times are insufficient.
- **Recorder:** archive MAVLink independently, monitor queue age/dropouts, and
  perform decoding, alignment and detector processing after capture.

## One complete test session

Retain `receiver.bin`, `flight.ulg`, `mavlink.bin`, receive-timestamp indexes,
`clock_sync.csv` and a `manifest.json` containing configuration, frames, antenna
offset and experiment labels. The confirmed antenna offset is `[0.02, 0, 0]` m
in body Forward/Right/Down coordinates.

Begin with a two-minute ground test. For a later flight, include 30–60 seconds
stationary before and after controlled, varied movement. Verify sustained
sample rates, complete phases and stable clock mapping before interpreting
detector outcomes. An inconclusive result is not a confirmed clean detection.

These are proposed acquisition changes; receiver configuration, PX4 parameters
and Pi recording scripts have not been changed.
