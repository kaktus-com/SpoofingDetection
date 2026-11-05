# Recording the next flight

The MAVLink router is already delivering useful telemetry. The main required
change is the recording pipeline: preserve independent original streams, their
measurement timestamps and their quality fields, then align them offline.
Keep PX4 native ULog as the primary accelerometer/attitude record.

## Confirmed on the Raspberry Pi, 3 November 2026

SSH host: `nss@10.15.4.141`, hostname `kth-nss-6`.

The active router command is:

```bash
/usr/bin/mavlink-routerd -e 127.0.0.1:14551 /dev/ttyACM0:1500000
```

`/dev/ttyACM0` is the CubeOrange USB virtual serial connection. Its stable name
is `/dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00`. The baud argument on this
USB connection must not be confused with the physical UART bandwidth limit of
a separate external receiver connection. Do not diagnose this Cube USB link
from July's `SER_TEL1_BAUD=57600` or `MAV_0_RATE=1200` settings alone: those
parameters describe a different PX4 interface.

The router is running manually. The systemd `mavlink-router.service` is inactive.
Starting another router against the same Cube USB device would create a second
owner. A reproducible service configuration can replace the manual launch in a
later deployment; the current process was not stopped or reconfigured.

A passive 12-second measurement of the existing router endpoint found:

| Message | Measured arrival rate | Currently saved by the Python recorder? |
|---|---:|---|
| `ATTITUDE` | 99.88 Hz | Yes, Euler angles and boot milliseconds |
| `ATTITUDE_QUATERNION` | 49.98 Hz | No |
| `HIGHRES_IMU` | 49.98 Hz | No |
| `TIMESYNC` | 10.00 Hz | No |
| `SYSTEM_TIME` | 1.00 Hz | No |

Distinct sensor/message timestamps had comparable rates for the three attitude/
IMU streams. A TIMESYNC packet arriving does not mean the recorder performs a
clock-offset estimate; the present Python synchronizer does not implement it.
No messages requesting stream-rate or flight-parameter changes were sent during
this inspection. The autopilot heartbeat reported disarmed.

The external device was initially absent and appeared on the requested recheck.
It is `/dev/ttyUSB0`, with stable identity
`/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DQ00KK6P-if00-port0`.
A read-only MON-VER query identified **ZED-X20P, EXT HPG 2.00, protocol 50.02**.
The physical receiver connection is an FTDI USB-to-UART bridge, not native X20
USB. Read-only CFG-VALGET queries confirmed UART1 at 921600 baud, a 40 ms
measurement interval, navigation ratio 1, and RAWX, NAV-SIG, RXM-MEASX and
NAV-PVT all enabled every navigation epoch on UART1.

An independent **8-second byte-only capture** read 737,554 bytes, approximately
92,172 bytes/s (measurement-boundary tolerance around the 92,160 bytes/s UART
limit). All 2,021 complete UBX frames validated; maximum host pending input was
496 bytes. Nevertheless only 110 RAWX epochs were transmitted, with a median
80 ms GPS-epoch interval, 65 gaps above 60 ms and a largest gap of 280 ms. RAWX
contained 104 observations/epoch. MON-COMMS showed 26,736 bytes pending on the
active output port, transmit-buffer usage peaking at 99% in its monitoring
interval and a historical peak of 100%. **The receiver output is saturating the
UART; missing RAWX epochs occur even when host byte capture keeps up.**

The direct serial-decoding test also printed checksum/format errors; a separate
fast byte capture followed by offline validation had zero invalid complete UBX
candidates. This supports separating raw acquisition from decoding. It does not
prove that every historical July loss had the same cause.

## What the current start script actually records

Both workspace and Pi `/home/nss/SpoofingDetection/start` run:

```bash
python3 spoofing_detector.py \
  --record-format csv --csv-layout wide --no-detrended --no-alert
```

This already disables moving-mean detrending and detection alerts. It does not
invoke the new detect2 paper detector. It also does not save a complete raw
receiver archive:

- The wide writer has fixed columns for GPS `gnssId=0,sigId=7`, PRNs 1–32. Other
  constellations/frequencies are silently omitted even if read from the receiver.
- `raw_carrier_samples()` drops observations with invalid pseudorange or carrier
  flags before saving. Those observations and their flags should instead remain
  in the original archive so discontinuities can be investigated later.
- The decoding path discards raw receiver packets, GPS week/leap information,
  pseudorange/Doppler, C/N0, carrier uncertainty, lock and half-cycle fields.
- The synchronizer chooses an attitude message nearest in host arrival time and
  rejects samples beyond its maximum arrival difference. That is not a common
  physical measurement clock. GNSS should be recorded independently even when
  MAVLink/attitude is temporarily unavailable.
- The MAVLink reader filters for `ATTITUDE`; it does not retain the incoming
  quaternion, IMU or clock-synchronization packets.

For an **interim** run that avoids the fixed wide-column loss, the existing
Python command supports:

```bash
cd /home/nss/SpoofingDetection
/home/nss/venv/bin/python spoofing_detector.py \
  --record-format csv --csv-layout long --no-detrended --no-alert
```

This is not the complete acquisition fix: it still drops quality metadata,
does not archive raw receiver bytes and still pairs by arrival time. Run one
recorder that owns the receiver port. `./start` already launches Python, so
running both creates duplicate consumers.

## Target recording arrangement

```text
PX4 ───────────► native SD-card .ulg (primary IMU + supplied attitude)
  └─ USB ─────► mavlink-router ─► independent MAVLink archive + arrival stamps
External X20 ─► independent original receiver-byte archive + arrival stamps
X20 PPS/time tags + MAVLink clock exchange ─► documented clock bridges
```

The serial acquisition path should write original bytes before filtering or
requiring a matching attitude. Store an index of byte offset/length and host
monotonic receive timestamps. Decoding, phase detrending, alignment and detector
fits can operate after capture. Archive all original traffic including receiver
quality and clock reset events. Keep separate health statistics for parser
failures, input byte rate, queue depth/age and dropped data; a large queue must
not hide steadily growing latency.

Each session should contain:

```text
session/
  receiver.bin             original receiver bytes, including all enabled packets
  receiver_receive.csv     byte offsets/lengths and host monotonic timestamps
  mavlink.bin              original MAVLink packets/datagrams
  mavlink_receive.csv      receive timestamps, source IDs and sequence diagnostics
  clock_sync.csv           clock requests/replies, round trip, PPS events if available
  flight.ulg               complete PX4 SD-card log
  manifest.json            hardware/configuration, rates, frame/offset and labels
```

Decoded CSVs are useful additional exports. They should not replace these sources.

## PX4 logging settings

Read-only requests to the current drone confirmed `SDLOG_PROFILE=1`,
`SDLOG_MODE=0`, and `SDLOG_BACKEND=3`, matching the July profile/mode. On the inspected PX4 source
the high-rate profile is bit 4. Retaining the default set and enabling that bit
gives **`SDLOG_PROFILE=17`**. **`SDLOG_MODE=2`** records boot to shutdown, allowing
stationary data before takeoff and after landing. Apply and verify these on the
actual firmware, including its reboot requirements; no remote parameters were
changed here. See [PX4 logging](https://docs.px4.io/main/en/dev_log/logging).

Aim to retain native accelerometer samples around 200 Hz and supplied attitude
at least 100 Hz where the publisher supports it. These are acquisition targets;
measure unique sample timestamps, gaps and ULog dropouts rather than trusting a
requested rate. Retain `sensor_combined`, `vehicle_attitude` (including sample
time/reset fields), sensor selection/bias/status and landing state. Native ULog
is preferable to relying on a telemetry link for the full inertial record.

The meaning of `logger_topics.txt` differs between the local older/custom PX4
source and current online documentation (replacement versus additions). For
this setup, use the verified profile mechanism first rather than installing a
small topic file that could unintentionally replace the default log set.

MAVLink can additionally retain `ATTITUDE_QUATERNION`, `HIGHRES_IMU`,
`TIMESYNC` and `SYSTEM_TIME`. The installed PX4 implementation of `HIGHRES_IMU`
may already subtract estimator biases, whereas our baseline uses calibrated
`sensor_combined` without extra EKF bias correction. Record the source and
calibration convention to prevent applying corrections twice. Router forwarding
does not itself request faster streams; stream requests are a separate operation.

## External X20 recording and bandwidth

Preserve the complete `RXM-RAWX` header and each observation: receiver TOW, week,
leap/time-valid and clock-reset flags; GNSS/SV/signal/frequency IDs; carrier,
pseudorange and Doppler; C/N0; lock time; standard-deviation encodings; tracking,
carrier-valid and half-cycle flags. Also retain the receiver navigation packets
and, when supported/configured, `RXM-SFRBX`, `NAV-SAT`, `NAV-PVT`, `TIM-TP` and
port/communication health. Receiver model/firmware and enabled port/message
settings belong in the manifest. Reference:
[u-blox X20 interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-X20-HPG-2.10_InterfaceDescription_UBXDOC-304424225-21263.pdf).

The concrete proposed settings are in
[`acquisition/x20_recording_profile.json`](acquisition/x20_recording_profile.json).
They have **not been applied**. They retain UART1 at 921600, use 50 ms measurement
intervals (20 Hz), preserve RAWX every epoch, disable high-volume redundant
NAV-SIG/RXM-MEASX and position/velocity/status duplicates, and retain NAV-PVT,
NAV-SAT and communication health at 1 Hz plus navigation subframes/timepulse
messages. Their message rate values count navigation epochs; timepulse and
subframe outputs are additionally limited by their source event availability.
Read the current receiver configuration back and archive it before deployment.

Choose a stable measured rate before increasing it. A starting target is 20 Hz
RAWX if supported by the actual X20 firmware/configuration, with much lower rates
for navigation/status messages. The July serial setting was 921600 baud. For an
8N1 UART that allows at most 92,160 bytes/s. RAWX occupies `24 + 32*N` bytes
including framing, where N is the number of signal observations. Thus:

| Observations/epoch | At 25 Hz | Share of 921600-baud UART capacity |
|---:|---:|---:|
| 100 | 80,600 bytes/s | 87.5% |
| 109 | 87,800 bytes/s | 95.3% |

Other messages, processing and jitter need capacity too. Current saturation is
measured; its contribution to the old July delay is not established independently.
The saved seven L5 columns do not imply that the receiver transmitted only seven
observations. If a USB-to-UART bridge is used, account for the receiver UART
rate; if native receiver USB is used, inspect that path's actual constraints.
Measure packet latency growth and receiver transmit-buffer statistics under the
chosen message configuration. Avoid storing only one band just to mask overload.

## Establish two clock bridges

1. **PX4 boot ↔ Pi monotonic time:** record actual MAVLink TIMESYNC exchanges and
   their round-trip/offset estimates. Keep source sensor timestamps and log
   offset changes/rejections. Protocol reference:
   [MAVLink time synchronization](https://mavlink.io/en/services/timesync.html).
2. **External GNSS epoch ↔ the same clock:** preferably capture the receiver's
   PPS/timepulse on a timestamped input and associate each pulse with its
   reported week/TOW/time-valid information. Exact wiring depends on the X20
   board and timestamp input. PPS wiring was not verified over SSH.

TIMESYNC alone does not establish receiver-to-PX4 measurement timing. Serial or
USB arrival times include transport/queueing delay. Preserve them to diagnose
latency, but do not treat them as the phase measurement epoch. Without a pulse
bridge, retain enough clock messages for a documented software calibration and
an explicit uncertainty estimate; do not assume sub-millisecond alignment.

## Bench acceptance before another flight

Use a two-minute ground recording first. Include 30–60 seconds stationary, a
short set of controlled orientation changes, then stationary data again. Keep
the whole matching ULog, raw receiver and MAVLink session. Check that:

- All expected streams and quality fields are present, with no fixed-column loss.
- Actual source-timestamp rates are sustained; gaps/dropouts are counted.
- Over the run, elapsed external GPS time and elapsed Pi/PX4 time track one
  another apart from measured clock drift; packet delay and queue age do not grow
  continuously. A constant unknown offset remains a separate calibration issue.
- Clock exchanges/pulses show repeatable mapping and its residual uncertainty.
- Body frame, quaternion direction, specific-force units and rigid antenna
  offset are documented. Current confirmed offset is `[0.02, 0, 0]` m FRD.
- Stationary segments support a first sensor/vibration assessment. An airborne
  motor-vibration model will still need evaluation.

For a later flight, record ordinary-flight labels and maneuver timing separately
from detector outputs. Include varied, controlled movement; yaw alone mainly
moves a forward-offset antenna in a plane. The current full-3D gate is a solver
restriction to revisit, not a reason to assume every clean flight must produce
a benign decision. First establish complete, synchronized records.

Only inspection, passive captures and read-only receiver/PX4 queries were
performed for this diagnosis. Router, receiver configuration, PX4 parameters
and Pi recording scripts were not modified. Opening the host serial port used
the same 921600-baud setting as the existing recorder.
