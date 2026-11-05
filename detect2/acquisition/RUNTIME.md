# One runtime recording command

For the flight-day sequence, use the [short run-and-fly guide](../FLIGHT_QUICKSTART.md).

Start on the ground with PX4 disarmed, the existing router running, and the
receiver USB/power connections secured. Wait for **RECORDING READY** before
arming or beginning a movement test.

```bash
cd /home/nss/SpoofingDetection
./start_recording
# Stop with Ctrl+C. Keep PX4 disarmed while the native ULog is finalized/downloaded.
```

For a timed bench test:

```bash
./start_recording --duration-s 30
```

For active Pi/PX4 clock estimation, add `--timesync`:

```bash
./start_recording --timesync
```

This records addressed 10 Hz TIMESYNC exchanges, original transmit bytes and
receive timestamps, RTT/offset/rejection events and clock segments. Open
`timesync/report.html` after finalization for plots and `timesync/summary.json`
for per-segment clock mappings. A one-second accepted-reply gap or a large
offset jump starts a new segment. No mapping extrapolates across that gap.
READY still means the required sensor streams are present; inspect the timing
report separately. Without this flag, recording remains passive.

The duration starts when the required raw streams are ready. Preparation and
native-log collection take additional time. The command prints the exact session
folder and generates `recording_report.html` there. Defaults store sessions in
`/home/nss/SpoofingDetection/recordings_v2`; override with `--output-root PATH`.

## What it does

1. Locks the receiver and router endpoint against another current detect2
   recorder, checks that no older process owns the UDP endpoint, and verifies
   the disarmed PX4 UID/firmware against `runtime_profile.json`.
2. Probes the receiver at 2 Mbaud and 921600. It checks X20 firmware/model,
   records the RAM baseline, reapplies the tested message profile if necessary,
   switches to 2 Mbaud if needed and verifies readback. All seven
   constellation-enable flags are preserved. No receiver flash/BBR write occurs.
3. Verifies logging parameters 17/2/3, correcting the reviewed logging-only
   values if necessary. PX4 may autosave parameter corrections; this is distinct
   from the explicitly volatile receiver settings and logger buffer.
4. Closes any previous native log on this disarmed aircraft, checks available
   MCU memory, and starts a new native log with a 128 KiB runtime buffer.
   It verifies the actual log path and buffer status before raw recording.
5. Archives original receiver bytes and MAVLink datagrams before live checks.
   It announces READY only after RAWX, quaternion and HIGHRES_IMU are present.
6. On stop, drains/finalizes raw files, closes the bookmarked native log while
   disarmed and downloads it into the same session. It leaves the native logger
   stopped after successful collection. Start this launcher again before the
   next flight or bench recording.

It never arms the aircraft, starts a router, reboots PX4 or writes an SD-card
startup script. The logger's 128 KiB runtime configuration is reapplied each run.
The accepted earlier native rate is 189.6 Hz. No attitude is derived, and no
GNSS/PX4 displacement solution is used in this launcher. Active TIMESYNC estimates
only Pi/PX4 clocks. GNSS-to-PX4 epoch alignment still needs separate validation;
RTT and fit residuals alone do not prove absolute timing accuracy.

## Interruptions and deferred collection

- A receiver exception stops this recording instead of reconnecting at a possibly
  reset baud. GNSS epoch gaps/resets, backwards PX4 source clocks, stale required
  streams, corruption and archive errors produce an explicit failed session.
  Reconfigure only by starting a new recording; previous bytes are retained.
- If stopped while armed, the launcher **does not stop the native log**. The
  session becomes `pending_native`. After disarm, run:

  ```bash
  ./start_recording --finalize /absolute/session/path
  ```

  This checks the same PX4 identity, collects the bookmarked file and preserves
  any newer active native log. It does not reconfigure the receiver. It also
  retries a failed download without overwriting an earlier partial file.
- Ctrl+C normally stops acquisition and then collects the native log. A second
  Ctrl+C during collection defers that step, preserving the partial transfer.
- `UNFINALIZED` describes raw archive finalization. `RUNTIME_UNFINALIZED`
  additionally tracks native collection. Never infer success from a binary file
  alone. Read `runtime_manifest.json` for overall status and issues.

Exit codes: 0 = completed/finalized; 130/143 = clean SIGINT/SIGTERM stop;
2 = failed/incomplete; 3 = native collection still pending. An intentional stop
has status `stopped`. A finalized failed session remains failed even if its
native log was collected successfully.

## Files and review

Each session contains the raw `manifest.json`, indexed receiver/MAVLink archives,
health data, before/after settings and transaction journals, a bookmarked native
SD-card path, downloaded `.ulg`, and `runtime_manifest.json` covering the combined
workflow. Raw file hashes remain unchanged while the native file is collected.
`recording_report.html` links the raw audit, configuration and native file.

The Pi report checks capture/collection integrity, not native sensor-rate
validation or measurement-clock alignment. On a computer with pyulog/numpy:

```bash
python3 -m detect2.acquisition.ulog_audit SESSION/native.ulg --output SESSION/native_audit
```

The launcher is pinned to the reviewed Cube UID and exact custom firmware, plus
X20 HPG 2.00/protocol 50.02. A firmware/hardware change requires reviewing the
profile and logger behavior rather than bypassing the identity checks.

The legacy `start` and raw-only `start_capture` remain available. Run one
recorder at a time. The legacy receiver settings are incompatible with the
2 Mbaud runtime profile until explicitly restored.
