# Native PX4 logging: Stage 4

Stage 3 was approved by “ok continue” on 5 November 2026. The
[Stage 4 review report](../results/acquisition_stage4/report.html) includes actual
native SD-card ULogs, raw external GNSS/MAVLink captures and both movement
confirmations. The aircraft was never armed by these tools.

The user subsequently accepted **189.6 Hz** and selected runtime configuration.
Use [the unified runtime launcher](RUNTIME.md) for new recordings. The startup
override below remains an uninstalled historical proposal; it is not required
for this workflow. The launcher stops both recordings on a disarmed stop and
collects the ULog, so run it again before the next flight/bench session.

## Applied changes and measured limits

- PX4 logging parameters: `SDLOG_PROFILE=17`, `SDLOG_MODE=2`, retain
  `SDLOG_BACKEND=3`. Parameters use PX4's bytewise INT32 MAVLink encoding,
  with identity checks, a write-ahead transaction journal and readback.
- The first native log used the matching firmware's 8 KiB default logger
  buffer. It contained 85 dropout records totaling 17.839 seconds.
- The repeat used a 128 KiB runtime buffer after checking contiguous MCU
  memory. It recorded zero ULog dropouts, with about 38 KiB peak buffer use.
  Only the logger module was restarted; the full autopilot was not rebooted.
- Native `sensor_combined` averaged 189.55 Hz, below the audit's strict
  190 Hz threshold. No IMU timestamp intervals exceeded its gap rule.
  Supplied `vehicle_attitude` measurement timing (`timestamp_sample`) is
  continuous. Its publication-time field has two intervals above the conservative
  jitter threshold, with a maximum of 7.948 ms; those diagnostics remain visible
  separately. The IMU rate check remains REVIEW.
- Sensor selection, calibration counters and quaternion reset fields are
  retained in the ULog and summarized in its JSON audit.

The first external raw session recorded an FTDI USB disconnect/reconnect during
movement. Kernel evidence, the failed session and recovery journals are retained.
The receiver restarted with its old 921600/25 Hz configuration. After the user
secured the connections, the approved RAM-only 20 Hz/2 Mbaud profile was reapplied.
The repeat raw session completed without source errors or recorder drops.

## Current operational state

The raw recorder is stopped. Native SD-card logging was restarted and left
running with mode 2 and the tested 128 KiB runtime buffer. The receiver remains
at 2 Mbaud in RAM. Neither receiver-profile persistence nor logger startup-buffer
persistence has been applied. A receiver power reset can therefore restore its
old baud/profile, as this bench test demonstrated.

The actual PX4 firmware hash is
`cb411db5b54a98442ab974292c81b74c057bdf25`, matching the workspace source. That
source sets `LOGGER_BUF=8` before sourcing `/fs/microsd/etc/config.txt` and then
starting the logger. A read-only NSH check found no existing `etc/config.txt`.
The proposed new file is provided in the review report:

```sh
# detect2: tested native logger buffer, KiB
set LOGGER_BUF 128
```

This startup change is **prepared, not installed**. The selected runtime workflow
reapplies the buffer each recording and leaves startup files untouched. A future
automatic log-from-boot workflow would need its own controlled restart check.

## Inspect and roll back

The actual native file was closed using `logger stop`. On this matching custom
firmware, `logger off` does not close a continuous `-f` log. The stopped ULog
was downloaded before native logging was restarted.

To restore the saved original logging parameters and original logger mode:

```bash
ssh nss@10.15.4.141
cd /home/nss/SpoofingDetection/releases/acquisition_20261105T110249Z
/home/nss/venv/bin/python run_stage4.py restore
```

The rollback verifies the same PX4 identity, restores profile 1/mode 0, retains
backend 3, restarts the logger in its original arm-based mode and saves readback.
It does not alter external receiver settings. Its original parameter baseline
remains in the dated first Stage 4 evidence directory on the Pi.

## Audit locally

```bash
python3 -m detect2.acquisition.ulog_audit native.ulg --output native_audit
```

This audits native data only; it does not integrate displacement or derive
attitude. Native IMU and MAVLink HIGHRES_IMU may have different corrections.
External GNSS-to-PX4 measurement-clock alignment remains UNKNOWN; Stage 5 has not
started. These recordings do not establish a validated spoofing decision.
