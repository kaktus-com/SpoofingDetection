# Drone recording implementation plan

Created: 4 November 2026. Baseline: read-only Pi/X20/PX4 inspection on 3 November.

**Execution rule:** implement one stage, run its automated checks, then pause at
its manual checkpoint. You verify the evidence and explicitly approve proceeding.
A failed or uncertain checkpoint stops dependent stages. Creating this plan does
not deploy code or change the drone.

The target is complete, timestamped data suitable for evaluating detect2. Passing
the recording checks does not guarantee a conclusive spoofing decision.

**Current progress:** Stage 0 was approved by your “ok do next” instruction on
5 November 2026. Stage 1 (**149 tests passing**) was approved by your “do it”
instruction. Stage 2 is deployed in an isolated, hash-verified Pi release and
has completed a 20-second timed capture and a roughly 15-second Ctrl+C capture.
Both Stage 2 archives finalized without recorder-reported dropped bytes, while
the original receiver output retained RAWX gaps. Your “do all of this” instruction
on 5 November approved proceeding, including a 2 Mbaud test.
**Stage 3 was approved by “ok continue”. The user accepted Stage 4's 189.6 Hz
rate and selected runtime configuration, then requested the runtime launcher.**
Open the [Stage 3 review page](results/acquisition_stage3/report.html): the
reduced-message profile was tested for 30 seconds at 921600 baud and for two
minutes at 2 Mbaud, with zero unexpected RAWX intervals. An additional two-minute
repeat corrected the MON-COMMS base-cycle assumption, but exposed a 128-chunk
recorder queue overflow and triggered verified automatic rollback. A revised
release uses 4096 chunks with the same 8 MiB byte cap per stream, and repeats
both tests with 1 Hz MON-COMMS (divisor 1). The failed recording remains available
in the report. Both revised captures passed: the two-minute 2 Mbaud run has
2,399 RAWX epochs, zero unexpected intervals/dropped bytes, 120 health samples,
zero pending/skipped/overrun growth and approximately 35.3% UART utilization.
All automated traffic checks pass; manual Checkpoint 3 was approved on 5 November.
The receiver currently uses temporary RAM settings at **2000000 baud**.
Constellation-enable flags, PX4 logging parameters, legacy scripts and the router
are unchanged. Persistent settings, clock validation and native ULog remain
separate later checkpoints. See the [capture/rollback guide](acquisition/BAUDRATE.md).

The [Stage 4 report](results/acquisition_stage4/report.html) contains native
SD-card ULogs and confirmed manual orientation changes. PX4 now has
`SDLOG_PROFILE=17`, `SDLOG_MODE=2`, backend 3. An initial FTDI/power interruption
reset the receiver's RAM settings; the user secured the connections and the
approved 20 Hz/2 Mbaud profile was reapplied with readback. The failed attempt
is retained. An initial native 8 KiB logger buffer produced dropouts; the repeat
used a memory-checked 128 KiB runtime buffer with zero ULog dropouts and a clean
simultaneous raw capture. Both native IMU and supplied attitude measurement
timestamps are continuous. The actual IMU rate is 189.55 Hz, below the audit's
strict 190 Hz cutoff; the original audit flag remains visible, and the user
explicitly accepted 189.6 Hz. Attitude publication-time jitter
is retained separately from its supplied `timestamp_sample` timing.

The logger was restarted as a module; the full autopilot was not rebooted.
Native logging is left running with the tested 128 KiB runtime buffer. The
firmware startup default is still 8 KiB. A concrete new `etc/config.txt` proposal
sets `LOGGER_BUF=128`; it remains **prepared, not applied**. The user chose runtime
configuration instead, so startup persistence and reboot verification are not
prerequisites for this workflow. The [runtime launcher](acquisition/RUNTIME.md)
reapplies the tested receiver settings and 128 KiB logger buffer, verifies live
streams, then records both. Disconnects fail the session; armed stop defers
native collection until disarm. Stage 5 clock verification has not started.
The launcher is installed as `/home/nss/SpoofingDetection/start_recording`.
All 41 tests and both live tests passed: a timed run and reset-state recovery
followed by Ctrl+C, with zero recorder drops and native files collected. Native
logging stops after normal disarmed finalization; start this command again
before each flight/bench session. See [the launcher review](results/runtime_launcher/report.html) and
[the native logging guide](acquisition/PX4_LOGGING.md).

The [Stage 0 report](results/acquisition_stage0/acquisition_stage0_20261104T152748Z/report.html)
records the backup collected on 4 November.
The verified backup includes eight script/launcher files, all 1,139 indexed PX4
parameters and the 18 receiver RAM keys touched by the proposed profile. Flash
and default-layer values were saved; the BBR-layer query was negatively
acknowledged and its contents remain unknown.

## Baseline setup before Stage 3

- Pi: `ssh nss@10.15.4.141`; repository `/home/nss/SpoofingDetection`;
  Python environment `/home/nss/venv/bin/python`.
- CubeOrange: `/dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00`.
- Router currently runs manually:
  `mavlink-routerd -e 127.0.0.1:14551 /dev/ttyACM0:1500000`.
- X20: ZED-X20P, HPG 2.00, UART1 at 921600 baud, connected through
  `/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DQ00KK6P-if00-port0`.
- Observed receiver output saturates this UART. Measurement interval is 40 ms,
  but RAWX epochs have gaps up to 280 ms. High-rate NAV-SIG and other duplicate
  messages consume bandwidth.
- PX4 currently uses `SDLOG_PROFILE=1`, `SDLOG_MODE=0`, `SDLOG_BACKEND=3`.
- The existing `start` launches the older recorder with a fixed wide CSV layout.
  It discards information needed for later validation.

Recheck these facts at implementation time. Device names, running processes and
settings may have changed since inspection.

## Stage 0 — Inspect and save a rollback baseline

**Work**

1. Confirm the device identities, router process and single ownership of each
   serial device/UDP recording endpoint.
2. Copy the current Pi scripts and launcher to a dated backup. Record their hashes,
   the router command/version and installed Python package versions.
3. Read and save X20 firmware and configuration values for every key the proposed
   profile will touch. Retain available RAM/persistent-layer values separately.
4. Export PX4 parameters and save the three logging parameters above explicitly.
5. Record free storage and the existing output rates, using a short independent
   byte capture. Save the capture and health results instead of only terminal text.

**Manual checkpoint 0**

On the Pi, inspect:

```bash
ls -l /dev/serial/by-id/
ps -eo pid,args | grep '[m]avlink-routerd'
ss -unlp
df -h /home/nss
```

- [ ] Both device identities match the physical hardware.
- [ ] Exactly one router owns the Cube connection; no second recorder owns X20.
- [ ] The backup contains scripts, receiver values, PX4 parameters and hashes.
- [ ] The baseline report reproduces the observed traffic problem or documents
      the current difference.

**Continue only after you approve the baseline.** No settings changed in this stage.

## Stage 1 — Build and test the recorder locally

**Coding work**

Add independent acquisition tools under `detect2/acquisition/`:

| Proposed module | Responsibility |
|---|---|
| `capture.py` | Session creation, process lifecycle and clean shutdown |
| `serial_capture.py` | Read original receiver bytes in chunks; record byte offsets and receive timestamps |
| `mavlink_capture.py` | Archive router datagrams and boundaries before decoding |
| `timesync.py` | Log clock exchanges and offset/round-trip estimates |
| `receiver_config.py` | Read configuration, show a diff, apply RAM-only settings, read back and restore |
| `audit_session.py` | Decode offline; report continuity, rates, quality fields, clocks and file integrity |

These tools are now implemented locally, along with raw protocol decoding,
replay and an offline HTML audit. See the
[implemented command/schema guide](acquisition/README.md). `start_capture` is
a separate launcher; the current `start` is unchanged for rollback.

The proposed manual command interface on the Pi is:

```bash
cd /home/nss/SpoofingDetection
# After Stage 2 deploys the locally tested tools:
./start_capture --duration-s 15
/home/nss/venv/bin/python -m detect2.acquisition.audit_session recordings_v2/SESSION
```

`start_capture` must print the actual session path, device identities and active
profile. The audit command must print its HTML report path and individual PASS,
FAIL or UNKNOWN results. Replace `SESSION` with the path printed by capture.
UNKNOWN must not count as PASS. Use `--duration-s 120` for the two-minute checks
and `--duration-s 600` for the sustained ten-minute test.

Required behavior:

- Save receiver bytes before filtering, requiring attitude or performing fits.
- Keep invalid observations and their original quality flags in the archive.
- Preserve all signal identities/frequencies; make long decoded CSV an export.
- Capture MAVLink independently, including quaternion, IMU and clock messages.
- Give every host receive record a monotonic nanosecond timestamp. Keep source
  measurement timestamps unchanged. Log host clock anchors and reboot identity.
- Use bounded buffering; report overflow and disk errors explicitly. Record queue
  age, input bytes, dropped data and health once per second.
- Flush and finalize a session manifest on normal stop. Mark interrupted or
  incomplete sessions accordingly. Capture must not depend on plotting/detection.

**Automated checks**

Replay synthetic/captured streams, including invalid flags, unknown messages,
missing MAVLink, receiver clock resets, slow writes, queue overflow and Ctrl+C.
Verify byte-for-byte archive preservation, correct indexes, explicit error status
and no silent omission of satellites or frequencies.

**Manual checkpoint 1**

- [ ] Run the documented replay command and open its HTML audit report.
- [ ] Confirm that injected missing/invalid data appear in the report.
- [ ] Confirm that simulated MAVLink interruption does not stop receiver capture.
- [ ] Confirm that the archive matches the replay source hash.
- [ ] Confirm that the help output and documented capture/audit commands agree.

**Continue only after you approve the local recorder.**

## Stage 2 — Deploy alongside the old recorder and make a short Pi capture

**Work**

1. Deploy the tested acquisition files and explicit dependency versions to a
   dated Pi release directory. Record the deployed hashes.
2. Run `start_capture` manually for 15–30 seconds, using the stable device paths
   and existing router. Do not run the old recorder simultaneously.
3. Save the receiver configuration unchanged. The known UART saturation is
   expected to remain visible at this stage.
4. Stop with Ctrl+C; copy the session back and generate its offline HTML audit.

**Manual checkpoint 2**

The [Stage 2 review page](results/acquisition_stage2/report.html) links both live
audits, byte indexes, source hashes and before/after settings. The Ctrl+C audit
deliberately retains lifecycle `stopped`, which its generic “capture session”
check labels FAIL; its separate finalization and archive checks pass. A timed
capture also contains a framing candidate warning; inspect its linked boundary
analysis. Telemetry continuity warnings concern Euler ATTITUDE timestamp jitter;
quaternion and HIGHRES_IMU streams have no gaps under the audit's declared rule.

The post-capture read-only MON-VER identity query timed out once and succeeded
on retry. This did not repeat either capture or write receiver configuration.

- [ ] Receiver and MAVLink archives both grow and contain indexed timestamps.
- [ ] The report shows incoming quaternion and IMU packets, not only Euler angles.
- [ ] Receiver samples remain present when no matching attitude exists.
- [ ] Ctrl+C exits cleanly and the manifest reports capture integrity.
- [ ] Any receiver gaps are distinguished from recorder-side byte loss/overflow.

**Continue only after you approve capture integrity.**

**Rollback:** stop the new recorder and select the backed-up launcher. Do not
start another router or another consumer of the X20 serial device.

## Stage 3 — Reduce X20 traffic temporarily and verify it

**Work**

1. Show the exact current-to-proposed configuration diff using
   [the proposed X20 profile](acquisition/x20_recording_profile.json).
2. After your checkpoint approval, apply only the temporary RAM layer. Initially
   retain UART1 baud, constellations, frequencies and physical connections.
   The 5 November authorization additionally permits a 2 Mbaud UART1 test:
   first compare the reduced-message profile at 921600, then journal the
   RAM-only baud transition, switch the host and verify receiver identity and
   baud readback. Keep constellations unless measured headroom requires a fallback.
3. Target 20 Hz RAWX, lower-rate PVT/SAT/communication health, and navigation/
   timepulse messages. Disable the listed redundant high-rate outputs.
4. Read every changed key back and save before/after values.
5. Run a two-minute raw capture and generate the continuity/bandwidth report.

**Manual checkpoint 3**

- [ ] Readback matches the reviewed profile: 50 ms measurement interval and
      RAWX on every navigation epoch.
- [ ] Interior RAWX epochs are spaced by 50 ms, with no missing or repeated
      epochs in the steady two-minute test. Boundary partial packets are counted
      separately. Receiver clock resets are reported rather than hidden.
- [ ] Total output has headroom: target at most 80% of UART 8N1 capacity:
      92,160 bytes/s at 921600 baud or 200,000 bytes/s at 2 Mbaud.
      If signal count pushes traffic higher, review the transport/rate
      explicitly before proceeding.
- [ ] Receiver transmit backlog is bounded and no longer approaches its previous
      near-full state; host queue age has no sustained growth or overflows.
- [ ] RAWX retains all observations and quality flags, and expected low-rate
      navigation/status packets are present.

These are bench acceptance targets, not measured performance guarantees.
Persistent settings are a separate action **after your approval** of this test.
Following persistence, verify the profile again after a controlled restart.

**Rollback:** explicitly restore the saved touched keys and verify readback.
Do not rely solely on a power cycle or on an assumed persistent configuration.
For the executed Stage 3 release, `run_stage3.py restore` restores both the
message/rate baseline and 921600 baud; the exact command is in the
[Stage 3 report](results/acquisition_stage3/report.html). The unchanged old
`start` assumes 921600 and must not be used against the active 2 Mbaud profile.

## Stage 4 — Enable and verify native PX4 logging

**Work**

1. Show the PX4 parameter diff. Proposed values:
   `SDLOG_PROFILE: 1 → 17`, `SDLOG_MODE: 0 → 2`; retain `SDLOG_BACKEND=3`.
2. After your approval, apply the values and perform the required restart.
3. Record a short stationary/controlled-movement bench session and download its
   actual SD-card `.ulg`.
4. Audit unique sample timestamps, rates, gaps, resets and ULog dropouts. Keep
   supplied attitude and our own displacement integration.

**Manual checkpoint 4**

Open the [Stage 4 review](results/acquisition_stage4/report.html), including both
native files and the failed USB-interrupted capture. Review the measured
189.55 Hz rate against the approximate 200 Hz target, supplied measurement times,
and the prepared startup-buffer override. The runtime 128 KiB result does not
verify that buffer allocation or automatic logging survives a whole-autopilot
restart. The exact payload and read-only proof that the target file is absent
are linked in the report; no startup file has been written.

- [ ] Parameter readback matches the proposal on the running firmware.
- [ ] One ULog covers the stationary periods and movement.
- [ ] Native accelerometer/gyro data are retained around 200 Hz, and supplied
      attitude reaches at least 100 Hz where supported. Record actual rates.
- [ ] Sample timestamps, attitude reset information, sensor selection/status and
      calibration provenance survive export.
- [ ] The report counts gaps/dropouts and shows acceptable continuity for the
      detector windows. Increasing the requested rate alone is not a pass.

If the firmware cannot meet these targets, review its publisher/logger behavior
before proceeding. `HIGHRES_IMU` may have different bias corrections from native
`sensor_combined`; document the distinction and do not apply corrections twice.

**Rollback:** restore the exported PX4 logging values and verify them after any
required restart.

## Stage 5 — Verify the two clock bridges

**5A. Pi ↔ PX4**

Implement actual MAVLink TIMESYNC request/reply handling with distinct recorder
identity, source filtering and request matching. Log the original timestamps,
round trip, estimated offset, rejected exchanges and any reset/reconnect. Use
these estimates to map clocks without rewriting archived source timestamps.

Implemented and bench-tested on 5 November 2026. Open the
[Stage 5A timing review](results/acquisition_stage5a/timesync_20261105T142326Z/report.html).
The three-minute run matched all 1,756 requests, with median RTT 1.040 ms and
p95 RTT 2.737 ms. Held-out fit residual p95 was 0.091 ms; this is not an
independent absolute-accuracy measurement. A separate 1.510-second UDP transport
pause created a new clock segment while RAWX continued at 20 Hz. This tests
transport pause/resume, not physical USB removal. The stationary native log is
continuous; the pause-test native log has two approximately 10 ms intervals in
both IMU and attitude, retained for review, with zero logger dropouts.
Use `./start_recording --timesync` for subsequent recordings. Prior recordings
have not been retroactively synchronized. GNSS epoch alignment remains UNKNOWN.
The following manual checklist remains for your review before 5B.

**Manual checkpoint 5A**

- [ ] A three-minute stationary test shows matched requests and replies.
- [ ] Round-trip/offset plots are available; outliers and resets are visible.
- [ ] A controlled reconnect starts a new clock segment rather than silently
      joining incompatible timestamps. Record capture downtime explicitly.
- [ ] Receiver recording continues throughout the MAVLink interruption.

**Pause for your approval before 5B.**

**5B. External GNSS epoch ↔ the same timebase**

Inspect the X20 board's PPS/timepulse output and available timestamp input.
Prepare a specific pin/voltage/capture plan for manual review before wiring.
Then capture pulse events and associate them with receiver week/TOW and validity
information. Establish how this maps into PX4 boot time through the verified
clock bridge. If hardware pulse capture is unavailable, document and validate a
software timing method with its actual uncertainty.

**Manual checkpoint 5B**

- [ ] Physical pulse capture, source time tags and their association are visible
      in the report. Missing/invalid pulse associations are marked.
- [ ] Estimate clock offset/drift and quantify timing uncertainty, including
      transport asymmetry. TIMESYNC round-trip scatter alone is not proof of
      absolute timing accuracy.
- [ ] Record an independently checked timing error budget. Aim around 1 ms for
      the intended movement tests; revise the target from expected antenna
      speed and acceptable phase error (`phase error ≈ speed × time error / λ`).
- [ ] Do not use a detector outcome or the flight's benign label to select timing.

Without a verified receiver-to-IMU mapping, capture may continue for diagnosis,
but the session remains unsuitable for a validated detector conclusion.

## Stage 6 — Combined ground acceptance and report

**Work**

Capture one complete session: 30–60 seconds stationary, controlled varied
movement, then stationary again. Keep the confirmed `[0.02, 0, 0]` m body FRD
antenna offset. Run the same setup for at least two minutes overall; include a
separate ten-minute stationary run to check sustained disk/queue behavior.

Each final session must contain:

```text
receiver.bin + receiver_receive.csv
mavlink.bin + mavlink_receive.csv
clock_sync.csv
flight.ulg
manifest.json
report.html + machine-readable quality summary
```

**Manual checkpoint 6**

- [ ] Both raw archives, complete ULog and calibration/configuration manifest
      belong to the same physical session, with hashes and time coverage.
- [ ] The HTML report shows receiver throughput/continuity, phase quality,
      IMU/attitude rates, clock mapping and estimated antenna movement.
- [ ] The stationary/movement/stationary sequence appears on the independently
      established common time axis.
- [ ] All earlier bandwidth, recorder-integrity and timing checks still pass
      when everything runs together. No overflow, missing archive bytes or ULog
      dropouts are silently accepted.
- [ ] The ten-minute run has bounded queues and no steadily increasing delay.
- [ ] Actual uncertainty/motion limitations are shown separately from acquisition
      integrity. Inconclusive remains a valid detector outcome.

**Continue only after you approve the complete ground session.**

## Stage 7 — Adopt the new launcher, then record a flight

After your approval, switch `start` to the tested capture launcher and record
the exact software/configuration versions. Keep a clearly named rollback copy.
If automatic startup is wanted, replace the current manual router launch with
one reviewed service configuration in a separate change, followed by a startup
test. Do not introduce a second owner of the Cube or X20 connection.

**Manual checkpoint 7**

- [ ] The normal `start` command creates the expected session with the approved
      profile and reports missing devices/errors clearly.
- [ ] A restart/relaunch test creates a new session and clock segment.
- [ ] Stop, download and report generation work through the normal workflow.

Then record an ordinary, documented flight with stationary data before/after
and controlled varied movement. Review the recording-quality report before
interpreting detector decisions. A solver supporting planar motion is a separate
detector improvement; it is not required to complete the acquisition work.

## Completion evidence

At every checkpoint, retain the exact command used, release hash, before/after
configuration, recording/report paths, observed numbers, your PASS/FAIL and any
rollback performed. Implementation should finish by committing the reviewed code
and documentation and recording the release deployed to the Pi. Pushing or
enabling automatic startup follows explicit deployment instructions.

Until a stage is implemented, this document describes intended work; it does
not assert that the proposed tools or checks already exist. For the inspection
details, see [ACQUISITION.md](ACQUISITION.md).
