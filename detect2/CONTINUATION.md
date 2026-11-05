# Continuation notes — 5 November 2026

## Current state

- `detect2` contains synthetic geometry/carrier data, the paper's QR detector,
  own acceleration integration using supplied PX4 attitude, native/raw recording
  tools, acquisition-stage reports and three imported November sessions.
- Runtime launcher on the Pi: `./start_recording --timesync`. Pi/PX4 TIMESYNC
  bench validation is complete as an estimated bridge; GNSS epochs are not yet
  linked to IMU measurement time with validated uncertainty.
- Three November raw recordings have been reviewed. Two contain flight; the
  third is stationary. Their original bytes/manifests must remain unchanged.
- [November detector replay](results/november_detector/report.html) runs the
  existing detector with explicit provisional timing and noise sensitivity.
  Read its summary before drawing conclusions; all validated decisions remain
  inconclusive until the missing calibration/timing checks are completed.
- Primary 1 m/s² assumed-noise runs: all 615 complete one-second windows are
  inconclusive. At zero offset the independent stationary-proxy sensitivity
  yields 79 benign-model preferences, 8 spoofed-model preferences and 528
  inconclusive outputs. These are conditional numbers, not verified alarms or
  false-alarm estimates. Check the fixed ±50 ms runs for sensitivity.
- Validation: **187 tests passed**, including nine raw-adapter checks. The HTML
  embeds its figures and interactive window data; PDF/CSV/JSON exports are saved
  alongside it. Source and code fingerprints are recorded per session.
- User confirmed a 3.3 V PPS output and will independently verify the electrical
  path. The exact carrier model is not established. AUX6 is a proposal only.

## Start here on the other computer

1. Pull `borna-branch`; read [HANDOVER.md](HANDOVER.md).
2. Open [the replay report](results/november_detector/report.html) and
   [summary.json](results/november_detector/summary.json). The report distinguishes
   primary gates, optimistic stationary-noise sensitivity, and timing offsets.
3. To reproduce, from the repository root:

   ```bash
   OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m detect2.recorded_raw
   PYTHONPATH=.. python -m pytest detect2/tests -q
   ```

The adapter is [recorded_raw.py](recorded_raw.py), the report generator is
[recorded_raw_report.py](recorded_raw_report.py), and its method/limitations are
in [RECORDED_RAW.md](RECORDED_RAW.md). Outputs are separate from the source review.

## Remaining software for PPS / Stage 5B

No new PX4 capture driver is currently indicated: the installed `pps_capture`
command exists. See [the live parameter/source research](PPS_INPUT_RESEARCH.md).
Do not start it or change pins until the user's hardware/configuration checkpoint.

1. **Reviewed runtime setup and health checks.** Extend the explicit configuration
   guard/profile with the approved PPS input and timer-bank reservation. Snapshot
   and verify values, avoid motor output changes, and handle the required restart
   through a manual checkpoint. Start/check capture before native logging and
   verify 1 Hz edge events actually appear in the ULog. The current launcher
   only configures the reviewed recording parameters; it does not configure PPS.
2. **External pulse-time decoder and association.** RAWX, TIM-TP and SFRBX are
   already recorded. Add tested TIM-TP/timepulse configuration parsing for this
   HPG 2.00 / protocol 50.02 receiver. Verify upcoming-pulse semantics at the
   retained 20 Hz RAWX rate, timebase, week/TOW, validity and quantization fields.
   Pair pulse epochs with native `pps_capture.timestamp`. Do not trust its
   `rtc_timestamp` as X20 time: that field derives from a PX4 GPS topic.
3. **Segmented clock mapping.** Build GNSS week/TOW → PX4 boot time maps with
   explicit unmatched/missing pulses, clock resets, bounded coverage and no
   interpolation across invalid segments. Preserve original timestamps and
   attach measured mapping uncertainty to every replay window.
4. **Replay integration and timing report.** Replace the provisional arrival
   anchor only when the mapping is valid. Gate bad timing windows and plot
   pulse associations, offset/drift, residuals, gaps and an independently checked
   error budget. TIMESYNC fit scatter alone is not absolute accuracy proof.

After these software/hardware checks: combined stationary and varied-motion
ground acceptance, then a new flight recording. Calibrate the field noise and
geometry assumptions before interpreting likelihood preferences as spoofing
decisions. Do not tune timing/noise thresholds to make an expected clean label
come out benign.

## Constraints to preserve

- Use PX4-supplied attitude; do not add a gyro AHRS as a prerequisite.
- Integrate IMU acceleration ourselves; PX4 fused displacement remains deferred.
- The antenna lever arm is `[0.02, 0, 0]` m in body FRD.
- ULog GNSS location is permitted only as a fixed geometry origin, never as
  displacement, timing reference or label-dependent calibration.
- Current November replay uses external NAV-SAT directions and no ULog GNSS.
- Keep receiver settings runtime-only unless the user changes that preference.
- Never use later bench TIMESYNC to retroactively certify earlier flight clocks.
- Preserve manual checkpoints, particularly before PPS wiring/configuration.
- Original `stored_flights/` July inputs and separate PX4 checkout are outside
  this Git repository. Imported November evidence and saved reports are included.
