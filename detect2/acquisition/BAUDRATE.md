# X20 UART1 at 2 Mbaud

Stage 3 was authorized on 5 November 2026, including testing 2 Mbaud.
The [live review report](../results/acquisition_stage3/report.html) records the
actual profile, readbacks, captures and current temporary state. Changes use
RAM only; this does not establish persistent startup settings.

The X20 lists 2000000 baud with 8N1 framing; the installed FT230X adapter is
specified up to 3 Mbaud. Capacity at 2 Mbaud is 200,000 bytes/s, versus
92,160 bytes/s at 921600. Validate the complete physical connection with
recorded continuity and receiver communication statistics.

References: [u-blox integration manual](https://content.u-blox.com/sites/default/files/documents/ZED-X20P_IntegrationManual_UBXDOC-963802114-12901.pdf),
[FT230X specifications](https://ftdichip.com/products/ft230xs/).

## Capture at the active receiver baud

Capture never changes the receiver baud or its message profile. Set its host
baud explicitly to match the receiver. On the Pi's dated Stage 3 release:

```bash
cd /home/nss/SpoofingDetection/releases/acquisition_20261105T103442Z
CAPTURE_PYTHON=/home/nss/venv/bin/python ./start_capture \
  --gps-baud 2000000 --expected-rawx-hz 20 --duration-s 120 \
  --output-root /home/nss/SpoofingDetection/recordings_v2
```

Use the existing router and one consumer per recording endpoint. The old
`start` remains unchanged and assumes 921600 baud; restore the baseline before
using it. After a receiver restart, verify the active profile instead of
assuming the temporary settings survived.

## Baud transition utility

`python -m detect2.acquisition.baudrate --help` documents the separate
transition operation. It accepts only 921600 and 2000000 and only writes the
`CFG_UART1_BAUDRATE` RAM key. It journals the original baud and receiver identity
before writing, waits 200 ms, switches the host rate, then verifies identity
and baud readback. An ACK alone is not sufficient across a baud change.

On failure, it probes the two reviewed rates and attempts restoration only on
the matching receiver. Journals explicitly distinguish `rolled_back` from
`recovery_failed`; neither is reported as a successful transition. Existing
journals cannot be overwritten. This utility does not alter constellations,
protocol selection, output rates, BBR or flash.

## Restore the whole Stage 3 baseline

Stop any running recorder, then run on the Pi:

```bash
cd /home/nss/SpoofingDetection/releases/acquisition_20261105T103442Z
/home/nss/venv/bin/python run_stage3.py restore
```

This supervisor uses the saved Stage 3 transaction to restore the original
18 message/rate keys and original 921600 baud, checks the receiver identity,
and verifies the complete saved RAM readback. It preserves restoration journals.
If restoration is uncertain, inspect its error/readback artifacts before
starting a recorder. Do not infer restoration from a power cycle.

## Validation

```bash
python3 -m pytest SpoofingDetection/detect2/tests/test_acquisition.py \
  SpoofingDetection/detect2/tests/test_baudrate.py -q
```

The 21 simulated acquisition/baud tests passed before the final deployment.
An initial live attempt overflowed the old 128-chunk queues and automatically
restored the original receiver settings. The final recorder uses 4096 chunks
and the same 8 MiB byte cap per stream, and records those bounds in its manifest.
The added paused-writer burst test verifies preservation of small packets;
explicit overflow tests still require a failed session and counted losses. Added cases
cover a verified switch and restoration, rejected writes, unreachable receivers,
stale starting rates, protected journals and identity mismatch during recovery.
Actual throughput, continuity and communication counters are separate live checks
in the Stage 3 report. ULog and measurement-clock validation remain later stages.
