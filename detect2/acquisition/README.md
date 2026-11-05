# Stage 1: local acquisition tools

These tools implement independent raw receiver and MAVLink capture, offline
auditing and deterministic replay. Stage 1 is local development/testing only.
Pi deployment and changes to the existing `start` launcher wait for their manual
checkpoints. Native SD-card ULog remains the primary inertial record.

## Manual checkpoint 1

From `/home/borna/thesis/SpoofingDetection` on the development computer:

```bash
./start_capture --help
python3 -m detect2.acquisition.replay --help
python3 -m detect2.acquisition.audit_session --help

python3 -m detect2.acquisition.replay \
  --scenario faults --output-root detect2/results/acquisition_stage1/manual_replay
```

Open the report path printed by replay. Run the audit again on that exact
directory using `python3 -m detect2.acquisition.audit_session SESSION_PATH`.
The audit can be repeated without changing the original binary archives.
Its exit status indicates report generation; use the individual PASS/FAIL/
UNKNOWN results to judge quality. A generated report alone is not acceptance.

Check the following:

- Archive/index checks and both source-byte comparisons say **PASS**.
- Receiver continuity says **FAIL**: the input deliberately omits one epoch and
  flags one clock reset. One invalid carrier observation survives in the long
  decoded CSV. All four signal IDs, including multiple bands/constellations,
  remain present. The valid unknown UBX message remains archived and counted.
- MAVLink source continuity says **FAIL**: an injected telemetry interruption is
  visible. The receiver still has **39 epochs / 156 observations**. Quaternions
  and HIGHRES_IMU remain in the MAVLink archive.
- GNSS-to-PX4 timing and native ULog checks say **UNKNOWN**. No timing mapping
  or detector decision is invented.

Other review scenarios:

```bash
python3 -m detect2.acquisition.replay --scenario healthy --output-root /tmp/acq-healthy
python3 -m detect2.acquisition.replay --scenario missing-mavlink --output-root /tmp/acq-missing
python3 -m detect2.acquisition.replay --scenario overflow --output-root /tmp/acq-overflow
```

| Scenario | Expected evidence |
|---|---|
| `healthy` | Archive/continuity PASS; clock and native ULog still UNKNOWN |
| `faults` | Source bytes preserved; known missing/invalid/reset data exposed |
| `missing-mavlink` | Receiver bytes still preserved; missing telemetry makes the session incomplete |
| `overflow` | Bounded buffering rejects input explicitly; failed session, lost-byte counts and source mismatch |

Deliberately red fault checks demonstrate detection of the injection; they are
not a claim that the replay input is good flight data. Each replay directory
contains its original input fixtures and SHA-256 fingerprints for comparison.
No serial device or network connection is opened by replay.

## Live capture interface (use after deployment approval)

```bash
./start_capture --duration-s 120 --expected-rawx-hz 20
```

The launcher chooses `/home/nss/venv/bin/python` on the inspected Pi layout;
otherwise it uses `python3`. `CAPTURE_PYTHON=/path/to/python` overrides this.
Dependencies are pinned in this directory's `requirements.txt` to versions
observed on the Pi. No dependency installation is performed automatically.

Default inputs are the stable FTDI device identity at 921600 baud and UDP
`127.0.0.1:14551`. Specify `--gps-device`, `--gps-baud`, `--mavlink-bind` and
`--output-root` when needed. The expected RAWX rate is a declaration used for
offline gap checking, not a command changing receiver output.

The receiver profile prints UNKNOWN because capture never queries or configures
the receiver. Take a reviewed settings snapshot separately. TIMESYNC transmission
is disabled by default. `--timesync` enables requests/replies only at the later
timing checkpoint; it does not change flight/stream-rate parameters.

Recordings use a new timestamped/unique directory and never overwrite a session:

```text
receiver.bin + receiver_receive.csv   original serial chunks and offsets/times
mavlink.bin + mavlink_receive.csv     original UDP datagrams and boundaries/peers
clock_sync.csv                       empty header when active exchanges disabled
clock_sync.jsonl + mavlink_tx.bin     present when active TIMESYNC is enabled
health.jsonl                         per-second bytes, bounded-buffer state and issues
manifest.json                        source hashes, packages, clocks and completion state
UNFINALIZED                          present until finalization succeeds
```

Ctrl+C drains buffers and finalizes metadata with `stop_reason=sigint` (exit 130).
A stopped/incomplete/failed session is distinct from a completed duration test.
If finalization is impossible, `UNFINALIZED` remains. No success is inferred from
the presence of a binary file alone. Serial reconnects receive new segment IDs;
source errors remain in session history instead of being silently erased.

Buffers are bounded by both chunk count and byte count. Raw acquisition never
waits for attitude, fitting or the optional clock observer. Overflow, short/disk
writes and worker shutdown failure are recorded explicitly. A fatal filesystem
failure can prevent a final manifest update; the initial recording state and
UNFINALIZED marker must then be treated as incomplete. Normal finalization fsyncs
the binary/index archives. This is not a guarantee against arbitrary power loss.

## Receiver configuration tools (future Stage 3)

`python3 -m detect2.acquisition.receiver_config --help` documents separate
`inspect`, `apply-ram` and `restore-ram` operations. Capture never calls them.

- Inspect saves the complete touched-key RAM baseline and a proposed diff.
- Apply requires that baseline, matching reported receiver/transport identity
  and unchanged current settings. It writes a transaction journal **before**
  sending a RAM-only command and verifies readback.
- A failed apply/readback attempts exact baseline restoration and records whether
  rollback succeeded. A lost connection can leave rollback uncertain; the
  journal must then be reviewed.
- Restore refuses unrelated current changes or an incomplete readback. There is
  no flash/BBR persistence command. Output journals cannot be overwritten.

The transport path and reported firmware/model are checked; a unique GNSS
hardware UID is not queried. Physical device identity still belongs to the
manual checkpoint. Stage 1 tests use a fake receiver backend; no real
configuration command has been sent.

## Clock and audit interpretation

Active TIMESYNC matches request token and source IDs, rejects excessive/negative
round trips, records timeouts and starts a new segment on an offset discontinuity.
The relation is `PX4 boot ns = Pi monotonic ns + remote_minus_host_ns`.
The median offset and half-round-trip proxy are estimates, not independent
absolute-accuracy certificates. Legacy installed message fields are supported;
target extensions are checked when decoded. See the
[MAVLink protocol](https://mavlink.io/en/services/timesync.html).

Offline auditing preserves original RAWX values/quality encodings in
`receiver_observations.csv`, reports reset boundaries and records unknown packet
types. MAVLink source gaps are observations, not a network-loss probability:
router filtering and rate changes can also affect source sequence/timestamp
spacing. Native ULog and independent receiver-to-IMU timing remain later checks.

For recorded replay, supply `--receiver FILE` and optionally `--mavlink FILE
--mavlink-index CSV`. The index defines the supplied boundaries. Stage 0's
MAVLink index contains parsed packets, whereas the new live recorder retains
original datagrams. Replay does not recover datagram boundaries absent in its
input. Replay host timestamps are explicitly simulated.

## Automated validation

From `/home/borna/thesis`:

```bash
python3 -m pytest SpoofingDetection/detect2/tests/test_acquisition.py -q
```

Tests cover raw-byte fidelity, all retained flags/bands, corruption, missing
telemetry, slow writes, overflow, disk errors, UDP datagram boundaries, Ctrl+C,
clock matching/reset handling and RAM transaction rollback. UDP/PTY integration
tests use only local simulated peers/devices, never the drone.
