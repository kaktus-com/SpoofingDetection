# Stage 5A evidence and review

Open `report.html` for the review, interactive timing plots and links to the
native IMU/attitude and raw-receiver audits. Rebuild from the repository root:

```bash
python3 detect2/results/acquisition_stage5a/timesync_20261105T142326Z/review_timesync.py
```

`source_evidence.tar.gz` is the unchanged drone snapshot; its SHA-256 is
`bde97495e3c233bdc985a6feeb7c5cdb708066055e4d1c5de3d27b572f1065c3`.
The script verifies all 66 original evidence files before deriving reports.
Generated reports/observation tables and duplicate FTP partials are omitted from
the snapshot. `evidence/evidence_hashes.json` lists the preserved source files.

The stationary test used the initial isolated release; the separate transport
test used the final release, which adds the bounded mapping helper, test harness
and run instructions. Core exchange handling is the same in both. Both source
manifests and deployment checks are retained. The earlier launcher is preserved
as the rollback release; see `launcher_activation.json` once activated.

The transport test deliberately drops both UDP directions for 1.51 seconds.
It is not a physical receiver/Cube reconnect test. Native sample gaps remain
visible as REVIEW even though clock/receiver-specific checks pass. No PPS wiring
changes or validated GNSS-to-PX4 epoch association are part of Stage 5A.

The three previous flights' 15 original raw files/indexes/manifests were hashed
again and verified unchanged. New bench exchanges must not be extrapolated into
those earlier sessions. No source timestamps are rewritten.
