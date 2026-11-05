# Open and continue this work on another computer

The `borna-branch` branch contains the detect2 source, tests, synthetic data,
saved HTML/PDF/plot outputs, and the imported acquisition/November-flight raw
evidence. The two root scripts `start_capture` and `start_recording` are included
because the acquisition launcher depends on them.

```bash
git clone --branch borna-branch git@github.com:kaktus-com/SpoofingDetection.git
cd SpoofingDetection
```

For an existing checkout, switch to `borna-branch` and pull normally after saving
any local edits. The repository includes substantial raw data; allow time and
disk space for the clone. Caches and incomplete `.ftp.part` transfers are excluded.

## View the reports

HTML reports can be opened directly in a browser. Alternatively, from the
repository root:

```bash
python3 -m http.server 8000 --bind 127.0.0.1 --directory detect2/results
```

Open `http://127.0.0.1:8000/` and use these entries:

- [Stage 5A timing review](results/acquisition_stage5a/report.html).
- [Three November recordings](results/recorded_nov05/review_20261105T135817Z/report.html).
- [Stage 4 native IMU/attitude logging](results/acquisition_stage4/report.html).
- [Stage 3 receiver bandwidth](results/acquisition_stage3/report.html).
- [Runtime launcher checks](results/runtime_launcher/report.html).
- [July-flight analysis](results/recorded_july07/report.html).

The original absolute Pi/workstation paths in audit metadata record provenance;
they do not change where the saved relative report links point.

## Run analysis or tests

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r detect2/requirements.txt
python -m pip install -r detect2/acquisition/requirements.txt
PYTHONPATH=.. python -m pytest detect2/tests -q
```

The full suite passed **178 tests** before this handover. Drone acquisition
requires its hardware/router; offline tests use fixtures and local fake peers.
The test imports use the `SpoofingDetection` package name, so the parent of the
default clone directory is added to the Python path for that command.
Rebuild the recent reviews from the repository root with:

```bash
python detect2/results/recorded_nov05/review_20261105T135817Z/review_recordings.py
python detect2/results/acquisition_stage5a/timesync_20261105T142326Z/review_timesync.py
```

Each verifies its preserved evidence fingerprints before generating derived
reports. Original `stored_flights/` data and the separate PX4 source checkout
outside this repository are not included; rebuilding the July-flight adapter
from its original ULog requires those original inputs. Its saved report and
adapted outputs are included.

## Current checkpoint

Pi/PX4 TIMESYNC is deployed; future recording command:
`./start_recording --timesync`. GNSS epoch-to-IMU alignment is still unverified.
The next step is the manual PPS input review in
[PPS_INPUT_RESEARCH.md](PPS_INPUT_RESEARCH.md), followed by a controlled bench
capture. PPS hardware/configuration changes have not been made.
