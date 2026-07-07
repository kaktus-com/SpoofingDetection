#!/usr/bin/env python3

from csv import writer
from datetime import datetime
from pathlib import Path

import numpy as np


DEFAULT_RECORDING_DIR = Path("recordings")


DETRENDED_CARRIER_FIELDS = (
    "gps_time",
    "plot_time",
    "sat",
    "carrier",
    "detrended",
)

SYNCED_MEASUREMENT_FIELDS = (
    "gps_time",
    "carrier_sync_time",
    "imu_sync_time",
    "time_offset",
    "sat",
    "carrier",
    "detrended",
    "roll",
    "pitch",
    "yaw",
    "mavlink_time_boot_ms",
)

DETECTION_EVENT_FIELDS = (
    "event_time",
    "gps_time",
    "sat",
    "detrended",
    "abs_detrended",
    "roll",
    "pitch",
    "yaw",
    "time_offset",
    "reason",
)


def session_directory(base_dir=DEFAULT_RECORDING_DIR):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = Path(base_dir) / timestamp
    path.mkdir(parents=True, exist_ok=False)
    return path


def detrended_carrier_row(sample):
    return (
        sample["time"],
        sample["plot_time"],
        sample["sat"],
        sample["carrier"],
        sample["detrended"],
    )


def synced_measurement_row(sample):
    return (
        sample["gps_time"],
        sample["carrier_sync_time"],
        sample["imu_sync_time"],
        sample["time_offset"],
        sample["sat"],
        sample["carrier"],
        sample["detrended"],
        sample["roll"],
        sample["pitch"],
        sample["yaw"],
        sample["mavlink_time_boot_ms"],
    )


def detection_event_row(event):
    return (
        event["event_time"],
        event["gps_time"],
        event["sat"],
        event["detrended"],
        event["abs_detrended"],
        event["roll"],
        event["pitch"],
        event["yaw"],
        event["time_offset"],
        event["reason"],
    )


def load_stream(session_dir, stream_name):
    csv_path = Path(session_dir) / f"{stream_name}.csv"
    if csv_path.exists():
        data = np.genfromtxt(
            csv_path,
            delimiter=",",
            names=True,
            dtype=None,
            encoding="utf-8",
        )
        return np.atleast_1d(data)

    return load_legacy_npy_stream(session_dir, stream_name)


def load_legacy_npy_stream(session_dir, stream_name):
    stream_dir = Path(session_dir) / stream_name
    chunks = sorted(stream_dir.glob("chunk_*.npy"))

    if not chunks:
        return None

    arrays = [np.load(path) for path in chunks]
    return np.concatenate(arrays)


class CsvStreamWriter:
    def __init__(self, session_dir, stream_name, fields, flush_rows=100):
        self.path = Path(session_dir) / f"{stream_name}.csv"
        self.fields = fields
        self.flush_rows = flush_rows
        self.rows_since_flush = 0
        self.total_rows = 0
        self.file = self.path.open("w", newline="", encoding="utf-8")
        self.writer = writer(self.file)
        self.writer.writerow(self.fields)
        self.file.flush()

    def append(self, row):
        self.writer.writerow(row)
        self.rows_since_flush += 1
        self.total_rows += 1
        if self.rows_since_flush >= self.flush_rows:
            self.flush()

    def flush(self):
        self.file.flush()
        self.rows_since_flush = 0

    def close(self):
        self.flush()
        self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.close()
