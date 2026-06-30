#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path
import json

import numpy as np


DEFAULT_RECORDING_DIR = Path("recordings")


DETRENDED_CARRIER_DTYPE = np.dtype([
    ("gps_time", "f8"),
    ("plot_time", "f8"),
    ("sat", "U24"),
    ("carrier", "f8"),
    ("detrended", "f8"),
])

SYNCED_MEASUREMENT_DTYPE = np.dtype([
    ("gps_time", "f8"),
    ("carrier_sync_time", "f8"),
    ("imu_sync_time", "f8"),
    ("time_offset", "f8"),
    ("sat", "U24"),
    ("carrier", "f8"),
    ("detrended", "f8"),
    ("roll", "f8"),
    ("pitch", "f8"),
    ("yaw", "f8"),
    ("mavlink_time_boot_ms", "i8"),
])

DETECTION_EVENT_DTYPE = np.dtype([
    ("event_time", "f8"),
    ("gps_time", "f8"),
    ("sat", "U24"),
    ("detrended", "f8"),
    ("abs_detrended", "f8"),
    ("roll", "f8"),
    ("pitch", "f8"),
    ("yaw", "f8"),
    ("time_offset", "f8"),
    ("reason", "U80"),
])


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
    stream_dir = Path(session_dir) / stream_name
    chunks = sorted(stream_dir.glob("chunk_*.npy"))

    if not chunks:
        return None

    arrays = [np.load(path) for path in chunks]
    return np.concatenate(arrays)


class NpyChunkWriter:
    def __init__(self, session_dir, stream_name, dtype, chunk_size=1000):
        self.stream_dir = Path(session_dir) / stream_name
        self.stream_dir.mkdir(parents=True, exist_ok=True)
        self.dtype = dtype
        self.chunk_size = chunk_size
        self.buffer = []
        self.chunk_index = 0
        self.total_rows = 0
        self.manifest_path = self.stream_dir / "manifest.json"

    def append(self, row):
        self.buffer.append(row)
        if len(self.buffer) >= self.chunk_size:
            self.flush()

    def flush(self):
        if not self.buffer:
            return

        path = self.stream_dir / f"chunk_{self.chunk_index:06d}.npy"
        data = np.array(self.buffer, dtype=self.dtype)
        np.save(path, data)

        self.total_rows += len(self.buffer)
        self.chunk_index += 1
        self.buffer.clear()
        self.write_manifest()

    def write_manifest(self):
        manifest = {
            "stream": self.stream_dir.name,
            "dtype": self.dtype.descr,
            "chunks": self.chunk_index,
            "rows": self.total_rows,
            "chunk_size": self.chunk_size,
        }
        self.manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    def close(self):
        self.flush()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.close()
