#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt

from npy_database import DEFAULT_RECORDING_DIR, load_stream


def latest_session(record_dir):
    sessions = sorted(path for path in Path(record_dir).iterdir() if path.is_dir())
    if not sessions:
        raise FileNotFoundError(f"No recording sessions found in {record_dir}")
    return sessions[-1]


def relative_time(values):
    if len(values) == 0:
        return values
    return values - values[0]


def plot_synced_measurements(data, session_dir):
    time = relative_time(data["carrier_sync_time"])
    sats = sorted(set(data["sat"]))

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(14, 9))
    fig.suptitle(f"Synchronized GNSS + IMU recording: {session_dir}")

    for sat in sats:
        sat_data = data[data["sat"] == sat]
        sat_time = sat_data["carrier_sync_time"] - data["carrier_sync_time"][0]
        axes[0].plot(sat_time, sat_data["detrended"], label=sat, linewidth=1.0)

    axes[0].set_ylabel("Detrended carrier (cycles)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", fontsize="small", ncols=2)

    axes[1].plot(time, data["roll"], label="roll", linewidth=1.0)
    axes[1].plot(time, data["pitch"], label="pitch", linewidth=1.0)
    axes[1].plot(time, data["yaw"], label="yaw", linewidth=1.0)
    axes[1].set_ylabel("Attitude (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(time, data["time_offset"], linewidth=1.0)
    axes[2].axhline(0, color="black", linewidth=0.8)
    axes[2].set_ylabel("GNSS - IMU time offset (s)")
    axes[2].set_xlabel("Recording time (s)")
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def print_summary(data, session_dir):
    duration = data["carrier_sync_time"][-1] - data["carrier_sync_time"][0]
    sats = sorted(set(data["sat"]))

    print(f"Session: {session_dir}")
    print(f"Rows: {len(data)}")
    print(f"Duration: {duration:.2f} s")
    print(f"Satellites/signals: {len(sats)}")
    print(f"Time offset min/max: {data['time_offset'].min():.4f} / {data['time_offset'].max():.4f} s")
    print(f"Time offset mean abs: {abs(data['time_offset']).mean():.4f} s")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot synchronized GNSS carrier phase and IMU attitude from .npy recordings."
    )
    parser.add_argument(
        "session",
        nargs="?",
        help="recording session folder; defaults to newest folder in recordings/",
    )
    parser.add_argument(
        "--record-dir",
        default=DEFAULT_RECORDING_DIR,
        help="base recordings directory used when session is omitted",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    session_dir = Path(args.session) if args.session else latest_session(args.record_dir)
    data = load_stream(session_dir, "synced_measurements")

    if data is None or len(data) == 0:
        raise SystemExit(f"No synced_measurements data found in {session_dir}")

    print_summary(data, session_dir)
    plot_synced_measurements(data, session_dir)


if __name__ == "__main__":
    main()
