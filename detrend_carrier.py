#!/usr/bin/env python3

import argparse
from collections import defaultdict, deque
from queue import Empty, Queue
from statistics import fmean
from threading import Thread
from time import monotonic

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from mavlink_read import carrier_phase_samples
from npy_database import (
    DEFAULT_RECORD_FORMAT,
    DETRENDED_CARRIER_FIELDS,
    DETRENDED_CARRIER_DTYPE,
    detrended_carrier_row,
    session_directory,
    stream_writer,
)


WINDOW = 20
BIAS_WINDOW = 100
MAX_POINTS = 1000
DEFAULT_CHUNK_SIZE = 1000
DEFAULT_FLUSH_ROWS = 100

queue = Queue()


def read_carrier_phase():
    for sample in detrended_carrier_phase_samples():
        queue.put(sample)


def detrend_carrier_samples(carrier_samples):
    windows = defaultdict(lambda: deque(maxlen=WINDOW))
    bias_windows = defaultdict(lambda: deque(maxlen=BIAS_WINDOW))
    last_locktime = {}

    for sample in carrier_samples:
        sat = sample["sat"]
        pr_valid = sample["pr_valid"]
        cp_valid = sample["cp_valid"]
        locktime = sample["locktime"]

        if not pr_valid or not cp_valid:
            windows[sat].clear()
            bias_windows[sat].clear()
            last_locktime.pop(sat, None)
            continue

        if locktime is not None and sat in last_locktime and locktime < last_locktime[sat]:
            windows[sat].clear()
            bias_windows[sat].clear()

        last_locktime[sat] = locktime
        windows[sat].append(sample)

        if len(windows[sat]) < WINDOW:
            continue

        window_mean = fmean(item["carrier"] for item in windows[sat])
        detrended_with_bias = sample["carrier"] - window_mean
        bias_windows[sat].append(detrended_with_bias)
        bias = fmean(bias_windows[sat])

        yield {
            "time": sample["time"],
            "plot_time": monotonic(),
            "sat": sat,
            "carrier": sample["carrier"],
            "detrended": detrended_with_bias - bias,
        }


def raw_carrier_samples(carrier_samples):
    for sample in carrier_samples:
        if not sample["pr_valid"] or not sample["cp_valid"]:
            continue

        yield {
            "time": sample["time"],
            "plot_time": monotonic(),
            "sat": sample["sat"],
            "carrier": sample["carrier"],
            "detrended": sample["carrier"],
        }


def detrended_carrier_phase_samples():
    yield from detrend_carrier_samples(carrier_phase_samples())


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot and record detrended carrier phase from RXM-RAWX."
    )
    parser.add_argument("--record-dir", default="recordings")
    parser.add_argument(
        "--record-format",
        choices=("npy", "csv"),
        default=DEFAULT_RECORD_FORMAT,
        help="record streams as chunked .npy directories or CSV files",
    )
    parser.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE)
    parser.add_argument("--flush-rows", type=int, default=DEFAULT_FLUSH_ROWS)
    return parser.parse_args()


def main():
    args = parse_args()
    detrended = defaultdict(lambda: deque(maxlen=MAX_POINTS))
    times = defaultdict(lambda: deque(maxlen=MAX_POINTS))
    lines = {}
    recording_dir = session_directory(args.record_dir)
    writer = stream_writer(
        recording_dir,
        "detrended_carrier",
        DETRENDED_CARRIER_FIELDS,
        DETRENDED_CARRIER_DTYPE,
        record_format=args.record_format,
        chunk_size=args.chunk_size,
        flush_rows=args.flush_rows,
    )

    print(f"Recording detrended carrier phase in {recording_dir}")
    print(f"Recording format: {args.record_format}")

    Thread(target=read_carrier_phase, daemon=True).start()

    fig, ax = plt.subplots()
    ax.set_title("Detrended carrier phase from RXM-RAWX")
    ax.set_xlabel("GPS time of week (s)")
    ax.set_ylabel("Detrended carrier phase centered around zero (cycles)")
    ax.grid(True)

    def update(_frame):
        while True:
            try:
                sample = queue.get_nowait()
            except Empty:
                break

            sat = sample["sat"]
            time = sample["time"]

            times[sat].append(time)
            detrended[sat].append(sample["detrended"])
            writer.append(detrended_carrier_row(sample))

            if sat not in lines:
                (lines[sat],) = ax.plot([], [], label=sat)
                ax.legend(loc="upper right")

        for sat, line in lines.items():
            line.set_data(times[sat], detrended[sat])

        ax.relim()
        ax.autoscale_view()
        return list(lines.values())

    animation = FuncAnimation(fig, update, interval=100, blit=False)
    try:
        plt.show()
    finally:
        writer.close()


if __name__ == "__main__":
    main()
