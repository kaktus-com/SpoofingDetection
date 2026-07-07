#!/usr/bin/env python3

import argparse
from collections import deque
from math import degrees
from threading import Event, Lock, Thread
from time import monotonic, sleep

from detrend_carrier import detrended_carrier_phase_samples
from mavlink_read import DRONE_CONNECTION
from npy_database import (
    ATTITUDE_MESSAGE_FIELDS,
    ATTITUDE_MESSAGE_DTYPE,
    DEFAULT_RECORD_FORMAT,
    SYNCED_MEASUREMENT_FIELDS,
    SYNCED_MEASUREMENT_DTYPE,
    attitude_message_row,
    session_directory,
    stream_writer,
    synced_measurement_row,
)
from pymavlink import mavutil


ATTITUDE_BUFFER_SIZE = 500
DEFAULT_MAX_TIME_OFFSET = 0.25
DEFAULT_CHUNK_SIZE = 1000
DEFAULT_FLUSH_ROWS = 100


class AttitudeSynchronizer:
    def __init__(
        self,
        connection=DRONE_CONNECTION,
        buffer_size=ATTITUDE_BUFFER_SIZE,
        attitude_writer=None,
    ):
        self.connection = connection
        self.attitude_writer = attitude_writer
        self.attitude_buffer = deque(maxlen=buffer_size)
        self.buffer_lock = Lock()
        self.stop_event = Event()
        self.thread = Thread(target=self.read_attitude, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2)

    def read_attitude(self):
        print(f"Listening for drone attitude on {self.connection}...")
        drone = mavutil.mavlink_connection(self.connection)

        print("Waiting for drone heartbeat...")
        drone.wait_heartbeat()
        print("Drone connected.")

        while not self.stop_event.is_set():
            msg = drone.recv_match(type="ATTITUDE", blocking=True, timeout=1)
            if msg is None:
                continue

            sample = {
                "sync_time": monotonic(),
                "mavlink_time_boot_ms": getattr(msg, "time_boot_ms", -1),
                "roll": degrees(msg.roll),
                "pitch": degrees(msg.pitch),
                "yaw": degrees(msg.yaw),
            }

            with self.buffer_lock:
                self.attitude_buffer.append(sample)

            if self.attitude_writer is not None:
                self.attitude_writer.append(attitude_message_row(sample))

    def has_samples(self):
        with self.buffer_lock:
            return bool(self.attitude_buffer)

    def nearest_attitude_sample(self, target_time):
        with self.buffer_lock:
            if not self.attitude_buffer:
                return None

            return min(
                self.attitude_buffer,
                key=lambda sample: abs(sample["sync_time"] - target_time),
            )


def synced_measurement_sample(carrier_sample, attitude_sample):
    carrier_time = carrier_sample["plot_time"]
    imu_time = attitude_sample["sync_time"]

    return {
        "gps_time": carrier_sample["time"],
        "carrier_sync_time": carrier_time,
        "imu_sync_time": imu_time,
        "time_offset": carrier_time - imu_time,
        "sat": carrier_sample["sat"],
        "carrier": carrier_sample["carrier"],
        "detrended": carrier_sample["detrended"],
        "roll": attitude_sample["roll"],
        "pitch": attitude_sample["pitch"],
        "yaw": attitude_sample["yaw"],
        "mavlink_time_boot_ms": attitude_sample["mavlink_time_boot_ms"],
    }


def synced_measurement_samples(
    max_time_offset=DEFAULT_MAX_TIME_OFFSET,
    attitude_buffer_size=ATTITUDE_BUFFER_SIZE,
    attitude_writer=None,
):
    yield from sync_with_attitude(
        detrended_carrier_phase_samples(),
        max_time_offset=max_time_offset,
        attitude_buffer_size=attitude_buffer_size,
        attitude_writer=attitude_writer,
    )


def sync_with_attitude(
    detrended_carrier_samples,
    max_time_offset=DEFAULT_MAX_TIME_OFFSET,
    attitude_buffer_size=ATTITUDE_BUFFER_SIZE,
    attitude_writer=None,
):
    synchronizer = AttitudeSynchronizer(
        buffer_size=attitude_buffer_size,
        attitude_writer=attitude_writer,
    )
    synchronizer.start()

    try:
        while not synchronizer.has_samples() and not synchronizer.stop_event.is_set():
            sleep(0.05)

        for carrier_sample in detrended_carrier_samples:
            if synchronizer.stop_event.is_set():
                break

            attitude_sample = synchronizer.nearest_attitude_sample(
                carrier_sample["plot_time"]
            )
            if attitude_sample is None:
                continue

            time_offset = abs(carrier_sample["plot_time"] - attitude_sample["sync_time"])
            if time_offset > max_time_offset:
                continue

            yield synced_measurement_sample(carrier_sample, attitude_sample)
    finally:
        synchronizer.stop()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record detrended carrier phase paired with nearest attitude sample."
    )
    parser.add_argument(
        "--record-dir",
        default="recordings",
        help="directory where timestamped recording sessions are saved",
    )
    parser.add_argument(
        "--record-format",
        choices=("npy", "csv"),
        default=DEFAULT_RECORD_FORMAT,
        help="record streams as chunked .npy directories or CSV files",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=DEFAULT_CHUNK_SIZE,
        help="number of synchronized rows per .npy chunk",
    )
    parser.add_argument(
        "--flush-rows",
        type=int,
        default=DEFAULT_FLUSH_ROWS,
        help="number of CSV rows to buffer before flushing",
    )
    parser.add_argument(
        "--max-time-offset",
        type=float,
        default=DEFAULT_MAX_TIME_OFFSET,
        help="maximum allowed GPS/IMU timestamp difference in seconds",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    recording_dir = session_directory(args.record_dir)
    writer = stream_writer(
        recording_dir,
        "synced_measurements",
        SYNCED_MEASUREMENT_FIELDS,
        SYNCED_MEASUREMENT_DTYPE,
        record_format=args.record_format,
        chunk_size=args.chunk_size,
        flush_rows=args.flush_rows,
    )
    attitude_writer = stream_writer(
        recording_dir,
        "attitude_messages",
        ATTITUDE_MESSAGE_FIELDS,
        ATTITUDE_MESSAGE_DTYPE,
        record_format=args.record_format,
        chunk_size=args.chunk_size,
        flush_rows=args.flush_rows,
    )

    print(f"Recording synchronized measurements in {recording_dir}")
    print(f"Recording format: {args.record_format}")
    print("Recording all ATTITUDE messages in attitude_messages")
    print(f"Maximum GPS/IMU time offset: {args.max_time_offset:.3f} s")

    try:
        for sample in synced_measurement_samples(
            args.max_time_offset,
            attitude_writer=attitude_writer,
        ):
            writer.append(synced_measurement_row(sample))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        writer.close()
        attitude_writer.close()


if __name__ == "__main__":
    main()
