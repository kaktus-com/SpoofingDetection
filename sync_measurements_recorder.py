#!/usr/bin/env python3

import argparse
from collections import deque
from math import degrees
from threading import Event, Lock, Thread
from time import monotonic, sleep

from detrend_carrier import detrended_carrier_phase_samples
from mavlink_read import DRONE_CONNECTION
from npy_database import (
    NpyChunkWriter,
    SYNCED_MEASUREMENT_DTYPE,
    session_directory,
    synced_measurement_row,
)
from pymavlink import mavutil


ATTITUDE_BUFFER_SIZE = 500
DEFAULT_MAX_TIME_OFFSET = 0.25


class AttitudeSynchronizer:
    def __init__(self, connection=DRONE_CONNECTION, buffer_size=ATTITUDE_BUFFER_SIZE):
        self.connection = connection
        self.attitude_buffer = deque(maxlen=buffer_size)
        self.buffer_lock = Lock()
        self.stop_event = Event()
        self.thread = Thread(target=self.read_attitude, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()

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
):
    yield from sync_with_attitude(
        detrended_carrier_phase_samples(),
        max_time_offset=max_time_offset,
        attitude_buffer_size=attitude_buffer_size,
    )


def sync_with_attitude(
    detrended_carrier_samples,
    max_time_offset=DEFAULT_MAX_TIME_OFFSET,
    attitude_buffer_size=ATTITUDE_BUFFER_SIZE,
):
    synchronizer = AttitudeSynchronizer(buffer_size=attitude_buffer_size)
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
        "--chunk-size",
        type=int,
        default=1000,
        help="number of synchronized rows per .npy chunk",
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
    writer = NpyChunkWriter(
        recording_dir,
        "synced_measurements",
        SYNCED_MEASUREMENT_DTYPE,
        chunk_size=args.chunk_size,
    )

    print(f"Recording synchronized measurements in {recording_dir}")
    print(f"Maximum GPS/IMU time offset: {args.max_time_offset:.3f} s")

    try:
        for sample in synced_measurement_samples(args.max_time_offset):
            writer.append(synced_measurement_row(sample))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        writer.close()


if __name__ == "__main__":
    main()
