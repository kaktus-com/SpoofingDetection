#!/usr/bin/env python3

import argparse
from time import monotonic, sleep

from detrend_carrier import detrend_carrier_samples
from mavlink_read import carrier_phase_samples
from npy_database import (
    DETECTION_EVENT_DTYPE,
    SYNCED_MEASUREMENT_DTYPE,
    NpyChunkWriter,
    detection_event_row,
    session_directory,
    synced_measurement_row,
)
from pymavlink import mavutil
from sync_measurements_recorder import (
    DEFAULT_MAX_TIME_OFFSET,
    sync_with_attitude,
)


ALERT_CONNECTION = "udpout:127.0.0.1:14550"
ALERT_MESSAGE = b"GNSS SPOOFING DETECTED"
DEFAULT_DETRENDED_THRESHOLD = 5.0
DEFAULT_CONSECUTIVE_SAMPLES = 3
DEFAULT_CHUNK_SIZE = 1000
DEFAULT_RECORD_DIR = "recordings"


def send_spoofing_message():
    mav = mavutil.mavlink_connection(
        ALERT_CONNECTION,
        source_system=1,
        source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
    )

    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    sleep(1)
    mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_CRITICAL, ALERT_MESSAGE)
    print("Sent:", ALERT_MESSAGE.decode())


def detection_event(sample, threshold, consecutive_count):
    return {
        "event_time": monotonic(),
        "gps_time": sample["gps_time"],
        "sat": sample["sat"],
        "detrended": sample["detrended"],
        "abs_detrended": abs(sample["detrended"]),
        "roll": sample["roll"],
        "pitch": sample["pitch"],
        "yaw": sample["yaw"],
        "time_offset": sample["time_offset"],
        "reason": (
            f"abs(detrended) >= {threshold} cycles for "
            f"{consecutive_count} samples"
        ),
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run GPS detrending, GPS/IMU sync, recording, and spoofing alerts."
    )
    parser.add_argument("--record-dir", default=DEFAULT_RECORD_DIR)
    parser.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE)
    parser.add_argument("--max-time-offset", type=float, default=DEFAULT_MAX_TIME_OFFSET)
    parser.add_argument(
        "--detrended-threshold",
        type=float,
        default=DEFAULT_DETRENDED_THRESHOLD,
    )
    parser.add_argument(
        "--consecutive-samples",
        type=int,
        default=DEFAULT_CONSECUTIVE_SAMPLES,
    )
    parser.add_argument("--no-alert", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    recording_dir = session_directory(args.record_dir)
    counts_by_sat = {}

    synced_writer = NpyChunkWriter(
        recording_dir,
        "synced_measurements",
        SYNCED_MEASUREMENT_DTYPE,
        chunk_size=args.chunk_size,
    )
    event_writer = NpyChunkWriter(
        recording_dir,
        "detection_events",
        DETECTION_EVENT_DTYPE,
        chunk_size=args.chunk_size,
    )

    print(f"Running spoofing detector. Recording in {recording_dir}")
    print(f"Maximum GPS/IMU time offset: {args.max_time_offset:.3f} s")
    print(
        f"Detection rule: abs(detrended) >= {args.detrended_threshold} cycles "
        f"for {args.consecutive_samples} consecutive samples"
    )

    raw_gps_samples = carrier_phase_samples()
    detrended_gps_samples = detrend_carrier_samples(raw_gps_samples)
    synced_samples = sync_with_attitude(
        detrended_gps_samples,
        max_time_offset=args.max_time_offset,
    )

    try:
        for sample in synced_samples:
            synced_writer.append(synced_measurement_row(sample))

            sat = sample["sat"]
            if abs(sample["detrended"]) < args.detrended_threshold:
                counts_by_sat[sat] = 0
                continue

            counts_by_sat[sat] = counts_by_sat.get(sat, 0) + 1
            if counts_by_sat[sat] < args.consecutive_samples:
                continue

            counts_by_sat[sat] = 0
            event = detection_event(
                sample,
                args.detrended_threshold,
                args.consecutive_samples,
            )
            event_writer.append(detection_event_row(event))

            print(
                f"Detection: sat={event['sat']} "
                f"gps_time={event['gps_time']:.3f} "
                f"detrended={event['detrended']:.3f} cycles"
            )

            if not args.no_alert:
                send_spoofing_message()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        synced_writer.close()
        event_writer.close()


if __name__ == "__main__":
    main()
