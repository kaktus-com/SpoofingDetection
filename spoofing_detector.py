#!/usr/bin/env python3

import argparse
from time import monotonic, sleep

from detrend_carrier import detrend_carrier_samples, raw_carrier_samples
from mavlink_read import (
    GPS_BAUD,
    GPS_PORT,
    GPS_QUEUE_CHUNKS,
    GPS_READ_SIZE,
    carrier_phase_samples,
)
from npy_database import (
    ATTITUDE_MESSAGE_FIELDS,
    ATTITUDE_MESSAGE_DTYPE,
    DEFAULT_CSV_LAYOUT,
    DEFAULT_RECORD_FORMAT,
    DETECTION_EVENT_FIELDS,
    DETECTION_EVENT_DTYPE,
    SYNCED_MEASUREMENT_FIELDS,
    SYNCED_MEASUREMENT_DTYPE,
    detection_event_row,
    session_directory,
    stream_writer,
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
DEFAULT_FLUSH_ROWS = 100
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
    parser.add_argument(
        "--record-format",
        choices=("npy", "csv"),
        default=DEFAULT_RECORD_FORMAT,
        help="record streams as chunked .npy directories or CSV files",
    )
    parser.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE)
    parser.add_argument("--flush-rows", type=int, default=DEFAULT_FLUSH_ROWS)
    parser.add_argument(
        "--csv-layout",
        choices=("wide", "long"),
        default=DEFAULT_CSV_LAYOUT,
        help="CSV layout for synced_measurements: one row per GPS epoch or one row per satellite",
    )
    parser.add_argument("--max-time-offset", type=float, default=DEFAULT_MAX_TIME_OFFSET)
    parser.add_argument("--gps-port", default=GPS_PORT)
    parser.add_argument("--gps-baud", type=int, default=GPS_BAUD)
    parser.add_argument("--gps-read-size", type=int, default=GPS_READ_SIZE)
    parser.add_argument("--gps-queue-chunks", type=int, default=GPS_QUEUE_CHUNKS)
    parser.add_argument(
        "--direct-serial",
        action="store_true",
        help="parse directly from pyserial instead of using the threaded UART reader",
    )
    parser.add_argument(
        "--no-detrended",
        "--no-detrededed",
        dest="no_detrended",
        action="store_true",
        help="record raw carrier phase in the detrended field and disable detection",
    )
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

    synced_writer = stream_writer(
        recording_dir,
        "synced_measurements",
        SYNCED_MEASUREMENT_FIELDS,
        SYNCED_MEASUREMENT_DTYPE,
        record_format=args.record_format,
        csv_layout=args.csv_layout,
        chunk_size=args.chunk_size,
        flush_rows=args.flush_rows,
    )
    event_writer = stream_writer(
        recording_dir,
        "detection_events",
        DETECTION_EVENT_FIELDS,
        DETECTION_EVENT_DTYPE,
        record_format=args.record_format,
        csv_layout=args.csv_layout,
        chunk_size=args.chunk_size,
        flush_rows=args.flush_rows,
    )
    attitude_writer = stream_writer(
        recording_dir,
        "attitude_messages",
        ATTITUDE_MESSAGE_FIELDS,
        ATTITUDE_MESSAGE_DTYPE,
        record_format=args.record_format,
        csv_layout=args.csv_layout,
        chunk_size=args.chunk_size,
        flush_rows=args.flush_rows,
    )

    print(f"Running spoofing detector. Recording in {recording_dir}")
    print(f"Recording format: {args.record_format}")
    if args.record_format == "csv":
        print(f"Synced measurement CSV layout: {args.csv_layout}")
    print("Recording all ATTITUDE messages in attitude_messages")
    print(
        f"GPS input: {args.gps_port} at {args.gps_baud} baud "
        f"({'direct' if args.direct_serial else 'threaded'} serial)"
    )
    if not args.direct_serial:
        print(
            f"UART reader: {args.gps_read_size} byte reads, "
            f"{args.gps_queue_chunks} queued chunks"
        )
    print(f"Maximum GPS/IMU time offset: {args.max_time_offset:.3f} s")
    if args.no_detrended:
        print("Carrier mode: raw carrier phase; detection disabled")
    else:
        print(
            f"Detection rule: abs(detrended) >= {args.detrended_threshold} cycles "
            f"for {args.consecutive_samples} consecutive samples"
        )

    raw_gps_samples = carrier_phase_samples(
        port=args.gps_port,
        baud=args.gps_baud,
        threaded=not args.direct_serial,
        read_size=args.gps_read_size,
        queue_chunks=args.gps_queue_chunks,
    )
    if args.no_detrended:
        detrended_gps_samples = raw_carrier_samples(raw_gps_samples)
    else:
        detrended_gps_samples = detrend_carrier_samples(raw_gps_samples)
    synced_samples = sync_with_attitude(
        detrended_gps_samples,
        max_time_offset=args.max_time_offset,
        attitude_writer=attitude_writer,
    )

    try:
        for sample in synced_samples:
            synced_writer.append(synced_measurement_row(sample))

            if args.no_detrended:
                continue

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
        attitude_writer.close()


if __name__ == "__main__":
    main()
