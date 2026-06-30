#!/usr/bin/env python3

import argparse
from math import degrees
from threading import Thread
from time import monotonic, sleep

from npy_database import (
    ATTITUDE_DTYPE,
    GPS_RAWX_DTYPE,
    NpyChunkWriter,
    attitude_row,
    gps_rawx_row,
    session_directory,
)
from pymavlink import mavutil
from pyubx2 import UBXReader
from serial import Serial


DRONE_CONNECTION = "udpin:0.0.0.0:14551"
GPS_PORT = "/dev/ttyUSB0"
GPS_BAUD = 921600


def carrier_phase_samples():
    gps = Serial(GPS_PORT, GPS_BAUD, timeout=1)
    reader = UBXReader(gps)

    while True:
        raw, msg = reader.read()

        if not msg or msg.identity != "RXM-RAWX":
            continue

        for i in range(1, msg.numMeas + 1):
            gnss = getattr(msg, f"gnssId_{i:02}")
            sv = getattr(msg, f"svId_{i:02}")
            sig = getattr(msg, f"sigId_{i:02}", 0)
            pr_valid = getattr(msg, f"prValid_{i:02}", None)
            cp_valid = getattr(msg, f"cpValid_{i:02}", None)
            trk_stat = getattr(msg, f"trkStat_{i:02}", None)

            yield {
                "host_time": monotonic(),
                "time": msg.rcvTow,
                "sat": f"{gnss}:{sv}:{sig}",
                "carrier": getattr(msg, f"cpMes_{i:02}"),
                "locktime": getattr(msg, f"locktime_{i:02}", None),
                "pr_valid": pr_valid,
                "cp_valid": cp_valid,
                "trk_stat": trk_stat,
            }


def read_drone_attitude(writer=None):
    print(f"Listening for drone on {DRONE_CONNECTION}...")
    drone = mavutil.mavlink_connection(DRONE_CONNECTION)

    print("Waiting for drone heartbeat...")
    drone.wait_heartbeat()
    print("Drone connected.")

    while True:
        msg = drone.recv_match(type="ATTITUDE", blocking=True)

        sample = {
            "host_time": monotonic(),
            "roll": degrees(msg.roll),
            "pitch": degrees(msg.pitch),
            "yaw": degrees(msg.yaw),
        }

        if writer is not None:
            writer.append(attitude_row(sample))

        print(
            f"ATT  roll={sample['roll']:7.2f}  "
            f"pitch={sample['pitch']:7.2f}  yaw={sample['yaw']:7.2f}"
        )


def read_gps(writer=None):
    print(f"Listening for GPS on {GPS_PORT}...")

    for sample in carrier_phase_samples():
        if writer is not None:
            writer.append(gps_rawx_row(sample))

        print(
            f"RAWX  time={sample['time']:.3f}  "
            f"sat={sample['sat']}  carrier={sample['carrier']:14.3f} cycles  "
            f"prValid={sample['pr_valid']} cpValid={sample['cp_valid']}"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read live MAVLink attitude and u-blox RXM-RAWX carrier phase."
    )
    parser.add_argument(
        "--no-record",
        action="store_true",
        help="print live samples without saving .npy chunks",
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
        help="number of rows per .npy chunk",
    )
    parser.add_argument(
        "--attitude",
        action="store_true",
        help="also listen for and record MAVLink ATTITUDE messages",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    writers = []

    if args.no_record:
        session_dir = None
        gps_writer = None
        attitude_writer = None
        print("Recording disabled.")
    else:
        session_dir = session_directory(args.record_dir)
        gps_writer = NpyChunkWriter(
            session_dir,
            "gps_rawx",
            GPS_RAWX_DTYPE,
            chunk_size=args.chunk_size,
        )
        writers.append(gps_writer)

        if args.attitude:
            attitude_writer = NpyChunkWriter(
                session_dir,
                "attitude",
                ATTITUDE_DTYPE,
                chunk_size=args.chunk_size,
            )
            writers.append(attitude_writer)
        else:
            attitude_writer = None

        print(f"Recording .npy chunks in {session_dir}")

    if args.attitude:
        Thread(target=read_drone_attitude, args=(attitude_writer,), daemon=True).start()

    Thread(target=read_gps, args=(gps_writer,), daemon=True).start()

    streams = "GPS and drone attitude" if args.attitude else "GPS"
    print(f"Reading {streams}. Press Ctrl+C to stop.")

    try:
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        for writer in writers:
            writer.close()


if __name__ == "__main__":
    main()
