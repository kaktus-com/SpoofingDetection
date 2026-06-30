#!/usr/bin/env python3

from math import degrees
from threading import Thread
from time import sleep

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
                "time": msg.rcvTow,
                "sat": f"{gnss}:{sv}:{sig}",
                "carrier": getattr(msg, f"cpMes_{i:02}"),
                "locktime": getattr(msg, f"locktime_{i:02}", None),
                "pr_valid": pr_valid,
                "cp_valid": cp_valid,
                "trk_stat": trk_stat,
            }


def read_drone_attitude():
    print(f"Listening for drone on {DRONE_CONNECTION}...")
    drone = mavutil.mavlink_connection(DRONE_CONNECTION)

    print("Waiting for drone heartbeat...")
    drone.wait_heartbeat()
    print("Drone connected.")

    while True:
        msg = drone.recv_match(type="ATTITUDE", blocking=True)

        roll = degrees(msg.roll)
        pitch = degrees(msg.pitch)
        yaw = degrees(msg.yaw)

        print(f"ATT  roll={roll:7.2f}  pitch={pitch:7.2f}  yaw={yaw:7.2f}")


def read_gps():
    print(f"Listening for GPS on {GPS_PORT}...")

    for sample in carrier_phase_samples():
        print(
            f"RAWX  time={sample['time']:.3f}  "
            f"sat={sample['sat']}  carrier={sample['carrier']:14.3f} cycles  "
            f"prValid={sample['pr_valid']} cpValid={sample['cp_valid']}"
        )


def main():
    Thread(target=read_drone_attitude, daemon=True).start()
    Thread(target=read_gps, daemon=True).start()

    print("Reading drone attitude and GPS at the same time. Press Ctrl+C to stop.")

    try:
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
