#!/usr/bin/env python3
"""
Server that receives both IMU (attitude) and GNSS (GPS) data,
processes them internally, and streams both over UDP to client.
"""

import json
import socket
from math import degrees
from threading import Thread
from time import sleep, time
from collections import deque

from pymavlink import mavutil
from pyubx2 import UBXReader
from serial import Serial


# Configuration
DRONE_CONNECTION = "udpin:0.0.0.0:14551"
GPS_PORT = "/dev/ttyUSB0"
GPS_BAUD = 921600
CLIENT_HOST = "10.15.4.124"
CLIENT_PORT = 5000

# Data buffers (keep last 100 samples)
attitude_buffer = deque(maxlen=100)
gps_buffer = deque(maxlen=100)

# Socket for sending to client
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def read_drone_attitude():
    """Read attitude (roll, pitch, yaw) from MAVLink drone."""
    print(f"Listening for drone on {DRONE_CONNECTION}...")
    try:
        drone = mavutil.mavlink_connection(DRONE_CONNECTION)
        print("Waiting for drone heartbeat...")
        drone.wait_heartbeat()
        print("Drone connected.")
    except Exception as e:
        print(f"ERROR: Failed to connect to drone: {e}")
        return

    while True:
        try:
            msg = drone.recv_match(type="ATTITUDE", blocking=True, timeout=2)
            if msg:
                timestamp = time()
                roll = degrees(msg.roll)
                pitch = degrees(msg.pitch)
                yaw = degrees(msg.yaw)

                attitude_data = {
                    "timestamp": timestamp,
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                }
                attitude_buffer.append(attitude_data)

                print(
                    f"ATT  roll={roll:7.2f}  pitch={pitch:7.2f}  yaw={yaw:7.2f}"
                )
        except Exception as e:
            print(f"ERROR reading attitude: {e}")
            sleep(0.1)


def read_gps():
    """Read GPS carrier phase data from u-blox."""
    print(f"Listening for GPS on {GPS_PORT}...")
    try:
        gps = Serial(GPS_PORT, GPS_BAUD, timeout=1)
        reader = UBXReader(gps)
        print(f"Serial connection opened successfully on {GPS_PORT}")
    except Exception as e:
        print(f"ERROR: Failed to open serial port {GPS_PORT}: {e}")
        return

    while True:
        try:
            raw, msg = reader.read()

            if not msg or msg.identity != "RXM-RAWX":
                continue

            timestamp = time()
            measurements = []

            for i in range(1, msg.numMeas + 1):
                gnss = getattr(msg, f"gnssId_{i:02}", None)
                sv = getattr(msg, f"svId_{i:02}", None)
                sig = getattr(msg, f"sigId_{i:02}", 0)
                carrier = getattr(msg, f"cpMes_{i:02}", None)
                locktime = getattr(msg, f"locktime_{i:02}", None)
                pr_valid = getattr(msg, f"prValid_{i:02}", None)
                cp_valid = getattr(msg, f"cpValid_{i:02}", None)

                if carrier is not None:
                    measurements.append(
                        {
                            "gnss": gnss,
                            "sv": sv,
                            "sig": sig,
                            "carrier": float(carrier),
                            "locktime": locktime,
                            "pr_valid": pr_valid,
                            "cp_valid": cp_valid,
                        }
                    )

            if measurements:
                gps_data = {
                    "timestamp": timestamp,
                    "rcvTow": float(msg.rcvTow),
                    "numMeas": len(measurements),
                    "measurements": measurements,
                }
                gps_buffer.append(gps_data)

                print(
                    f"GNSS  rcvTow={msg.rcvTow:.3f}  measurements={len(measurements)}"
                )

        except Exception as e:
            print(f"ERROR reading GPS: {e}")
            sleep(0.1)


def send_to_client():
    """Send buffered IMU and GNSS data to client over UDP."""
    print(f"Streaming data to {CLIENT_HOST}:{CLIENT_PORT}...")

    while True:
        try:
            if attitude_buffer or gps_buffer:
                packet = {
                    "attitude": list(attitude_buffer),
                    "gnss": list(gps_buffer),
                }
                data = json.dumps(packet).encode("utf-8")
                client_socket.sendto(data, (CLIENT_HOST, CLIENT_PORT))

            sleep(0.05)  # Send at ~20 Hz
        except Exception as e:
            print(f"ERROR sending to client: {e}")
            sleep(0.1)


def main():
    print("=" * 60)
    print("GNSS/IMU Server - Receiving and Streaming Data")
    print("=" * 60)

    # Start reader threads
    Thread(target=read_drone_attitude, daemon=True).start()
    Thread(target=read_gps, daemon=True).start()
    Thread(target=send_to_client, daemon=True).start()

    print("Server running. Press Ctrl+C to stop.\n")

    try:
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("\nServer stopped.")


if __name__ == "__main__":
    main()
