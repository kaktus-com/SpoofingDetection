#!/usr/bin/env python3
"""
Non-blocking async server using asyncio.
Handles IMU and GNSS data concurrently without blocking.
"""

import asyncio
import json
import socket
from collections import deque
from math import degrees
from time import time

from pymavlink import mavutil
from pyubx2 import UBXReader
from serial import Serial


# Configuration
DRONE_CONNECTION = "udpin:0.0.0.0:14551"
GPS_PORT = "/dev/ttyUSB0"
GPS_BAUD = 921600
CLIENT_HOST = "10.15.4.124"
CLIENT_PORT = 5000
BUFFER_SIZE = 100

# Data buffers
attitude_buffer = deque(maxlen=BUFFER_SIZE)
gps_buffer = deque(maxlen=BUFFER_SIZE)

# UDP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.setblocking(False)


async def read_drone_attitude():
    """Read attitude from MAVLink (runs in thread pool to avoid blocking)."""
    loop = asyncio.get_event_loop()

    def _read():
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
                msg = drone.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)
                if msg:
                    timestamp = time()
                    roll = degrees(msg.roll)
                    pitch = degrees(msg.pitch)
                    yaw = degrees(msg.yaw)

                    attitude_buffer.append(
                        {
                            "timestamp": timestamp,
                            "roll": roll,
                            "pitch": pitch,
                            "yaw": yaw,
                        }
                    )
                    print(
                        f"ATT  roll={roll:7.2f}  pitch={pitch:7.2f}  yaw={yaw:7.2f}"
                    )
            except Exception as e:
                print(f"ERROR reading attitude: {e}")

    await loop.run_in_executor(None, _read)


async def read_gps():
    """Read GPS data (runs in thread pool to avoid blocking)."""
    loop = asyncio.get_event_loop()

    def _read():
        print(f"Listening for GPS on {GPS_PORT}...")
        try:
            gps = Serial(GPS_PORT, GPS_BAUD, timeout=0.5)
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
                    gps_buffer.append(
                        {
                            "timestamp": timestamp,
                            "rcvTow": float(msg.rcvTow),
                            "numMeas": len(measurements),
                            "measurements": measurements,
                        }
                    )
                    print(
                        f"GNSS  rcvTow={msg.rcvTow:.3f}  measurements={len(measurements)}"
                    )

            except Exception as e:
                print(f"ERROR reading GPS: {e}")

    await loop.run_in_executor(None, _read)


async def send_to_client():
    """Send buffered data to client at maximum rate (no artificial delays)."""
    print(f"Streaming data to {CLIENT_HOST}:{CLIENT_PORT}...\n")

    while True:
        try:
            if attitude_buffer or gps_buffer:
                packet = {
                    "attitude": list(attitude_buffer),
                    "gnss": list(gps_buffer),
                }
                data = json.dumps(packet).encode("utf-8")
                client_socket.sendto(data, (CLIENT_HOST, CLIENT_PORT))

            # Yield control but don't block - send as fast as possible
            await asyncio.sleep(0)

        except Exception as e:
            print(f"ERROR sending to client: {e}")
            await asyncio.sleep(0.1)


async def main():
    print("=" * 60)
    print("GNSS/IMU Async Server - Non-Blocking Architecture")
    print("=" * 60)
    print()

    # Run all tasks concurrently
    await asyncio.gather(
        read_drone_attitude(),
        read_gps(),
        send_to_client(),
    )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped.")
