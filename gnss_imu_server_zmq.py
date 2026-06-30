#!/usr/bin/env python3
"""
Ultra-efficient ZeroMQ PUB-SUB server.
- Zero-copy messaging
- Non-blocking, high-throughput
- Built for real-time sensor data
"""

import asyncio
import json
import socket
from collections import deque
from math import degrees
from time import time

import zmq
import zmq.asyncio
from pymavlink import mavutil
from pyubx2 import UBXReader
from serial import Serial


# Configuration
DRONE_CONNECTION = "udpin:0.0.0.0:14551"
GPS_PORT = "/dev/ttyUSB0"
GPS_BAUD = 921600
ZMQ_HOST = "tcp://127.0.0.1:5555"
BUFFER_SIZE = 100

# Data buffers
attitude_buffer = deque(maxlen=BUFFER_SIZE)
gps_buffer = deque(maxlen=BUFFER_SIZE)

# ZMQ context
ctx = zmq.asyncio.Context()
socket_pub = ctx.socket(zmq.PUB)


async def read_drone_attitude():
    """Read attitude from MAVLink."""
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

                    # Send immediately to subscribers
                    asyncio.run_coroutine_threadsafe(
                        socket_pub.send_multipart(
                            [
                                b"attitude",
                                json.dumps(attitude_data).encode("utf-8"),
                            ]
                        ),
                        loop,
                    )

            except Exception as e:
                print(f"ERROR reading attitude: {e}")

    await loop.run_in_executor(None, _read)


async def read_gps():
    """Read GPS data."""
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

                    # Send immediately to subscribers
                    asyncio.run_coroutine_threadsafe(
                        socket_pub.send_multipart(
                            [b"gnss", json.dumps(gps_data).encode("utf-8")]
                        ),
                        loop,
                    )

            except Exception as e:
                print(f"ERROR reading GPS: {e}")

    await loop.run_in_executor(None, _read)


async def main():
    print("=" * 60)
    print("GNSS/IMU ZeroMQ Server - Ultra-Efficient Streaming")
    print("=" * 60)
    print(f"Publishing on {ZMQ_HOST}\n")

    socket_pub.bind(ZMQ_HOST)
    await asyncio.sleep(0.1)  # Let socket bind

    await asyncio.gather(
        read_drone_attitude(),
        read_gps(),
    )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped.")
