#!/usr/bin/env python3

from math import degrees
from queue import Empty, Queue
from threading import Event
from threading import Thread
from time import sleep

from pymavlink import mavutil
from pyubx2 import UBXReader
from serial import Serial, SerialException


DRONE_CONNECTION = "udpin:0.0.0.0:14551"
GPS_PORT = "/dev/ttyUSB0"
GPS_BAUD = 921600
GPS_READ_SIZE = 65536
GPS_QUEUE_CHUNKS = 4096
GPS_READ_TIMEOUT = 0.05


class ThreadedSerialStream:
    def __init__(
        self,
        port=GPS_PORT,
        baud=GPS_BAUD,
        read_size=GPS_READ_SIZE,
        queue_chunks=GPS_QUEUE_CHUNKS,
        timeout=GPS_READ_TIMEOUT,
    ):
        self.serial = Serial(port, baud, timeout=timeout)
        self.read_size = read_size
        self.timeout = timeout
        self.queue = Queue(maxsize=queue_chunks)
        self.buffer = bytearray()
        self.stop_event = Event()
        self.read_error = None
        self.total_bytes = 0
        self.thread = Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        read_buffer = bytearray(self.read_size)

        try:
            while not self.stop_event.is_set():
                count = self.serial.readinto(read_buffer)
                if not count:
                    sleep(0)
                    continue

                self.total_bytes += count
                self.queue.put(bytes(memoryview(read_buffer)[:count]))
        except (OSError, SerialException, TypeError) as exc:
            if not self.stop_event.is_set():
                self.read_error = exc

    def _fill_buffer(self, size=None):
        while len(self.buffer) < size and not self.stop_event.is_set():
            try:
                self.buffer.extend(self.queue.get(timeout=self.timeout))
            except Empty:
                if self.read_error is not None:
                    raise self.read_error
                if self.buffer:
                    break
                return b""

        return None

    def read(self, size=1):
        if size is None or size < 0:
            size = 1

        self._fill_buffer(size)

        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        return result

    def readline(self, size=-1):
        while not self.stop_event.is_set():
            newline_index = self.buffer.find(b"\n")
            if newline_index >= 0:
                end = newline_index + 1
                break

            if size is not None and size > 0 and len(self.buffer) >= size:
                end = size
                break

            try:
                self.buffer.extend(self.queue.get(timeout=self.timeout))
            except Empty:
                if self.read_error is not None:
                    raise self.read_error
                if not self.buffer:
                    return b""
                end = len(self.buffer)
                break
        else:
            end = len(self.buffer)

        if size is not None and size > 0:
            end = min(end, size)

        result = bytes(self.buffer[:end])
        del self.buffer[:end]
        return result

    def close(self):
        self.stop_event.set()
        self.serial.close()
        self.thread.join(timeout=1)


def carrier_phase_samples(
    port=GPS_PORT,
    baud=GPS_BAUD,
    threaded=True,
    read_size=GPS_READ_SIZE,
    queue_chunks=GPS_QUEUE_CHUNKS,
):
    if threaded:
        gps = ThreadedSerialStream(
            port=port,
            baud=baud,
            read_size=read_size,
            queue_chunks=queue_chunks,
        )
    else:
        gps = Serial(port, baud, timeout=1)

    reader = UBXReader(gps)

    try:
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
    finally:
        gps.close()


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
