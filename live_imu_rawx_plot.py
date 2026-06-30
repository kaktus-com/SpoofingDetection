#!/usr/bin/env python3

from collections import defaultdict, deque
from math import degrees
from queue import Full, Empty, Queue
from threading import Event, Thread
from time import monotonic

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from detrend_carrier import detrended_carrier_phase_samples
from npy_database import (
    ATTITUDE_DTYPE,
    DETRENDED_CARRIER_DTYPE,
    NpyChunkWriter,
    attitude_row,
    detrended_carrier_row,
    session_directory,
)
from pymavlink import mavutil


DRONE_CONNECTION = "udpin:0.0.0.0:14551"

PLOT_SECONDS = 20
MAX_POINTS = 3000
UPDATE_MS = 50

attitude_queue = Queue(maxsize=3000)
carrier_queue = Queue(maxsize=3000)
stop_event = Event()
recording_dir = session_directory()
attitude_writer = NpyChunkWriter(recording_dir, "attitude", ATTITUDE_DTYPE)
carrier_writer = NpyChunkWriter(
    recording_dir,
    "detrended_carrier",
    DETRENDED_CARRIER_DTYPE,
)

print(f"Recording live plot data in {recording_dir}")


def put_latest(queue, item):
    try:
        queue.put_nowait(item)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        queue.put_nowait(item)


def read_drone_attitude():
    print(f"Listening for drone attitude on {DRONE_CONNECTION}...")
    drone = mavutil.mavlink_connection(DRONE_CONNECTION)

    print("Waiting for drone heartbeat...")
    drone.wait_heartbeat()
    print("Drone connected.")

    while not stop_event.is_set():
        msg = drone.recv_match(type="ATTITUDE", blocking=True, timeout=1)
        if msg is None:
            continue

        put_latest(attitude_queue, {
            "time": monotonic(),
            "roll": degrees(msg.roll),
            "pitch": degrees(msg.pitch),
            "yaw": degrees(msg.yaw),
        })
        attitude_writer.append(attitude_row({
            "host_time": monotonic(),
            "roll": degrees(msg.roll),
            "pitch": degrees(msg.pitch),
            "yaw": degrees(msg.yaw),
        }))


def read_detrended_carrier():
    print("Listening for detrended carrier phase from detrend_carrier.py...")

    for sample in detrended_carrier_phase_samples():
        if stop_event.is_set():
            break

        put_latest(carrier_queue, sample)
        carrier_writer.append(detrended_carrier_row(sample))


class LivePlot(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live MAVLink Attitude + Detrended UBX Carrier Phase")

        self.start_time = monotonic()
        self.attitude_time = deque(maxlen=MAX_POINTS)
        self.attitude_data = {
            "roll": deque(maxlen=MAX_POINTS),
            "pitch": deque(maxlen=MAX_POINTS),
            "yaw": deque(maxlen=MAX_POINTS),
        }
        self.carrier_time = defaultdict(lambda: deque(maxlen=MAX_POINTS))
        self.carrier_data = defaultdict(lambda: deque(maxlen=MAX_POINTS))
        self.carrier_lines = {}

        self.graph = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.graph)

        self.attitude_lines = {}
        self.attitude_plots = []
        attitude_plots = [
            ("roll", "Roll", "deg", "r"),
            ("pitch", "Pitch", "deg", "g"),
            ("yaw", "Yaw", "deg", "b"),
        ]

        for index, (name, title, unit, color) in enumerate(attitude_plots):
            plot = self.graph.addPlot(row=0, col=index, title=title)
            plot.setLabel("left", unit)
            plot.setLabel("bottom", "s")
            plot.showGrid(x=True, y=True, alpha=0.25)
            self.attitude_plots.append(plot)
            self.attitude_lines[name] = plot.plot(pen=pg.mkPen(color, width=1.8))

        self.carrier_plot = self.graph.addPlot(
            row=1,
            col=0,
            colspan=3,
            title="Detrended Carrier Phase",
        )
        self.carrier_plot.setLabel("left", "cycles minus rolling mean")
        self.carrier_plot.setLabel("bottom", "s")
        self.carrier_plot.showGrid(x=True, y=True, alpha=0.25)
        self.carrier_plot.addLegend()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(UPDATE_MS)

    def update_plots(self):
        self.read_queues()

        now = monotonic() - self.start_time
        xmin = max(0, now - PLOT_SECONDS)

        attitude_x = list(self.attitude_time)
        for name, line in self.attitude_lines.items():
            line.setData(attitude_x, list(self.attitude_data[name]))

        for sat, line in self.carrier_lines.items():
            line.setData(list(self.carrier_time[sat]), list(self.carrier_data[sat]))

        for plot in self.attitude_plots:
            plot.setXRange(xmin, max(PLOT_SECONDS, now), padding=0)
        self.carrier_plot.setXRange(xmin, max(PLOT_SECONDS, now), padding=0)

    def read_queues(self):
        while True:
            try:
                sample = attitude_queue.get_nowait()
            except Empty:
                break

            t = sample["time"] - self.start_time
            self.attitude_time.append(t)

            for name in self.attitude_data:
                self.attitude_data[name].append(sample[name])

        while True:
            try:
                sample = carrier_queue.get_nowait()
            except Empty:
                break

            sat = sample["sat"]
            t = sample["plot_time"] - self.start_time

            self.carrier_time[sat].append(t)
            self.carrier_data[sat].append(sample["detrended"])

            if sat not in self.carrier_lines:
                color = pg.intColor(len(self.carrier_lines), hues=32)
                self.carrier_lines[sat] = self.carrier_plot.plot(
                    pen=pg.mkPen(color, width=1.3),
                    name=sat,
                )

    def closeEvent(self, event):
        stop_event.set()
        event.accept()


Thread(target=read_drone_attitude, daemon=True).start()
Thread(target=read_detrended_carrier, daemon=True).start()

app = pg.mkQApp("Live MAVLink Attitude + Detrended UBX Carrier Phase")
window = LivePlot()
window.resize(1500, 950)
window.show()
pg.exec()
stop_event.set()
attitude_writer.close()
carrier_writer.close()
