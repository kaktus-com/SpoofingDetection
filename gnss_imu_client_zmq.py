#!/usr/bin/env python3
"""
ZeroMQ client for GNSS/IMU real-time plotting.
Subscribes to attitude and GNSS data streams.
"""

import asyncio
import json
from collections import defaultdict, deque
from threading import Thread

import zmq
import zmq.asyncio
import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QGridLayout
from PyQt6.QtCore import QTimer


# Configuration
ZMQ_HOST = "tcp://127.0.0.1:5555"
BUFFER_SIZE = 1000

# Data storage
attitude_data = {
    "timestamp": deque(maxlen=BUFFER_SIZE),
    "roll": deque(maxlen=BUFFER_SIZE),
    "pitch": deque(maxlen=BUFFER_SIZE),
    "yaw": deque(maxlen=BUFFER_SIZE),
}

satellite_data = defaultdict(lambda: deque(maxlen=BUFFER_SIZE))
satellite_timestamps = defaultdict(lambda: deque(maxlen=BUFFER_SIZE))


async def receive_data():
    """Subscribe to ZMQ streams (attitude and gnss)."""
    ctx = zmq.asyncio.Context()
    socket = ctx.socket(zmq.SUB)
    socket.connect(ZMQ_HOST)

    # Subscribe to both topics
    socket.subscribe(b"attitude")
    socket.subscribe(b"gnss")

    print(f"Connected to server on {ZMQ_HOST}")

    while True:
        try:
            topic, data = await socket.recv_multipart()
            topic = topic.decode("utf-8")
            packet = json.loads(data.decode("utf-8"))

            if topic == "attitude":
                attitude_data["timestamp"].append(packet["timestamp"])
                attitude_data["roll"].append(packet["roll"])
                attitude_data["pitch"].append(packet["pitch"])
                attitude_data["yaw"].append(packet["yaw"])

            elif topic == "gnss":
                ts = packet["timestamp"]
                for meas in packet.get("measurements", []):
                    sat_id = f"{meas['gnss']}:{meas['sv']}:{meas['sig']}"
                    satellite_data[sat_id].append(meas["carrier"])
                    satellite_timestamps[sat_id].append(ts)

        except Exception as e:
            print(f"Error receiving: {e}")


class RealTimeMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GNSS/IMU Real-Time Monitor (ZMQ)")
        self.setGeometry(100, 100, 1920, 1080)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        grid_layout = QGridLayout()
        main_layout.addLayout(grid_layout)

        # Attitude plots (top row)
        self.attitude_plots = {}
        for i, (key, label) in enumerate(
            [("roll", "Roll (°)"), ("pitch", "Pitch (°)"), ("yaw", "Yaw (°)")]
        ):
            plot_widget = pg.PlotWidget(title=label)
            plot_widget.setLabel("left", label)
            plot_widget.setLabel("bottom", "Time (s)")
            self.attitude_plots[key] = plot_widget.plot(pen="b")
            grid_layout.addWidget(plot_widget, 0, i)

        # Satellite carrier phase plots (4x6 grid)
        self.sat_plots = {}
        for row in range(4):
            for col in range(6):
                sat_idx = row * 6 + col
                plot_widget = pg.PlotWidget(title=f"Satellite {sat_idx + 1}")
                plot_widget.setLabel("left", "Carrier Phase (cycles)")
                plot_widget.setLabel("bottom", "Time (s)")
                self.sat_plots[sat_idx] = plot_widget.plot(pen="r")
                grid_layout.addWidget(plot_widget, row + 1, col)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)  # Update every 50ms

    def update_plots(self):
        """Update plots with latest data."""
        if attitude_data["timestamp"]:
            time_axis = list(attitude_data["timestamp"])
            time_zero = time_axis[0] if time_axis else 0

            for key, plot in self.attitude_plots.items():
                if attitude_data[key]:
                    plot.setData(
                        x=[t - time_zero for t in time_axis],
                        y=list(attitude_data[key]),
                    )

        sorted_sats = sorted(satellite_data.keys())[: 24]

        for sat_idx, sat_id in enumerate(sorted_sats):
            if sat_idx < len(self.sat_plots):
                if satellite_data[sat_id]:
                    time_vals = list(satellite_timestamps[sat_id])
                    if time_vals:
                        time_zero = time_vals[0]
                        self.sat_plots[sat_idx].setData(
                            x=[t - time_zero for t in time_vals],
                            y=list(satellite_data[sat_id]),
                        )
                        self.sat_plots[sat_idx].getPlotItem().setTitle(sat_id)


async def main_async():
    """Run asyncio receiver in background."""
    await receive_data()


def main():
    print("=" * 60)
    print("GNSS/IMU Real-Time Plotter (ZeroMQ)")
    print("=" * 60)
    print()

    # Start ZMQ receiver in background thread
    def run_zmq():
        asyncio.run(main_async())

    receiver_thread = Thread(target=run_zmq, daemon=True)
    receiver_thread.start()

    # Start GUI
    app = QApplication([])
    monitor = RealTimeMonitor()
    monitor.show()

    app.exec()


if __name__ == "__main__":
    main()
