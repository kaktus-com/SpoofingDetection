#!/usr/bin/env python3
"""
Client that receives GNSS and IMU data over UDP and plots in real-time using pyqtgraph.
Displays 3 plots for attitude (roll, pitch, yaw) and 4x6 grid for satellite carrier phases.
"""

import json
import socket
from collections import defaultdict, deque
from threading import Thread

import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QGridLayout, QLabel
from PyQt6.QtCore import QTimer


# Configuration
SERVER_HOST = "10.15.4.124"
SERVER_PORT = 5000
BUFFER_SIZE = 1000  # samples to keep in history

# Data storage
attitude_data = {"timestamp": deque(maxlen=BUFFER_SIZE), "roll": deque(maxlen=BUFFER_SIZE), "pitch": deque(maxlen=BUFFER_SIZE), "yaw": deque(maxlen=BUFFER_SIZE)}

# Dictionary to store satellite carrier phases: sat_id -> deque of carrier values
satellite_data = defaultdict(lambda: deque(maxlen=BUFFER_SIZE))
satellite_timestamps = defaultdict(lambda: deque(maxlen=BUFFER_SIZE))

# UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udp_socket.bind((SERVER_HOST, SERVER_PORT))
udp_socket.setblocking(False)


def receive_data():
    """Receive UDP packets from server in background thread."""
    while True:
        try:
            data, addr = udp_socket.recvfrom(65536)
            packet = json.loads(data.decode("utf-8"))

            # Process attitude data
            for att in packet.get("attitude", []):
                attitude_data["timestamp"].append(att["timestamp"])
                attitude_data["roll"].append(att["roll"])
                attitude_data["pitch"].append(att["pitch"])
                attitude_data["yaw"].append(att["yaw"])

            # Process GNSS data
            for gnss in packet.get("gnss", []):
                ts = gnss["timestamp"]
                for meas in gnss.get("measurements", []):
                    sat_id = f"{meas['gnss']}:{meas['sv']}:{meas['sig']}"
                    satellite_data[sat_id].append(meas["carrier"])
                    satellite_timestamps[sat_id].append(ts)

        except BlockingIOError:
            pass
        except Exception as e:
            print(f"Error receiving data: {e}")


class RealTimeMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GNSS/IMU Real-Time Monitor")
        self.setGeometry(100, 100, 1920, 1080)

        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Create grid layout
        grid_layout = QGridLayout()
        main_layout.addLayout(grid_layout)

        # Create attitude plots (top row)
        self.attitude_plots = {}
        for i, (key, label) in enumerate(
            [("roll", "Roll (°)"), ("pitch", "Pitch (°)"), ("yaw", "Yaw (°)")]
        ):
            plot_widget = pg.PlotWidget(title=label)
            plot_widget.setLabel("left", label)
            plot_widget.setLabel("bottom", "Time (s)")
            self.attitude_plots[key] = plot_widget.plot(pen="b")
            grid_layout.addWidget(plot_widget, 0, i)

        # Create satellite carrier phase plots (4x6 grid)
        self.sat_plots = {}
        for row in range(4):
            for col in range(6):
                sat_idx = row * 6 + col
                plot_widget = pg.PlotWidget(
                    title=f"Satellite {sat_idx + 1}"
                )
                plot_widget.setLabel("left", "Carrier Phase (cycles)")
                plot_widget.setLabel("bottom", "Time (s)")
                self.sat_plots[sat_idx] = plot_widget.plot(pen="r")
                grid_layout.addWidget(plot_widget, row + 1, col)

        # Timer for updating plots
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)  # Update every 100ms

        print("Real-time monitor running...")

    def update_plots(self):
        """Update all plots with new data."""
        # Update attitude plots
        if attitude_data["timestamp"]:
            time_axis = list(attitude_data["timestamp"])
            time_zero = time_axis[0] if time_axis else 0

            for key, plot in self.attitude_plots.items():
                if attitude_data[key]:
                    plot.setData(
                        x=[t - time_zero for t in time_axis],
                        y=list(attitude_data[key]),
                    )

        # Update satellite plots
        sorted_sats = sorted(satellite_data.keys())[: 24]  # Get up to 24 satellites

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


def main():
    print("=" * 60)
    print("GNSS/IMU Real-Time Plotter - Starting Client")
    print("=" * 60)
    print(f"Listening for server on {SERVER_HOST}:{SERVER_PORT}...")
    print()

    # Start UDP receiver thread
    Thread(target=receive_data, daemon=True).start()

    # Create and show GUI
    app = QApplication([])
    monitor = RealTimeMonitor()
    monitor.show()

    app.exec()


if __name__ == "__main__":
    main()
