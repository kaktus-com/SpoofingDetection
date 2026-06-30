#!/usr/bin/env python3

from collections import deque
from queue import Empty, Queue
from statistics import fmean
from threading import Thread
from time import monotonic

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from mavlink_read import carrier_phase_samples


MAX_POINTS = 1000
UPDATE_MS = 50
WINDOW = 20
SATELLITE_COUNT = 7

queue = Queue()


def read_three_satellites():
    selected_sats = []
    windows = {}

    for sample in carrier_phase_samples():
        sat = sample["sat"]

        if sat not in selected_sats and len(selected_sats) < SATELLITE_COUNT:
            selected_sats.append(sat)
            windows[sat] = deque(maxlen=WINDOW)
            print(f"Plotting detrended carrier phase for satellite/signal {sat}")

        if sat not in selected_sats:
            continue

        windows[sat].append(sample["carrier"])

        if len(windows[sat]) < WINDOW:
            continue

        queue.put({
            "time": monotonic(),
            "sat": sat,
            "carrier": sample["carrier"] - fmean(windows[sat]),
        })


class RawCarrierPlot(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Detrended Carrier Phase")

        self.start_time = monotonic()
        self.times = {}
        self.carrier = {}
        self.lines = {}
        self.plots = {}

        self.graph = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.graph)

        for row in range(SATELLITE_COUNT):
            plot = self.graph.addPlot(row=row, col=0, title=f"Satellite {row + 1}")
            plot.setLabel("bottom", "Time", units="s")
            plot.setLabel("left", "Carrier phase minus 20-sample mean", units="cycles")
            plot.showGrid(x=True, y=True, alpha=0.25)
            self.plots[row] = plot

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(UPDATE_MS)

    def update_plot(self):
        while True:
            try:
                sample = queue.get_nowait()
            except Empty:
                break

            sat = sample["sat"]

            if sat not in self.lines:
                index = len(self.lines)
                if index >= SATELLITE_COUNT:
                    continue

                self.times[sat] = deque(maxlen=MAX_POINTS)
                self.carrier[sat] = deque(maxlen=MAX_POINTS)
                self.plots[index].setTitle(f"Satellite/signal {sat}")
                self.lines[sat] = self.plots[index].plot(
                    pen=pg.mkPen("c", width=1.5),
                )

            self.times[sat].append(sample["time"] - self.start_time)
            self.carrier[sat].append(sample["carrier"])

        for sat, line in self.lines.items():
            line.setData(list(self.times[sat]), list(self.carrier[sat]))


Thread(target=read_three_satellites, daemon=True).start()

app = pg.mkQApp("Detrended Carrier Phase")
window = RawCarrierPlot()
window.resize(1000, 1200)
window.show()
pg.exec()
