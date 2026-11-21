#!/usr/bin/env python3
# Copyright (C) 2026 Diptopal Basu (embeddedfreedom)
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''
Copyright (C) 2026 Diptopal Basu (embeddedfreedom)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Furnace Controller: 1kHz Compensator / 20kHz Simulation
'''


import sys
import math
import serial
import time
import numpy as np
from PySide6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

# --- MDCS config (socat PTY paths from framework/common/mdcs_config.h) ---
import os
_HERE = os.path.dirname(os.path.abspath(__file__))
for _ in range(8):
    _CAND = os.path.join(_HERE, "framework", "common")
    if os.path.isdir(_CAND):
        sys.path.insert(0, _CAND)
        break
    _HERE = os.path.dirname(_HERE)
from mdcs_config import MDCS_PTY_SIM_GRAPH

# --- CONFIGURATION ---
CHANNELS = 6         # T1, T2, u1, u2, q12, Tamb
FS = 1000            
HISTORY_SEC = 600    
VIEW_SEC = 10        
MAX_SAMPLES = FS * HISTORY_SEC

class SerialWorker(QtCore.QObject):
    data_signal = QtCore.Signal(np.ndarray)
    
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True

    def run(self):
        while self.running:
            try:
                ser = serial.Serial(self.port, 115200, timeout=0.001)
                buffer = ""
                while self.running:
                    if ser.in_waiting:
                        raw = ser.read(ser.in_waiting).decode('utf-8', 'ignore')
                        buffer += raw
                        while "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                            clean = line.strip()
                            if clean.startswith("<") and clean.endswith(">"):
                                try:
                                    values = [float(x) for x in clean[1:-1].split()]
                                    if len(values) >= CHANNELS:
                                        data_arr = np.array(values[:CHANNELS], dtype=np.float64)
                                        self.data_signal.emit(data_arr)
                                except: pass
                    time.sleep(0.0001)
            except Exception:
                time.sleep(1.0) 

class ThermalStripWidget(QtWidgets.QWidget):
    """ Mini 'Infrared Camera' Thermal Strip Widget displaying live temperature profile across the bar """
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(40)
        self.T1 = 25.0
        self.T2 = 25.0

    def update_temps(self, t1, t2):
        self.T1 = t1
        self.T2 = t2
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        rect = self.rect()
        width = rect.width()

        gradient = QtGui.QLinearGradient(0, 0, width, 0)
        
        def get_qcolor(temp):
            t = max(25.0, min(100.0, temp))
            if t <= 45.0:
                f = (t - 25.0) / 20.0
                return QtGui.QColor(int(f * 255), int(128 + f * 127), 255)
            else:
                f = min(1.0, (t - 45.0) / 55.0)
                return QtGui.QColor(255, int(255 * (1.0 - f * 0.7)), int(255 * 0.5 * f))

        gradient.setColorAt(0.0, get_qcolor(self.T1 - 5))
        gradient.setColorAt(0.45, get_qcolor(self.T1))
        gradient.setColorAt(0.55, get_qcolor(self.T2))
        gradient.setColorAt(1.0, get_qcolor(self.T2 + 5))

        painter.fillRect(rect, gradient)
        painter.setPen(QtGui.QPen(QtCore.Qt.white, 2))
        painter.drawRect(rect.adjusted(0, 0, -1, -1))

        painter.setPen(QtCore.Qt.black)
        painter.setFont(QtGui.QFont("Consolas", 11, QtGui.QFont.Bold))
        painter.drawText(rect, QtCore.Qt.AlignCenter, f"LIVE THERMAL FLIR STRIP | Zone 1: {self.T1:.2f}°C  <--->  Zone 2: {self.T2:.2f}°C")

class TelemetryDash(QtWidgets.QMainWindow):
    def __init__(self, port):
        super().__init__()
        self.setWindowTitle("Dual-Zone Coupled Heat Slab Telemetry Dashboard")
        self.resize(1200, 850)

        self.time_data = np.zeros(MAX_SAMPLES)
        self.y_data = np.zeros((CHANNELS, MAX_SAMPLES))
        self.ptr = 0
        self.tick = 0
        self.last_t = 0.0
        
        self.pkts_since_last_update = 0
        self.last_load_check = time.time()
        self.current_hz = 0
        self.auto_scroll = True
        
        self.labels = ["T1 (°C)", "T2 (°C)", "u1 (PWM)", "u2 (PWM)", "Q12 (W)", "Tamb (°C)"]
        self.colors = ['#FF4500', '#00FFFF', '#00FF00', '#FFFF00', '#FF00FF', '#AAAAAA']

        self.init_ui()
        
        self.worker_thread = QtCore.QThread()
        self.worker = SerialWorker(port)
        self.worker.moveToThread(self.worker_thread)
        self.worker.data_signal.connect(self.update_buffers)
        self.worker_thread.started.connect(self.worker.run)
        self.worker_thread.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh_plot)
        self.timer.start(20)

    def init_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        self.flir_strip = ThermalStripWidget()
        layout.addWidget(self.flir_strip)

        self.stats_label = QtWidgets.QLabel("Serial Rate: 0 Hz | Buffer: 0%")
        self.stats_label.setStyleSheet("color: #AAAAAA; font-family: monospace; font-weight: bold;")
        layout.addWidget(self.stats_label, alignment=QtCore.Qt.AlignRight)

        # 3 Synchronized Stacked Subplots
        self.graph_win = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graph_win)

        # Subplot 1: Temperatures
        self.plot1 = self.graph_win.addPlot(row=0, col=0)
        self.plot1.showGrid(x=True, y=True)
        self.plot1.addLegend()
        self.plot1.setLabel('left', 'Temperature', units='°C')
        self.plot1.disableAutoRange(axis=pg.ViewBox.XAxis)
        self.plot1.setXRange(-VIEW_SEC, 0, padding=0)

        # Subplot 2: Controller Effort / PWM
        self.plot2 = self.graph_win.addPlot(row=1, col=0)
        self.plot2.showGrid(x=True, y=True)
        self.plot2.addLegend()
        self.plot2.setLabel('left', 'Control Effort', units='PWM')
        self.plot2.setXLink(self.plot1)
        self.plot2.disableAutoRange(axis=pg.ViewBox.XAxis)
        self.plot2.setXRange(-VIEW_SEC, 0, padding=0)

        # Subplot 3: Heat Flux Q12
        self.plot3 = self.graph_win.addPlot(row=2, col=0)
        self.plot3.showGrid(x=True, y=True)
        self.plot3.addLegend()
        self.plot3.setLabel('left', 'Heat Flux Q12', units='W')
        self.plot3.setLabel('bottom', 'Time', units='s')
        self.plot3.setXLink(self.plot1)
        self.plot3.disableAutoRange(axis=pg.ViewBox.XAxis)
        self.plot3.setXRange(-VIEW_SEC, 0, padding=0)

        # Curves
        self.curve_t1 = self.plot1.plot(pen=pg.mkPen(self.colors[0], width=2), name=self.labels[0])
        self.curve_t2 = self.plot1.plot(pen=pg.mkPen(self.colors[1], width=2), name=self.labels[1])
        self.curve_tamb = self.plot1.plot(pen=pg.mkPen(self.colors[5], style=QtCore.Qt.DashLine, width=1), name=self.labels[5])

        self.curve_u1 = self.plot2.plot(pen=pg.mkPen(self.colors[2], width=1.5), name=self.labels[2])
        self.curve_u2 = self.plot2.plot(pen=pg.mkPen(self.colors[3], width=1.5), name=self.labels[3])

        self.curve_q12 = self.plot3.plot(pen=pg.mkPen(self.colors[4], width=1.5), name=self.labels[4])

        controls = QtWidgets.QHBoxLayout()
        self.checks = []
        for i, label in enumerate(self.labels):
            cb = QtWidgets.QCheckBox(label)
            cb.setChecked(True)
            cb.setStyleSheet(f"color: {self.colors[i]}; font-weight: bold;")
            controls.addWidget(cb)
            self.checks.append(cb)

        self.live_btn = QtWidgets.QPushButton("LIVE")
        self.live_btn.setCheckable(True)
        self.live_btn.setChecked(True)
        self.live_btn.setStyleSheet("background-color: #004400; color: white;")
        self.live_btn.toggled.connect(self.toggle_live_mode)
        controls.addWidget(self.live_btn)

        self.save_btn = QtWidgets.QPushButton("Export CSV")
        self.save_btn.clicked.connect(self.save_data)
        controls.addWidget(self.save_btn)

        self.clear_btn = QtWidgets.QPushButton("Clear Buffer")
        self.clear_btn.clicked.connect(self.clear_buffer)
        controls.addWidget(self.clear_btn)

        layout.addLayout(controls)

    def toggle_live_mode(self, checked):
        self.auto_scroll = checked
        if checked:
            self.live_btn.setText("LIVE")
            self.live_btn.setStyleSheet("background-color: #004400; color: white;")
            for p in [self.plot1, self.plot2, self.plot3]:
                p.setMouseEnabled(x=False, y=True)
        else:
            self.live_btn.setText("PAUSED")
            self.live_btn.setStyleSheet("background-color: #440000; color: white;")
            for p in [self.plot1, self.plot2, self.plot3]:
                p.setMouseEnabled(x=True, y=True)

    def clear_buffer(self):
        self.time_data.fill(0)
        self.y_data.fill(0)
        self.ptr = 0
        self.tick = 0
        self.last_t = 0.0
        self.plot1.setXRange(-VIEW_SEC, 0, padding=0)

    @QtCore.Slot(np.ndarray)
    def update_buffers(self, data):
        t = self.tick * 0.001 
        self.time_data[self.ptr] = t
        self.last_t = t
        
        for i in range(CHANNELS):
            self.y_data[i, self.ptr] = data[i]
            
        self.flir_strip.update_temps(data[0], data[1])

        self.ptr = (self.ptr + 1) % MAX_SAMPLES
        self.tick += 1
        self.pkts_since_last_update += 1

    def refresh_plot(self):
        now = time.time()
        dt = now - self.last_load_check
        if dt >= 1.0:
            self.current_hz = int(self.pkts_since_last_update / dt)
            fill_pct = (min(self.tick, MAX_SAMPLES) / MAX_SAMPLES) * 100
            self.stats_label.setText(f"Serial Rate: {self.current_hz} Hz | Buffer: {fill_pct:.1f}%")
            self.pkts_since_last_update = 0
            self.last_load_check = now

        if self.tick == 0: return
        
        if self.auto_scroll:
            self.plot1.setXRange(self.last_t - VIEW_SEC, self.last_t, padding=0)

        view_range = self.plot1.getViewBox().viewRange()[0]
        xmin, xmax = view_range

        if self.tick < MAX_SAMPLES:
            x_f, y_f = self.time_data[:self.ptr], self.y_data[:, :self.ptr]
        else:
            x_f = np.concatenate((self.time_data[self.ptr:], self.time_data[:self.ptr]))
            y_f = np.concatenate((self.y_data[:, self.ptr:], self.y_data[:, :self.ptr]), axis=1)

        mask = (x_f >= xmin - 0.5) & (x_f <= xmax + 0.5)
        x_v = x_f[mask]
        
        if len(x_v) > 0:
            if self.checks[0].isChecked(): self.curve_t1.setData(x_v, y_f[0][mask])
            else: self.curve_t1.clear()
            if self.checks[1].isChecked(): self.curve_t2.setData(x_v, y_f[1][mask])
            else: self.curve_t2.clear()
            if self.checks[5].isChecked(): self.curve_tamb.setData(x_v, y_f[5][mask])
            else: self.curve_tamb.clear()

            if self.checks[2].isChecked(): self.curve_u1.setData(x_v, y_f[2][mask])
            else: self.curve_u1.clear()
            if self.checks[3].isChecked(): self.curve_u2.setData(x_v, y_f[3][mask])
            else: self.curve_u2.clear()

            if self.checks[4].isChecked(): self.curve_q12.setData(x_v, y_f[4][mask])
            else: self.curve_q12.clear()

    def save_data(self):
        if self.auto_scroll:
            QtWidgets.QMessageBox.warning(self, "Hold on", "Pause the graph before exporting.")
            return
        if self.tick == 0:
            QtWidgets.QMessageBox.information(self, "Nothing to export", "No telemetry received yet.")
            return
        # Use Qt's own file dialog on purpose. The native GTK/portal-backed chooser
        # can block the GUI thread indefinitely on some Linux desktops, which makes
        # the whole window look frozen. DontUseNativeDialog avoids that code path.
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Data", "telemetry.csv", "CSV Files (*.csv)",
            options=QtWidgets.QFileDialog.Option.DontUseNativeDialog)
        if not path:
            return
        if not path.lower().endswith(".csv"):
            path += ".csv"

        if self.tick < MAX_SAMPLES:
            x, y = self.time_data[:self.ptr], self.y_data[:, :self.ptr]
        else:
            x = np.concatenate((self.time_data[self.ptr:], self.time_data[:self.ptr]))
            y = np.concatenate((self.y_data[:, self.ptr:], self.y_data[:, :self.ptr]), axis=1)
        data_to_save = np.vstack((x - x[0], y)).T
        header = "Time_s," + ",".join(self.labels)
        try:
            # %.6f is plenty at 1 kHz and keeps the file ~2.5x smaller than the
            # numpy default (%.18e), which also makes the export much faster.
            np.savetxt(path, data_to_save, delimiter=",", header=header,
                       comments='', fmt="%.6f")
        except OSError as exc:
            QtWidgets.QMessageBox.critical(self, "Export failed", f"Could not write {path}:\n{exc}")
            return
        QtWidgets.QMessageBox.information(self, "Done", f"Export Successful\n{path}")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = TelemetryDash(MDCS_PTY_SIM_GRAPH) 
    win.show()
    sys.exit(app.exec())