'''
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
Pendulum Controller: 1kHz Compensator / 20kHz Simulation
'''

import sys
import math
import serial
import time
import numpy as np
from PySide6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

# --- CONFIGURATION ---
CHANNELS = 4         
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

class TelemetryDash(QtWidgets.QMainWindow):
    def __init__(self, port):
        super().__init__()
        self.setWindowTitle("DC Motor with Pendulum Real-Time Telemetry")
        self.resize(1200, 750)

        # Buffers and Sync
        self.time_data = np.zeros(MAX_SAMPLES)
        self.y_data = np.zeros((CHANNELS, MAX_SAMPLES))
        self.ptr = 0
        self.tick = 0
        self.last_t = 0.0
        
        # Load Monitoring
        self.pkts_since_last_update = 0
        self.last_load_check = time.time()
        self.current_hz = 0

        self.auto_scroll = True
        self.use_degrees = False
        
        self.labels = ["Theta", "Theta_Dot", "V_effective", "Torque_Motor"]
        self.colors = ['#00FFFF', '#FF00FF', '#FF4500', '#00FF00']

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

        # Monitor Overlay
        self.stats_label = QtWidgets.QLabel("Serial Rate: 0 Hz | Buffer: 0%")
        self.stats_label.setStyleSheet("color: #AAAAAA; font-family: monospace; font-weight: bold; padding: 5px;")
        layout.addWidget(self.stats_label, alignment=QtCore.Qt.AlignRight)

        self.graph_win = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graph_win)
        self.plot = self.graph_win.addPlot()
        self.plot.showGrid(x=True, y=True)
        self.plot.addLegend()
        self.plot.setLabel('bottom', 'Time', units='s')
        
        self.plot.disableAutoRange(axis=pg.ViewBox.XAxis)
        self.plot.setXRange(-VIEW_SEC, 0, padding=0)
        
        self.vLine = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('w', style=QtCore.Qt.DashLine))
        self.hLine = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen('w', style=QtCore.Qt.DashLine))
        self.plot.addItem(self.vLine, ignoreBounds=True)
        self.plot.addItem(self.hLine, ignoreBounds=True)

        self.tooltip = pg.TextItem(anchor=(0, 1), color='w', fill=(50, 50, 50, 200))
        self.plot.addItem(self.tooltip, ignoreBounds=True)

        self.curves = [self.plot.plot(pen=pg.mkPen(self.colors[i], width=1), name=self.labels[i]) for i in range(CHANNELS)]

        controls = QtWidgets.QHBoxLayout()
        self.checks = []
        for i, label in enumerate(self.labels):
            cb = QtWidgets.QCheckBox(label)
            cb.setChecked(True)
            cb.setStyleSheet(f"color: {self.colors[i]}; font-weight: bold;")
            controls.addWidget(cb)
            self.checks.append(cb)

        self.unit_btn = QtWidgets.QPushButton("Radians")
        self.unit_btn.setCheckable(True)
        self.unit_btn.clicked.connect(self.toggle_units)
        controls.addWidget(self.unit_btn)

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
        self.proxy = pg.SignalProxy(self.plot.scene().sigMouseMoved, rateLimit=60, slot=self.mouse_moved)

    def toggle_units(self):
        self.use_degrees = self.unit_btn.isChecked()
        self.unit_btn.setText("Degrees" if self.use_degrees else "Radians")

    def toggle_live_mode(self, checked):
        self.auto_scroll = checked
        if checked:
            self.live_btn.setText("LIVE")
            self.live_btn.setStyleSheet("background-color: #004400; color: white;")
            self.plot.setMouseEnabled(x=False, y=True)
        else:
            self.live_btn.setText("PAUSED")
            self.live_btn.setStyleSheet("background-color: #440000; color: white;")
            self.plot.setMouseEnabled(x=True, y=True)

    def clear_buffer(self):
        self.time_data.fill(0)
        self.y_data.fill(0)
        self.ptr = 0
        self.tick = 0
        self.last_t = 0.0
        self.plot.setXRange(-VIEW_SEC, 0, padding=0)

    @QtCore.Slot(np.ndarray)
    def update_buffers(self, data):
        # Time is strictly 1ms (0.001s) per packet to ensure 
        # the graph speed matches the actual data flow rate
        t = self.tick * 0.001 
        self.time_data[self.ptr] = t
        self.last_t = t # Update the "current" view time based on data
        
        for i in range(CHANNELS):
            val = data[i]
            if self.use_degrees and i < 2:
                val = math.degrees(val)
            self.y_data[i, self.ptr] = val
            
        self.ptr = (self.ptr + 1) % MAX_SAMPLES
        self.tick += 1
        self.pkts_since_last_update += 1

    def mouse_moved(self, evt):
        pos = evt[0]
        if self.plot.sceneBoundingRect().contains(pos):
            mousePoint = self.plot.vb.mapSceneToView(pos)
            x, y = mousePoint.x(), mousePoint.y()
            self.vLine.setPos(x)
            self.hLine.setPos(y)
            self.tooltip.setText(f"Time: {x:.3f}s\nValue: {y:.3f}")
            self.tooltip.setPos(x, y)

    def refresh_plot(self):
        # 1. Update Load Monitor Stats
        now = time.time()
        dt = now - self.last_load_check
        if dt >= 1.0:
            self.current_hz = int(self.pkts_since_last_update / dt)
            fill_pct = (min(self.tick, MAX_SAMPLES) / MAX_SAMPLES) * 100
            self.stats_label.setText(f"Serial Rate: {self.current_hz} Hz | Buffer: {fill_pct:.1f}%")
            self.pkts_since_last_update = 0
            self.last_load_check = now

        if self.tick == 0: return
        
        # 2. Sync graph window to the LATEST data received (slowing down if data is slow)
        if self.auto_scroll:
            self.plot.setXRange(self.last_t - VIEW_SEC, self.last_t, padding=0)

        view_range = self.plot.getViewBox().viewRange()[0]
        xmin, xmax = view_range

        if self.tick < MAX_SAMPLES:
            x_f, y_f = self.time_data[:self.ptr], self.y_data[:, :self.ptr]
        else:
            x_f = np.concatenate((self.time_data[self.ptr:], self.time_data[:self.ptr]))
            y_f = np.concatenate((self.y_data[:, self.ptr:], self.y_data[:, :self.ptr]), axis=1)

        mask = (x_f >= xmin - 0.5) & (x_f <= xmax + 0.5)
        x_v = x_f[mask]
        
        for i in range(CHANNELS):
            if self.checks[i].isChecked() and len(x_v) > 0:
                self.curves[i].setData(x_v, y_f[i][mask])
            else:
                self.curves[i].clear()

    def save_data(self):
        if self.auto_scroll:
            QtWidgets.QMessageBox.warning(self, "Hold on", "Pause the graph before exporting.")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export Data", "", "CSV Files (*.csv)")
        if path:
            if self.tick < MAX_SAMPLES:
                x, y = self.time_data[:self.ptr], self.y_data[:, :self.ptr]
            else:
                x = np.concatenate((self.time_data[self.ptr:], self.time_data[:self.ptr]))
                y = np.concatenate((self.y_data[:, self.ptr:], self.y_data[:, :self.ptr]), axis=1)
            data_to_save = np.vstack((x - x[0], y)).T
            header = "Time_s," + ",".join(self.labels)
            np.savetxt(path, data_to_save, delimiter=",", header=header, comments='')
            QtWidgets.QMessageBox.information(self, "Done", "Export Successful")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = TelemetryDash("/home/dipto/sim_graph") 
    win.show()
    sys.exit(app.exec())
