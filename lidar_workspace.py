"""COIN-D6 acquisition and local SLAM workspace; never commands the drivetrain."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import time
import numpy as np
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg
from coind6.acquisition import Acquisition, TCPSource, ReplaySource
from lidar_filters import filter_scan
from slam import LocalSLAM
from map_export import save_map


class LidarWorkspace(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.reader = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='slam')
        self.future = None
        self.generation = 0
        self.model = LocalSLAM()
        self.history = deque(maxlen=100)
        self.revision = None
        self.last_scan = None
        self.result = None
        self.finished = False
        self.finish_pending = False
        self.agv_geometry = {}
        self.config = Path(__file__).with_name('lidar_settings.json')
        layout = QtWidgets.QHBoxLayout(self)
        controls = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(controls)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(controls)
        scroll.setMaximumWidth(360)
        layout.addWidget(scroll)
        self.source = QtWidgets.QComboBox()
        self.source.addItems(['TCP · live', 'Capture .bin · historical'])
        form.addRow('Source', self.source)
        self.host = QtWidgets.QLineEdit('192.168.4.1')
        self.port = QtWidgets.QSpinBox(); self.port.setRange(1, 65535); self.port.setValue(9000)
        self.file = QtWidgets.QLineEdit()
        for label, widget in [('LiDAR IP', self.host), ('TCP', self.port), ('Capture', self.file)]:
            form.addRow(label, widget)
        self.button(form, 'Choose capture', self.browse)
        self.checksum = QtWidgets.QComboBox(); self.checksum.addItems(['auto', 'manual', 'sdk'])
        form.addRow('Checksum', self.checksum)
        self.record = QtWidgets.QCheckBox('Record bytes and scans in capturas_lidar')
        form.addRow(self.record)
        self.connect_button = self.button(form, 'Connect', self.connect_source)
        self.button(form, 'Disconnect', self.disconnect)
        self.button(form, 'Start LiDAR', lambda: self.command('start'))
        self.button(form, 'Stop LiDAR', lambda: self.command('stop'))
        self.x = self.number(form, 'Mount X (m, forward)', -2, 2, .115)
        self.y = self.number(form, 'Mount Y (m, left)', -2, 2, 0)
        self.angle = self.number(form, 'Mount yaw (deg CCW)', -180, 180, 0)
        self.minimum = self.number(form, 'Minimum range (m)', .05, 15, .10)
        self.maximum = self.number(form, 'Maximum range (m)', .1, 16, 12)
        self.body_margin = self.number(form, '23 x 23 cm footprint margin (m)', 0, .3, .02)
        self.sector_center = self.number(form, 'Excluded sector centre (deg)', -180, 180, 180)
        self.sector_width = self.number(form, 'Excluded sector width (deg, 0=off)', 0, 360, 0)
        mask_note = QtWidgets.QLabel('Footprint returns are excluded. Adjust the sector for obstructions: 0 deg forward, +90 left, 180 rear. Apply with New map.')
        mask_note.setWordWrap(True); form.addRow(mask_note)
        self.filter_status = QtWidgets.QLabel('Footprint filter active'); form.addRow(self.filter_status)
        self.odom = QtWidgets.QCheckBox('Use AGV odometry prediction')
        form.addRow(self.odom)
        self.odom_mode = QtWidgets.QComboBox()
        self.odom_mode.addItems(['Left encoder + IMU', 'Right encoder + IMU', 'AGV odometry (2 encoders)'])
        form.addRow('Prediction source', self.odom_mode)
        self.mapping = QtWidgets.QCheckBox('Build local map'); self.mapping.setChecked(True)
        form.addRow(self.mapping)
        self.button(form, 'New map / apply calibration', self.reset_map)
        self.button(form, 'Finish scan + save map/image', self.finish_scan)
        self.button(form, 'Save map + trajectory + PNG', self.export)
        self.status = QtWidgets.QLabel('LiDAR disconnected'); self.status.setWordWrap(True); form.addRow(self.status)
        self.quality = QtWidgets.QLabel(); self.quality.setWordWrap(True); form.addRow(self.quality)
        note = QtWidgets.QLabel('Experimental local SLAM: no loop closure or navigation.\nMount: 11.5 cm forward, centred. Confirm yaw and external LiDAR IP (192.168.4.1 is provisional).\nDedicated PC-to-LiDAR TCP connection; no LiDAR traffic through the AGV.\nX forward, Y left; metres. Replay never uses live odometry.')
        note.setWordWrap(True); form.addRow(note)
        self.plot = pg.PlotWidget(title='LiDAR / local SLAM · point map')
        self.plot.setAspectLocked(True); self.plot.showGrid(x=True, y=True)
        self.plot.setLabel('bottom', 'X', 'm'); self.plot.setLabel('left', 'Y', 'm')
        self.plot.setRange(xRange=(-5, 5), yRange=(-5, 5))
        layout.addWidget(self.plot, 1)
        self.map_item = self.plot.plot(pen=None, symbol='o', symbolSize=2, symbolBrush='#688aab')
        self.scan_item = self.plot.plot(pen=None, symbol='o', symbolSize=3, symbolBrush='#72eed6')
        self.path_item = self.plot.plot(pen=pg.mkPen('#eec766', width=2))
        self.robot_item = self.plot.plot(pen=pg.mkPen('#ff875e', width=3))
        self.load()
        self.reset_map(save=False)
        self.odom.toggled.connect(lambda: self.reset_map(save=False))
        self.odom_mode.currentIndexChanged.connect(lambda: self.reset_map(save=False))
        self.mapping.toggled.connect(lambda: self.reset_map(save=False))
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.poll); self.timer.start(80)

    @staticmethod
    def button(form, label, slot):
        button = QtWidgets.QPushButton(label); button.clicked.connect(slot); form.addRow(button); return button

    @staticmethod
    def number(form, label, low, high, value):
        field = QtWidgets.QDoubleSpinBox(); field.setDecimals(3); field.setRange(low, high); field.setValue(value)
        form.addRow(label, field); return field

    def load(self):
        try:
            data = json.loads(self.config.read_text(encoding='utf8'))
            for name in ('host',):
                if isinstance(data.get(name), str): getattr(self, name).setText(data[name])
            for name in ('port', 'x', 'y', 'angle', 'minimum', 'maximum', 'body_margin', 'sector_center', 'sector_width'):
                value = data.get(name)
                if isinstance(value, (int, float)) and math.isfinite(value):
                    getattr(self, name).setValue(int(value) if name == 'port' else value)
            if data.get('odom_mode') in (0,1,2): self.odom_mode.setCurrentIndex(data['odom_mode'])
            if isinstance(data.get('use_odom'),bool): self.odom.setChecked(data['use_odom'])
        except (OSError, ValueError, TypeError):
            pass

    def reset_map(self, checked=False, save=True):
        if self.minimum.value() >= self.maximum.value():
            self.quality.setText('Minimum range must be below maximum range'); return
        self.generation += 1
        self.model = LocalSLAM(); self.result = None
        self.map_geometry = dict(self.agv_geometry)
        self.finished = False; self.finish_pending = False
        self.mount = (self.x.value(), self.y.value(), self.angle.value(), self.minimum.value(), self.maximum.value())
        self.mask = (self.body_margin.value(), self.sector_center.value(), self.sector_width.value())
        self.use_odom = self.odom.isChecked() and self.source.currentIndex() == 0
        self.history.clear()
        for item in (self.map_item, self.scan_item, self.path_item, self.robot_item): item.setData([], [])
        self.quality.setText('Map reset; waiting for a valid scan')
        if save:
            data = {name: getattr(self, name).value() for name in ('port', 'x', 'y', 'angle', 'minimum', 'maximum', 'body_margin', 'sector_center', 'sector_width')}
            data.update(host=self.host.text(), odom_mode=self.odom_mode.currentIndex(), use_odom=self.odom.isChecked())
            try: self.config.write_text(json.dumps(data, indent=2), encoding='utf8')
            except OSError as exc: self.quality.setText(str(exc))

    def browse(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'LiDAR capture', '', 'BIN (*.bin)')
        if path: self.file.setText(path)

    def connect_source(self):
        if self.reader and self.reader.is_alive():
            self.status.setText('Disconnect and wait for shutdown before reconnecting'); return
        if self.minimum.value() >= self.maximum.value():
            self.quality.setText('Minimum range must be below maximum range'); return
        mode = self.source.currentIndex()
        if mode == 0 and not self.host.text().strip(): return
        if mode == 1 and not Path(self.file.text()).is_file():
            self.status.setText('Select a valid .bin file'); return
        source = TCPSource(self.host.text(), self.port.value()) if mode == 0 else ReplaySource(self.file.text())
        self.reset_map(); self.last_scan = None
        self.live = mode == 0
        output = Path(__file__).with_name('capturas_lidar') if self.record.isChecked() and self.live else None
        self.reader = Acquisition(source, output, self.checksum.currentText()); self.reader.start()
        self.source.setEnabled(False)

    def command(self, action):
        if self.reader and self.reader.is_alive(): self.reader.command(action)

    def disconnect(self):
        if self.reader: self.reader.shutdown()

    def receive_state(self, state):
        self.agv_geometry = dict(state.get('geometry', self.agv_geometry))
        revision = state.get('pose_revision')
        mode = self.odom_mode.currentIndex()
        if mode != 2: revision = (revision, state.get('imu_segment'))
        if self.use_odom and not self.finished and revision != self.revision:
            self.reset_map(save=False)
        self.revision = revision
        if state.get('fresh') and not state.get('capture_active'):
            if mode == 2:
                pose = np.array([state['x']/1000, state['y']/1000, math.radians(state['yaw'])])
            else:
                raw = state.get('single_left' if mode == 0 else 'single_right')
                if not state.get('imu_valid') or raw is None:
                    self.history.clear()
                    return
                pose = np.array([raw[0]/1000, raw[1]/1000, raw[2]])
            self.history.append((state.get('host_monotonic', time.monotonic()), pose))
        else:
            self.history.clear()

    def poll(self):
        if self.future and self.future.done():
            future, generation = self.future, self.job_generation; self.future = None
            try: result = future.result()
            except Exception as exc:
                self.quality.setText(f'SLAM error: {exc}'); result = None
            if generation == self.generation and result is not None:
                self.result = result; self.quality.setText(result['note'])
                for key, item in [('map', self.map_item), ('path', self.path_item), ('points', self.scan_item)]:
                    data = result[key]; item.setData(data[:, 0], data[:, 1])
                x, y, a = result['pose']
                self.robot_item.setData([x, x+.25*math.cos(a)], [y, y+.25*math.sin(a)])
        if self.finish_pending and self.future is None:
            self.finish_pending = False
            self.export()
        if not self.reader: return
        scan, stats = self.reader.snapshot()
        if not self.reader.is_alive(): self.source.setEnabled(True)
        age = time.monotonic() - stats['last_rx'] if stats['last_rx'] else float('inf')
        label = 'LIVE' if self.live and stats['connected'] and age < .5 else 'NO RECENT DATA' if self.live else 'HISTORICAL CAPTURE'
        self.status.setText(f"{label} · {stats['state']}\nPackets {stats['packets']} · CRC errors {stats['bad_crc']} · scans {stats['scans']}")
        while True:
            try: self.last_message = self.reader.messages.get_nowait()
            except queue.Empty: break
        if hasattr(self, 'last_message'): self.status.setText(self.status.text()+'\n'+self.last_message)
        if self.finished or scan is None or scan.sequence == self.last_scan or self.future: return
        self.last_scan = scan.sequence
        if scan.damaged or (self.live and time.monotonic()-scan.end_monotonic > .5):
            self.quality.setText('Damaged or stale scan: skipped'); return
        points, excluded = filter_scan(scan.points, self.mount, *self.mask)
        self.filter_status.setText(f'Footprint/sector: {excluded} excluded · {len(points)} usable')
        if not self.mapping.isChecked():
            self.scan_item.setData(points[:, 0], points[:, 1]); self.map_item.setData([], []); self.path_item.setData([], [])
            self.robot_item.setData([], []); self.quality.setText('Scan only · robot coordinates'); return
        odom = None
        if self.use_odom:
            sample = min(self.history, key=lambda v: abs(v[0]-scan.end_monotonic), default=None)
            if sample is None or abs(sample[0]-scan.end_monotonic) > .2:
                self.quality.setText('Waiting for timely odometry; single-encoder mode requires a valid IMU'); return
            odom = sample[1]
        self.job_generation = self.generation
        if not len(self.model.map):self.map_geometry = dict(self.agv_geometry)
        self.future = self.pool.submit(self.model.update, points, scan.end_monotonic, odom)

    def finish_scan(self):
        self.finished = True
        self.command('stop')
        self.finish_pending = True
        self.quality.setText('Finishing pending scan; map will remain available after saving')

    def export(self):
        if self.result is None or not len(self.result['map']):
            self.quality.setText('No map to export'); return
        default_name = 'slam_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.npz'
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save point map + PNG', default_name, 'NPZ (*.npz)')
        if path:
            try:
                archive,picture=save_map(path,self.result,dict(created_utc=datetime.now(timezone.utc).isoformat(),
                    mount_xy_yaw_deg=self.mount[:3],range_limits_m=self.mount[3:],source='live' if self.live else 'replay',
                    mask_margin_m_center_deg_width_deg=self.mask,agv_geometry=self.map_geometry,
                    odometry_mode=self.odom_mode.currentText() if self.use_odom else 'none',
                    algorithm='local_scan_to_map_icp_no_loop_closure'))
                self.quality.setText(f'Saved map: {archive}\nImage: {picture}')
            except (OSError,ValueError) as exc: self.quality.setText('Save failed: '+str(exc))

    def shutdown(self):
        self.timer.stop(); self.disconnect()
        if self.reader: self.reader.join(2.5)
        self.pool.shutdown(wait=True, cancel_futures=True)
