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
from lidar_filters import partition_scan
from slam import LocalSLAM, transform
from map_export import save_map
from lidar_guidance import sector_ranges, checkpoint_delta, motor_spin, spin_during_scan
from lidar_calibration import LidarCalibrationWizard, raw_calibration_points
from lidar_exploration import ExplorationGrid


class LidarWorkspace(QtWidgets.QWidget):
    straight_pressed = QtCore.Signal(int)
    stop_requested = QtCore.Signal()
    turn_requested = QtCore.Signal(float)
    trial_motion_requested = QtCore.Signal(int)

    def __init__(self):
        super().__init__()
        self.reader = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='slam')
        self.future = None
        self.generation = 0
        self.model = LocalSLAM()
        self.exploration=ExplorationGrid()
        self.history = deque(maxlen=100)
        self.revision = None
        self.trial_file = None
        self.last_scan = None
        self.result = None
        self.result_time = None
        self.map_status = 'esperando escaneo'
        self.checkpoint = None
        self.lidar_fresh = False
        self.robot_stationary = False
        self.odom_ready = False
        self.front_wizard = None
        self.agv_telemetry_fresh = False
        self.inspection_points = np.empty((0,2))
        self.inspection_time = None
        self.calibration_points = np.empty((0,2));self.calibration_time=None
        self.finished = False
        self.finish_pending = False
        self.agv_geometry = {}
        self.config = Path(__file__).with_name('lidar_settings.json')
        layout = QtWidgets.QHBoxLayout(self)
        controls = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(controls)
        form.setRowWrapPolicy(QtWidgets.QFormLayout.RowWrapPolicy.WrapLongRows)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(controls)
        scroll.setMinimumWidth(300)
        scroll.setMaximumWidth(360)
        layout.addWidget(scroll)
        self.host = QtWidgets.QLineEdit('192.168.4.1')
        form.addRow('IP LiDAR',self.host)
        self.connect_button = self.button(form, 'Conectar', self.connect_source)
        self.button(form, 'Iniciar escaneo 360°', lambda: self.command('start'))
        self.button(form, 'Detener LiDAR', lambda: self.command('stop'))
        self.button(form, 'Nuevo mapa / aplicar ajustes', self.reset_map)
        self.tracking_status=QtWidgets.QLabel('LiDAR: desconectado · Mapa: esperando datos');self.tracking_status.setWordWrap(True);form.addRow(self.tracking_status)
        self.trial_mode = QtWidgets.QComboBox()
        self.trial_mode.addItems(['Quieto (30 s)', 'Automático: giro +90°', 'Automático: avanzar 30 cm y regresar'])
        form.addRow('Prueba corta', self.trial_mode)
        self.trial_button = self.button(form, 'Grabar prueba', self.toggle_trial)
        self.trial_mode.currentIndexChanged.connect(lambda i:self.trial_button.setText('Iniciar movimiento y grabar' if i else 'Grabar prueba'))
        self.trial_motion_active = False
        self.trial_status = QtWidgets.QLabel()
        self.trial_status.setWordWrap(True)
        form.addRow(self.trial_status)
        self.inspect_toggle=QtWidgets.QPushButton('Ver escaneo para medir ángulos');self.inspect_toggle.setCheckable(True);form.addRow(self.inspect_toggle)
        self.front_calibrate_button=self.button(form,'Asistente de calibración',self.begin_front_calibration)
        self.front_calibrate_button.setToolTip('Selecciona un objeto frente al LiDAR en una captura y revisa la orientación antes de aplicarla.')
        self.inspection_help=QtWidgets.QLabel('Verde: usado · naranja: excluido. Sin retorno no significa espacio libre.');self.inspection_help.setWordWrap(True);form.addRow(self.inspection_help)
        self.angle_readout=QtWidgets.QLabel('Ángulo: —');self.angle_readout.setWordWrap(True);form.addRow(self.angle_readout)
        self.filter_status = QtWidgets.QLabel('Filtro pendiente');self.filter_status.setWordWrap(True);form.addRow(self.filter_status)
        self.button(form, 'Guardar mapa e imagen', self.export)
        self.button(form, 'Guardar posición actual', self.save_checkpoint)
        self.proximity = QtWidgets.QLabel('Sectores: esperando escaneo');self.proximity.setWordWrap(True);form.addRow(self.proximity)
        self.checkpoint_status=QtWidgets.QLabel('Punto de referencia: sin guardar');self.checkpoint_status.setWordWrap(True);form.addRow(self.checkpoint_status)
        advanced_toggle=QtWidgets.QPushButton('Configuración avanzada ▸');advanced_toggle.setCheckable(True);form.addRow(advanced_toggle)
        advanced=QtWidgets.QWidget();advanced.setVisible(False);form.addRow(advanced)
        advanced_toggle.toggled.connect(advanced.setVisible);extra=QtWidgets.QFormLayout(advanced)
        self.source = QtWidgets.QComboBox()
        self.source.addItems(['TCP · live', 'Capture .bin · historical'])
        extra.addRow('Fuente', self.source)
        self.port = QtWidgets.QSpinBox(); self.port.setRange(1, 65535); self.port.setValue(9000)
        self.file = QtWidgets.QLineEdit()
        for label, widget in [('Puerto TCP', self.port), ('Archivo', self.file)]:
            extra.addRow(label, widget)
        self.button(extra, 'Elegir captura', self.browse)
        self.checksum = QtWidgets.QComboBox(); self.checksum.addItems(['auto', 'manual', 'sdk'])
        extra.addRow('Checksum', self.checksum)
        self.record = QtWidgets.QCheckBox('Record bytes and scans in capturas_lidar')
        extra.addRow(self.record)
        self.button(extra, 'Desconectar', self.disconnect)
        self.x = self.number(extra, 'Montaje X (m, adelante)', -2, 2, .11)
        self.y = self.number(extra, 'Montaje Y (m, izquierda)', -2, 2, 0)
        self.angle = self.number(extra, 'Giro de montaje (°)', -180, 180, 0)
        self.minimum = self.number(extra, 'Distancia mínima (m)', .05, 15, .10)
        self.maximum = self.number(extra, 'Distancia máxima (m)', .1, 16, 12)
        self.body_margin = self.number(extra, 'Margen de chasis (m)', 0, .3, .02)
        self.sector_center = self.number(extra, 'Centro zona tapada (°)', -180, 180, 180)
        self.sector_width = self.number(extra, 'Ancho zona tapada (°)', 0, 360, 0)
        mask_note = QtWidgets.QLabel('0° delante; 180° atrás. Una máscara de 0° no excluye sectores. Cambiar ajustes requiere Nuevo mapa.')
        mask_note.setWordWrap(True); extra.addRow(mask_note)
        self.odom = QtWidgets.QCheckBox('Usar encoder e IMU para el mapa')
        form.addRow(self.odom)
        self.odom_mode = QtWidgets.QComboBox()
        self.odom_mode.addItems(['Left encoder + IMU', 'Right encoder + IMU', 'AGV odometry (2 encoders)'])
        extra.addRow('Fuente de odometría', self.odom_mode)
        self.mapping = QtWidgets.QCheckBox('Construir mapa local'); self.mapping.setChecked(True)
        extra.addRow(self.mapping)
        self.button(extra, 'Finalizar escaneo y guardar', self.finish_scan)
        self.status = QtWidgets.QLabel('LiDAR desconectado'); self.status.setWordWrap(True); form.addRow(self.status)
        self.quality = QtWidgets.QLabel(); self.quality.setWordWrap(True); form.addRow(self.quality)
        note = QtWidgets.QLabel('Sectores desde el centro del AGV; no certifican espacio libre. En la pista, paredes de 15 cm: confirma que el plano del LiDAR las vea. No detecta obstáculos fuera de ese plano ni mueve el robot.')
        note.setWordWrap(True); form.addRow(note)
        self.plot = pg.PlotWidget(title='Mapa: azul tenue = rayos observados · puntos = retornos · negro = desconocido')
        self.plot.setAspectLocked(True); self.plot.showGrid(x=True, y=True)
        self.plot.setLabel('bottom', 'X', 'm'); self.plot.setLabel('left', 'Y', 'm')
        self.plot.setRange(xRange=(-2.5, 2.5), yRange=(-2.5, 2.5))
        plots=QtWidgets.QVBoxLayout();layout.addLayout(plots,1);plots.addWidget(self.plot,1)
        motion = QtWidgets.QGroupBox('Movimiento de prueba del AGV')
        plots.insertWidget(0, motion)
        buttons = QtWidgets.QHBoxLayout(motion)
        for label, direction in [('Avanzar', 1), ('Retroceder', -1)]:
            button = QtWidgets.QPushButton(label + ' · mantener')
            button.pressed.connect(lambda direction=direction: self.straight_pressed.emit(direction))
            button.released.connect(self.stop_requested.emit)
            buttons.addWidget(button)
        self.trial_angle = QtWidgets.QDoubleSpinBox()
        self.trial_angle.setRange(-360, 360)
        self.trial_angle.setSuffix('°')
        self.trial_angle.setValue(90)
        self.trial_angle.setToolTip('Positivo gira a la izquierda. Negativo gira a la derecha.')
        buttons.addWidget(self.trial_angle)
        turn_button = QtWidgets.QPushButton('Girar · IMU precisa')
        turn_button.clicked.connect(lambda: self.turn_requested.emit(self.trial_angle.value()))
        self.trial_angle.valueChanged.connect(lambda angle: turn_button.setEnabled(angle != 0))
        buttons.addWidget(turn_button)
        stop_button = QtWidgets.QPushButton('STOP AGV')
        stop_button.clicked.connect(self.stop_requested.emit)
        buttons.addWidget(stop_button)
        self.observed_item=pg.ImageItem(axisOrder='row-major');self.observed_item.setZValue(-10);self.plot.addItem(self.observed_item)
        self.map_item = self.plot.plot(pen=None, symbol='o', symbolSize=2, symbolBrush='#688aab')
        self.scan_item = self.plot.plot(pen=None, symbol='o', symbolSize=4, symbolBrush='#72eed6')
        self.path_item = self.plot.plot(pen=pg.mkPen('#eec766', width=2))
        self.robot_body_item=self.plot.plot(pen=pg.mkPen('#ff875e',width=2))
        self.robot_sensor_item=self.plot.plot(pen=None,symbol='+',symbolSize=12,symbolPen=pg.mkPen('#ffd76a',width=2))
        self.robot_direction=self.plot.plot(pen=pg.mkPen('#ff875e',width=3))
        self.plot.addLine(x=0, pen=pg.mkPen('#498574'))
        self.plot.addLine(y=0, pen=pg.mkPen('#aa6458'))
        origin = pg.TextItem('Origen (0, 0)', color='#dedede', anchor=(0,1))
        self.plot.addItem(origin)
        self.plot.setToolTip('La flecha naranja marca 180° respecto al eje X del robot.')
        self.inspection_plot=pg.PlotWidget(title='Vista relativa al AGV · verde: usado · naranja: excluido')
        self.inspection_plot.setAspectLocked(True);self.inspection_plot.showGrid(x=True,y=True)
        self.inspection_plot.setLabel('bottom','Adelante (+X)','m');self.inspection_plot.setLabel('left','Izquierda (+Y)','m')
        self.inspection_plot.setRange(xRange=(-1,1.5),yRange=(-1.25,1.25));plots.addWidget(self.inspection_plot,1);self.inspection_plot.hide()
        self.inspection_usable=self.inspection_plot.plot(pen=None,symbol='o',symbolSize=4,symbolBrush='#72eed6')
        self.inspection_excluded=self.inspection_plot.plot(pen=None,symbol='o',symbolSize=4,symbolBrush='#ff9c5b')
        self.inspection_body=self.inspection_plot.plot(pen=pg.mkPen('#c3cada',width=2))
        self.inspection_sensor=self.inspection_plot.plot(pen=None,symbol='+',symbolSize=14,symbolPen=pg.mkPen('#ffd76a',width=2))
        self.inspection_mouse=pg.SignalProxy(self.inspection_plot.scene().sigMouseMoved,rateLimit=20,slot=self.inspection_cursor_moved)
        self.inspect_toggle.toggled.connect(self.show_inspection)
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

    def show_inspection(self,enabled):
        self.inspection_plot.setVisible(enabled);self.plot.setVisible(not enabled)
        self.inspect_toggle.setText('Volver al mapa' if enabled else 'Ver escaneo para medir ángulos')

    def draw_robot(self,pose,detected=None):
        # Nominal 23 cm body, not a collision envelope; +X is its front.
        shape=np.array([[-.115,-.115],[.115,-.115],[.115,.115],[-.115,.115],[-.115,-.115]])
        body=transform(shape,pose)
        sensor=transform(np.array([[self.mount[0],self.mount[1]]]),pose)
        self.robot_body_item.setData(body[:,0],body[:,1])
        self.robot_sensor_item.setData(sensor[:,0],sensor[:,1])
        direction=np.array([math.cos(pose[2]),math.sin(pose[2])])
        arrow=self.arrow_towards(np.asarray(pose[:2]),-direction)
        self.robot_direction.setData(arrow[:,0],arrow[:,1])
    @staticmethod
    def arrow_towards(origin,direction):
        direction=np.asarray(direction);normal=np.array([-direction[1],direction[0]])
        tip=origin+.20*direction;tail=origin+.04*direction
        return np.vstack((tail,tip,tip-.055*direction+.035*normal,tip,tip-.055*direction-.035*normal))

    def begin_front_calibration(self):
        if self.front_wizard is not None:
            self.front_wizard.raise_();self.front_wizard.activateWindow();return
        if not self.lidar_fresh or self.calibration_time is None or time.monotonic()-self.calibration_time>2 or not len(self.calibration_points):
            QtWidgets.QMessageBox.information(self,'Calibración LiDAR','Conecta el LiDAR, inicia el escaneo y espera a que aparezcan puntos.');return
        if self.agv_telemetry_fresh and not self.robot_stationary:
            QtWidgets.QMessageBox.information(self,'Calibración LiDAR','Detén el AGV antes de capturar la referencia.');return
        raw_mount=(self.mount[0],self.mount[1],0.,0.,self.mount[4])
        wizard=LidarCalibrationWizard(self.calibration_points,raw_mount,self,
                                     sector_center=self.mask[1],sector_width=self.mask[2],body_margin=self.mask[0],current_angle=self.mount[2])
        self.front_wizard=wizard;generation=self.generation
        wizard.refresh_button.clicked.connect(lambda:self.refresh_calibration(wizard,generation))
        wizard.accepted.connect(lambda:self.apply_front_calibration(wizard,generation))
        wizard.finished.connect(lambda _result:setattr(self,'front_wizard',None))
        wizard.open()

    def refresh_calibration(self,wizard,generation):
        if generation!=self.generation:
            QtWidgets.QMessageBox.warning(wizard,'Calibración LiDAR','Cambió el mapa. Cierra el asistente y vuelve a abrirlo.');return
        if self.agv_telemetry_fresh and not self.robot_stationary:
            QtWidgets.QMessageBox.warning(wizard,'Calibración LiDAR','Detén el AGV antes de actualizar la captura.');return
        if not self.lidar_fresh or self.calibration_time is None or time.monotonic()-self.calibration_time>2 or not len(self.calibration_points):
            QtWidgets.QMessageBox.information(wizard,'Calibración LiDAR','No hay un escaneo reciente. Comprueba la conexión y el escaneo.');return
        wizard.replace_points(self.calibration_points)

    def apply_front_calibration(self,wizard,generation):
        if generation!=self.generation or (self.agv_telemetry_fresh and not self.robot_stationary):
            QtWidgets.QMessageBox.warning(self,'Calibración LiDAR','Cambió el mapa o el AGV se movió; repite la captura.');return
        self.angle.setValue(wizard.candidate)
        self.sector_center.setValue(wizard.center.value());self.sector_width.setValue(wizard.width.value())
        if self.reset_map():
            self.quality.setText(f'Giro aplicado: {wizard.candidate:+.1f}°. Verifica frente y lados. Mapa nuevo.')
        else:
            QtWidgets.QMessageBox.warning(self,'Calibración LiDAR','No se pudo guardar la configuración. Revisa el mensaje de estado y vuelve a intentarlo.')

    def inspection_cursor_moved(self,event):
        position=event[0]
        if not self.inspection_plot.sceneBoundingRect().contains(position):return
        point=self.inspection_plot.getViewBox().mapSceneToView(position)
        dx=point.x()-self.mount[0];dy=point.y()-self.mount[1]
        angle=math.degrees(math.atan2(dy,dx));distance=math.hypot(dx,dy)
        self.angle_readout.setText(f'Cursor desde LiDAR: {angle:+.1f}° · {distance:.2f} m')

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
        self.reference_lost = False
        self.trial_event('nuevo_mapa', generation=self.generation)
        self.model = LocalSLAM(); self.result = None
        self.exploration=ExplorationGrid();self.observed_item.clear()
        self.result_time=None;self.checkpoint=None
        self.map_status='esperando escaneo'
        self.checkpoint_status.setText('Punto de referencia: sin guardar')
        self.map_geometry = dict(self.agv_geometry)
        self.finished = False; self.finish_pending = False
        self.mount = (self.x.value(), self.y.value(), self.angle.value(), self.minimum.value(), self.maximum.value())
        self.mask = (self.body_margin.value(), self.sector_center.value(), self.sector_width.value())
        self.filter_status.setText(f'Máscara aplicada: centro {self.mask[1]:g}° · ancho {self.mask[2]:g}°')
        self.use_odom = self.odom.isChecked() and self.source.currentIndex() == 0
        self.history.clear()
        self.inspection_points=np.empty((0,2));self.inspection_time=None
        self.calibration_points=np.empty((0,2));self.calibration_time=None
        for item in (self.map_item, self.scan_item, self.path_item, self.robot_body_item,self.robot_sensor_item,self.robot_direction): item.setData([], [])
        for item in (self.inspection_usable,self.inspection_excluded):item.setData([],[])
        self.inspection_body.setData([-.115,.115,.115,-.115,-.115],[-.115,-.115,.115,.115,-.115])
        self.inspection_sensor.setData([self.mount[0]],[self.mount[1]])
        self.quality.setText('Mapa nuevo; esperando escaneo válido' if self.mount_plausible() else 'Revisa el montaje: X o Y supera 0.5 m; corrígelo en configuración avanzada y pulsa Nuevo mapa')
        if save:
            data = {name: getattr(self, name).value() for name in ('port', 'x', 'y', 'angle', 'minimum', 'maximum', 'body_margin', 'sector_center', 'sector_width')}
            data.update(host=self.host.text(), odom_mode=self.odom_mode.currentIndex(), use_odom=self.odom.isChecked())
            try: self.config.write_text(json.dumps(data, indent=2), encoding='utf8')
            except OSError as exc: self.quality.setText(str(exc));return False
        return True

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
        self.result_time=None;self.lidar_fresh=False;self.proximity.setText('Sectores: LiDAR desconectado')
        self.inspection_usable.setData([],[]);self.inspection_excluded.setData([],[])

    def mount_plausible(self):
        return abs(self.mount[0])<=.5 and abs(self.mount[1])<=.5

    def current_local_pose(self):
        if not getattr(self,'live',False) or self.finished or not self.mount_plausible() or not self.lidar_fresh:return None
        if self.use_odom and not self.odom_ready:return None
        if not self.reader or not self.reader.is_alive() or not self.result or not self.result['accepted'] or self.result_time is None:return None
        if time.monotonic()-self.result_time>1.0:return None
        return self.result['pose']

    def save_checkpoint(self):
        pose=self.current_local_pose()
        if pose is None or not self.robot_stationary or len(self.result.get('path',[]))<2:
            self.checkpoint_status.setText('No se guardó: detén el AGV y espera al menos dos escaneos localizados recientes.');return
        self.checkpoint=np.asarray(pose,dtype=float).copy()
        self.checkpoint_status.setText('Punto guardado en el mapa local; sin movimiento automático.')

    def update_checkpoint(self):
        if self.checkpoint is None:return
        pose=self.current_local_pose()
        if pose is None:
            self.checkpoint_status.setText('Punto guardado · posición actual no disponible');return
        x,y,yaw=checkpoint_delta(pose,self.checkpoint)
        self.checkpoint_status.setText(f'Hacia el punto guardado: adelante {x*100:+.1f} cm · izquierda {y*100:+.1f} cm · giro {yaw:+.1f}°. Estimado; no mueve el AGV.')

    def toggle_trial(self):
        if self.trial_file:
            self.finish_trial()
            return
        if not self.lidar_fresh or not self.odom_ready or not self.use_odom or not self.mapping.isChecked():
            self.trial_status.setText('Conecta AGV y LiDAR, inicia el escaneo y activa mapa y odometría.')
            return
        if self.trial_mode.currentIndex() and (self.reference_lost or self.calibration_time is None or
                                               time.monotonic()-self.calibration_time > 1):
            self.trial_status.setText('Espera un escaneo reciente y una referencia de odometría válida.')
            return
        folder = Path(__file__).with_name('capturas_lidar')
        try:
            folder.mkdir(exist_ok=True)
            self.trial_path = folder / datetime.now().strftime('prueba_%Y%m%d_%H%M%S_%f.jsonl')
            self.trial_file = self.trial_path.open('x', encoding='utf8')
        except OSError as exc:
            self.trial_status.setText(str(exc))
            return
        self.trial_start = time.monotonic()
        self.trial_scans = self.trial_accepted = self.trial_rejected = 0
        self.trial_mode.setEnabled(False)
        self.trial_button.setText('Terminar y guardar')
        self.trial_status.setText('Grabando 30 s. STOP AGV cancela el movimiento.')
        self.trial_event('inicio', prueba=self.trial_mode.currentText(), mount=self.mount, mask=self.mask,
                         odometry=self.odom_mode.currentText(), generation=self.generation)
        mode = self.trial_mode.currentIndex()
        if self.trial_file and mode:
            self.trial_motion_active = True
            self.trial_motion_requested.emit(mode)

    def trial_event(self, event, **data):
        if self.trial_file:
            try:
                row = dict(event=event, host_s=time.monotonic(), **data)
                self.trial_file.write(json.dumps(row, ensure_ascii=False)+'\n')
            except (OSError, ValueError) as exc:
                self.finish_trial(error=str(exc))

    def finish_trial(self, error=None):
        if self.trial_file is None:
            return
        if self.trial_motion_active:
            self.trial_motion_active = False
            self.stop_requested.emit()
        try:
            self.trial_file.close()
        except OSError as exc:
            error = str(exc)
        self.trial_file = None
        self.trial_mode.setEnabled(True)
        self.trial_button.setText('Iniciar movimiento y grabar' if self.trial_mode.currentIndex() else 'Grabar prueba')
        self.trial_status.setText(error or f'{self.trial_scans} escaneos · {self.trial_accepted} localizados · '
                                  f'{self.trial_rejected} rechazados. Archivo: {self.trial_path.name}')

    def receive_state(self, state):
        if self.trial_file:
            telemetry = state.get('telemetry', {})
            self.trial_event('agv', ticks=[telemetry.get('tl'), telemetry.get('tr')],
                             **{key: state.get(key) for key in ('host_monotonic','age','fresh','imu_valid','imu_yaw',
                                'imu_segment','pose_revision','single_left','single_right','x','y','yaw',
                                'routine_step','result','sent_pwm')})

        telemetry=state.get('telemetry',{})
        self.agv_telemetry_fresh=bool(state.get('fresh'))
        self.odom_ready=bool(state.get('fresh') and not state.get('capture_active') and (self.odom_mode.currentIndex()==2 or state.get('imu_valid')))
        self.robot_stationary=bool(state.get('fresh') and tuple(state.get('sent_pwm',(1,1)))==(0,0)
            and telemetry.get('pl')==0 and telemetry.get('pr')==0
            and abs(state.get('vl',float('inf')))<8 and abs(state.get('vr',float('inf')))<8)
        self.agv_geometry = dict(state.get('geometry', self.agv_geometry))
        revision = state.get('pose_revision')
        mode = self.odom_mode.currentIndex()
        if mode != 2: revision = (revision, state.get('imu_segment'))
        if self.use_odom and not self.finished and revision != self.revision:
            if self.revision is None or (not len(self.model.map) and self.future is None):
                self.reset_map(save=False)
            elif not self.reference_lost:
                self.reference_lost = True
                self.generation += 1
                self.result_time = None
                self.history.clear()
                self.map_status = 'Referencia perdida. Guarda el mapa y pulsa Nuevo mapa para reiniciar.'
                self.scan_item.setData([], [])
                self.trial_event('referencia_perdida', previous=self.revision, current=revision)
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
            spin=bool(mode!=2 and state.get('imu_valid') and motor_spin(telemetry,self.agv_geometry))
            self.history.append((state.get('host_monotonic', time.monotonic()), pose,spin))
        else:
            self.history.clear()

    def poll(self):
        if self.trial_file and time.monotonic()-self.trial_start >= 30:
            self.finish_trial()
        if self.future and self.future.done():
            future, generation = self.future, self.job_generation; self.future = None
            try: result = future.result()
            except Exception as exc:
                self.map_status=f'error de SLAM: {exc}';self.quality.setText(self.map_status); result = None
            if generation == self.generation and result is not None:
                if self.trial_file:
                    self.trial_accepted += int(result['accepted'])
                    self.trial_rejected += int(not result['accepted'])
                    self.trial_event('slam', scan_end_s=getattr(self,'job_scan_time',None), accepted=result['accepted'],
                                     reason=result['note'], pose=result['pose'].tolist(), generation=generation)
                self.result = result; self.quality.setText(result['note'])
                self.map_status=('localizado: ' if result['accepted'] else 'sin localización: ')+result['note']
                self.result_time=getattr(self,'job_scan_time',None) if result['accepted'] else None
                for key, item in [('map', self.map_item), ('path', self.path_item)]:
                    data = result[key]; item.setData(data[:, 0], data[:, 1])
                if result['accepted']:
                    visible=(transform(self.job_scan_points,result['pose']) if hasattr(self,'job_scan_points') else result['points'])
                    self.scan_item.setData(visible[:,0],visible[:,1])
                    origin=transform(np.array([self.mount[:2]]),result['pose'])[0]
                    self.exploration.update(origin,visible)
                    background,rect=self.exploration.image()
                    if background is not None:
                        self.observed_item.setImage(background,autoLevels=False)
                        self.observed_item.setRect(QtCore.QRectF(*rect))
                    result['observed_cells']=np.asarray(list(self.exploration.observed),dtype=np.int32).reshape(-1,2)
                    result['observed_resolution_m']=self.exploration.resolution
                else:self.scan_item.setData([],[])
                self.draw_robot(result['pose'],visible if result['accepted'] else None)
        if self.finish_pending and self.future is None:
            self.finish_pending = False
            self.export()
        self.update_checkpoint()
        if not self.reader: return
        scan, stats = self.reader.snapshot()
        if not self.reader.is_alive(): self.source.setEnabled(True)
        age = time.monotonic() - stats['last_rx'] if stats['last_rx'] else float('inf')
        self.lidar_fresh=bool(self.live and stats['connected'] and age<.5)
        scan_age=time.monotonic()-self.calibration_time if self.calibration_time is not None else float('inf')
        if self.trial_motion_active and (not self.lidar_fresh or scan_age>1 or not self.odom_ready or self.reference_lost):
            self.finish_trial(error='Movimiento detenido: se perdió el LiDAR, la odometría o la referencia.')
        reception=('sin conexión' if not stats['connected'] else 'recibiendo escaneos' if scan_age<2 else 'recibe datos, sin escaneo reciente') if self.live else 'captura histórica'
        self.tracking_status.setText(f'LiDAR: {reception}\nMapa: {self.map_status}')
        if self.live and scan_age>=2:
            self.scan_item.setData([],[]);self.result_time=None
        label = 'LIVE' if self.live and stats['connected'] and age < .5 else 'NO RECENT DATA' if self.live else 'HISTORICAL CAPTURE'
        self.status.setText(f"{label} · {stats['state']}\nPackets {stats['packets']} · CRC errors {stats['bad_crc']} · scans {stats['scans']}")
        if self.live and (not stats['connected'] or age>=.5):
            if stats['connected'] and age>=.5:self.tracking_status.setText(f'LiDAR: sin datos recientes\nMapa: {self.map_status}')
            self.scan_item.setData([],[]);self.result_time=None
            self.proximity.setText('Sectores: sin escaneo reciente')
            self.inspection_usable.setData([],[]);self.inspection_excluded.setData([],[])
            if self.front_wizard is not None and self.front_wizard.live:
                self.front_wizard.status.setText('Sin datos recientes del LiDAR; comprueba la conexión y que el escaneo esté iniciado.')
        while True:
            try: self.last_message = self.reader.messages.get_nowait()
            except queue.Empty: break
        if hasattr(self, 'last_message'): self.status.setText(self.status.text()+'\n'+self.last_message)
        if self.finished or scan is None or scan.sequence == self.last_scan: return
        self.last_scan = scan.sequence
        if self.trial_file:
            self.trial_scans += 1
            self.trial_event('lidar', **scan.record(), packets=stats['packets'], crc=stats['bad_crc'])
        if scan.damaged or (self.live and time.monotonic()-scan.end_monotonic > .5):
            self.map_status='escaneo dañado o antiguo; esperando otro'
            self.result_time=None;self.proximity.setText('Sectores: escaneo incompleto o antiguo');self.quality.setText('Escaneo incompleto o antiguo: descartado'); return
        # Calibration must not depend on the old yaw, mask, chassis or range limits.
        self.calibration_points=raw_calibration_points(scan.points,self.mount[:2]);self.calibration_time=scan.end_monotonic
        if self.front_wizard is not None:self.front_wizard.update_live_points(self.calibration_points)
        if not self.mount_plausible():
            self.result_time=None;self.proximity.setText('Sectores: montaje sin confirmar');self.quality.setText('Corrige X/Y del montaje en configuración avanzada y pulsa Nuevo mapa');return
        points, excluded_points = partition_scan(scan.points, self.mount, *self.mask)
        self.inspection_points=np.vstack((points,excluded_points));self.inspection_time=scan.end_monotonic
        self.inspection_usable.setData(points[:,0],points[:,1])
        self.inspection_excluded.setData(excluded_points[:,0],excluded_points[:,1])
        ranges=sector_ranges(points)
        self.proximity.setText('Mediana de retornos por sector (desde centro AGV):\n'+' · '.join(f'{name}: {distance*100:.0f} cm' if distance is not None else f'{name}: sin datos' for name,distance in ranges.items()))
        self.filter_status.setText(f'Retornos válidos: {len(self.calibration_points)} · para mapa: {len(points)} · excluidos: {len(excluded_points)} · sector {self.mask[1]:g}° ± {self.mask[2]/2:g}°')
        if not self.mapping.isChecked():
            self.scan_item.setData(points[:, 0], points[:, 1]); self.map_item.setData([], []); self.path_item.setData([], [])
            self.draw_robot(np.zeros(3),points); self.quality.setText('Escaneo en coordenadas del AGV; sin mapa'); return
        # Keep the calibration preview fresh even while scan matching is busy.
        if self.reference_lost:
            return
        if self.future:return
        odom = None;rotation_only=False
        if self.use_odom:
            sample = min(self.history, key=lambda v: abs(v[0]-scan.end_monotonic), default=None)
            if sample is None or abs(sample[0]-scan.end_monotonic) > .2:
                self.map_status='esperando encoder e IMU recientes'
                self.scan_item.setData([],[])
                self.result_time=None;self.quality.setText('Esperando odometría reciente e IMU válida'); return
            odom = sample[1]
            rotation_only=spin_during_scan(self.history,scan.start_monotonic,scan.end_monotonic)
        self.job_generation = self.generation
        self.job_scan_time = scan.end_monotonic
        self.job_scan_points = points.copy()
        if not len(self.model.map):self.map_geometry = dict(self.agv_geometry)
        self.future = self.pool.submit(self.model.update, points, scan.end_monotonic, odom, rotation_only)

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
        self.finish_trial()
        self.timer.stop(); self.disconnect()
        if self.reader: self.reader.join(2.5)
        self.pool.shutdown(wait=True, cancel_futures=True)
