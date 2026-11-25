"""Two independent camera processes; unavailable streams never block the Qt UI."""
import base64
import json
import sys
import time
import hashlib
from pathlib import Path
from PySide6 import QtCore, QtGui, QtWidgets


class CalibrationDialog(QtWidgets.QDialog):
    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Calibrar cámara AGV')
        self.resize(720, 600)
        self.output = str(Path(__file__).with_name('camera_agv_calibration.npz'))
        self.buffer = b''
        self.calculating = False
        self.proc = QtCore.QProcess(self)
        self.proc.readyReadStandardOutput.connect(self.read_output)
        self.proc.readyReadStandardError.connect(lambda: self.proc.readAllStandardError())
        self.proc.errorOccurred.connect(lambda _: self.status.setText('No se pudo iniciar la calibración.'))
        self.proc.finished.connect(self.finished)
        layout = QtWidgets.QVBoxLayout(self)
        instructions = QtWidgets.QLabel('Tablero: 10 × 8 cuadros, 25 mm por cuadro (9 × 7 esquinas interiores). Mantén el AGV quieto y mueve el tablero. Captura al menos 12 vistas completas con posiciones e inclinaciones distintas; se recomiendan 20–30.')
        instructions.setWordWrap(True); layout.addWidget(instructions)
        self.preview = QtWidgets.QLabel('Esperando cámara…')
        self.preview.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.preview.setMinimumSize(320, 240); layout.addWidget(self.preview, 1)
        self.status = QtWidgets.QLabel('Abriendo cámara…'); self.status.setWordWrap(True); layout.addWidget(self.status)
        buttons = QtWidgets.QHBoxLayout(); layout.addLayout(buttons)
        self.capture_button = QtWidgets.QPushButton('Capturar vista'); self.capture_button.clicked.connect(lambda: self.send('capture')); buttons.addWidget(self.capture_button)
        self.finish_button = QtWidgets.QPushButton('Calcular y guardar'); self.finish_button.setEnabled(False)
        self.finish_button.clicked.connect(self.finish_calibration); buttons.addWidget(self.finish_button)
        cancel = QtWidgets.QPushButton('Cancelar'); cancel.clicked.connect(self.reject); buttons.addWidget(cancel)
        self.capture_button.setEnabled(False)
        config = dict(config, output=self.output)
        self.proc.start(sys.executable, ['-u', str(Path(__file__).with_name('camera_calibration.py')), json.dumps(config)])

    def send(self, command):
        if self.proc.state() == QtCore.QProcess.ProcessState.Running:
            self.proc.write((command + '\n').encode())

    def finish_calibration(self):
        self.calculating = True
        self.capture_button.setEnabled(False); self.finish_button.setEnabled(False)
        self.status.setText('Calculando calibración…'); self.send('finish')

    def read_output(self):
        self.buffer += bytes(self.proc.readAllStandardOutput())
        if len(self.buffer) > 4000000:
            self.proc.kill(); self.status.setText('Buffer de calibración excedido.'); return
        lines = self.buffer.split(b'\n'); self.buffer = lines.pop()
        for line in lines:
            try: packet = json.loads(line)
            except (ValueError, TypeError): continue
            if 'error' in packet: self.status.setText(packet['error'])
            if 'status' in packet:
                self.status.setText(packet['status'])
                if 'count' in packet:
                    self.calculating = False
                    self.capture_button.setEnabled(True); self.finish_button.setEnabled(packet['count'] >= 12)
            if 'jpeg' in packet:
                image = QtGui.QImage.fromData(base64.b64decode(packet['jpeg']))
                if not image.isNull():
                    if not self.calculating:self.capture_button.setEnabled(True)
                    pix = QtGui.QPixmap.fromImage(image)
                    self.preview.setPixmap(pix.scaled(self.preview.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio, QtCore.Qt.TransformationMode.FastTransformation))
            if 'completed' in packet:
                self.rms = packet['rms']; self.accept(); return

    def finished(self, *_):
        self.capture_button.setEnabled(False); self.finish_button.setEnabled(False)
        if self.result() != QtWidgets.QDialog.DialogCode.Accepted:
            self.status.setText('Calibración detenida. ' + self.status.text())

    def done(self, result):
        if self.proc.state() != QtCore.QProcess.ProcessState.NotRunning:
            self.proc.kill(); self.proc.waitForFinished(2000)
        super().done(result)


class CameraPanel(QtWidgets.QGroupBox):
    calibration_saved = QtCore.Signal()
    preview_frame = QtCore.Signal(object)
    marker_sample = QtCore.Signal(object)
    alignment_requested = QtCore.Signal(object)
    sequence_requested = QtCore.Signal(object)
    alignment_stop = QtCore.Signal()
    approach_pose_requested = QtCore.Signal()
    pickup_requested = QtCore.Signal()
    release_requested = QtCore.Signal()
    def __init__(self,title,config,parent=None,arm_camera_api=False):
        super().__init__(title,parent)
        self.proc=QtCore.QProcess(self);self.buffer=b'';self.last_frame=0;self.last_activity=0;self.stopping=False
        self.arm_camera_api=arm_camera_api;self.received_frame=False;self.restart_on_finish=False
        self.marker_readings=[];self.active_config=None;self.active_identity=None
        self.reference_path=Path(__file__).with_name('aruco_agv_reference.json')
        self.proc.readyReadStandardOutput.connect(self.read_output)
        self.proc.readyReadStandardError.connect(lambda:self.proc.readAllStandardError())
        self.proc.finished.connect(self.finished)
        self.proc.errorOccurred.connect(lambda _:self.status.setText('Error al iniciar el proceso de cámara'))
        box=QtWidgets.QVBoxLayout(self)
        advanced_toggle=QtWidgets.QPushButton('Configuración y calibración ▸');advanced_toggle.setCheckable(True);box.addWidget(advanced_toggle)
        advanced=QtWidgets.QWidget();advanced_layout=QtWidgets.QVBoxLayout(advanced);advanced.setVisible(False);box.addWidget(advanced)
        advanced_toggle.toggled.connect(advanced.setVisible)
        form=QtWidgets.QFormLayout();advanced_layout.addLayout(form)
        self.url=QtWidgets.QLineEdit(config.get('url',''));self.url.setPlaceholderText('http://192.168.x.x:81/stream')
        form.addRow('ESP32-S3 · URL',self.url)
        if self.arm_camera_api:
            self.url.setPlaceholderText('ESP32-S3 IP or base URL, e.g. 192.168.3.17')
            form.labelForField(self.url).setText('Arm ESP32-S3 IP / URL')
        self.dictionary=QtWidgets.QComboBox();self.dictionary.addItems(['DICT_4X4_50','DICT_4X4_100','DICT_5X5_100','DICT_6X6_250'])
        self.dictionary.setCurrentText(config.get('dictionary','DICT_4X4_50'));form.addRow('Diccionario ArUco',self.dictionary)
        self.rotation=QtWidgets.QComboBox()
        for label,degrees in [('Sin giro (0°)',0),('90° horario',90),('90° antihorario',-90),('180°',180)]:
            self.rotation.addItem(label,degrees)
        rotation_index=self.rotation.findData(config.get('rotation',0))
        self.rotation.setCurrentIndex(rotation_index if rotation_index>=0 else 0)
        form.addRow('Rotación del preview',self.rotation)
        self.marker=QtWidgets.QDoubleSpinBox();self.marker.setRange(1,1000);self.marker.setValue(config.get('marker_mm',35));self.marker.setSuffix(' mm');form.addRow('Lado exterior marcador',self.marker)
        self.calibration=QtWidgets.QLineEdit(config.get('calibration',''));form.addRow('Calibración .npz',self.calibration)
        select=QtWidgets.QPushButton('Elegir calibración');select.clicked.connect(self.select_calibration);advanced_layout.addWidget(select)
        if not self.arm_camera_api:
            calibrate=QtWidgets.QPushButton('Calibrar cámara AGV');calibrate.clicked.connect(self.calibrate_camera);advanced_layout.addWidget(calibrate)
            pose=QtWidgets.QPushButton('Colocar brazo en postura de aproximación')
            pose.setToolTip('Primero postura ArUco; después base 108.5°, hombro 149.4°, codo 180°, muñeca 6.8° y pinza 75°.')
            pose.clicked.connect(self.approach_pose_requested.emit);advanced_layout.addWidget(pose)
            teach=QtWidgets.QPushButton('Guardar referencia ArUco 12');teach.clicked.connect(self.save_reference);advanced_layout.addWidget(teach)
            alignment=QtWidgets.QHBoxLayout();box.addLayout(alignment)
            align=QtWidgets.QPushButton('Iniciar secuencia ArUco');align.clicked.connect(self.start_alignment);alignment.addWidget(align)
            resume=QtWidgets.QPushButton('Continuar alineación');resume.clicked.connect(self.continue_alignment);alignment.addWidget(resume)
            stop_alignment=QtWidgets.QPushButton('STOP AGV');stop_alignment.clicked.connect(self.alignment_stop.emit);alignment.addWidget(stop_alignment)
            pickup=QtWidgets.QPushButton('PICK');pickup.clicked.connect(self.pickup_requested.emit);box.addWidget(pickup)
            pickup.setToolTip('Approach → PICK → cerrar pinza → DEPART → SAFE_Rescue. Verifica primero la distancia y el recorrido del brazo.')
            release=QtWidgets.QPushButton('UNPICK');release.clicked.connect(self.release_requested.emit);box.addWidget(release)
            release.setToolTip('SAFE_Rescue → DEPART → PICK → abrir pinza → Approach.')
            message=('Referencia guardada disponible; conecta la cámara para comprobarla.' if self.reference_path.is_file()
                     else 'Sin referencia guardada: centra la pieza con el gripper a la distancia de recogida y guarda la referencia.')
            self.alignment_status=QtWidgets.QLabel(message);self.alignment_status.setWordWrap(True);box.addWidget(self.alignment_status)
        row=QtWidgets.QHBoxLayout();box.addLayout(row)
        self.connect_button=QtWidgets.QPushButton('Conectar');self.connect_button.clicked.connect(self.start);row.addWidget(self.connect_button)
        stop=QtWidgets.QPushButton('Detener');stop.clicked.connect(self.stop);row.addWidget(stop)
        self.status=QtWidgets.QLabel('Desconectada');self.status.setWordWrap(True);box.addWidget(self.status)
        self.preview=QtWidgets.QLabel('ESP32-S3 · sin imagen');self.preview.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.preview.setMinimumSize(280,180);self.preview.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,QtWidgets.QSizePolicy.Policy.Expanding)
        self.table=QtWidgets.QTableWidget(0,5);self.table.setHorizontalHeaderLabels(['ID','X cámara mm','Y cámara mm','Z cámara mm','Distancia mm'])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch);self.table.setMaximumHeight(160)
        view=QtWidgets.QHBoxLayout();box.addLayout(view,1);view.addWidget(self.preview,3)
        details=QtWidgets.QVBoxLayout();view.addLayout(details,2)
        self.distance_label=QtWidgets.QLabel('Distancia aproximada: —');self.distance_label.setWordWrap(True);details.addWidget(self.distance_label)
        details.addWidget(self.table);details.addStretch()
        note=QtWidgets.QLabel('Alinea el AGV y verifica la distancia y el recorrido del brazo antes de iniciar PICK o UNPICK.');note.setWordWrap(True);details.addWidget(note)
        self.watchdog=QtCore.QTimer(self);self.watchdog.timeout.connect(self.check_age);self.watchdog.start(500)

    def config(self):return {'url':self.url.text().strip(),'dictionary':self.dictionary.currentText(),'marker_mm':self.marker.value(),'calibration':self.calibration.text().strip(),'rotation':self.rotation.currentData(),'arm_camera_api':self.arm_camera_api}
    def reference_identity(self):
        cfg=self.config()
        if cfg != self.active_config:raise ValueError('La configuración cambió; reconecta la cámara.')
        if not cfg['calibration']:raise ValueError('Carga la calibración de la cámara.')
        digest=hashlib.sha256(Path(cfg['calibration']).read_bytes()).hexdigest()
        return {'url':cfg['url'],'dictionary':cfg['dictionary'],'marker_mm':cfg['marker_mm'],'calibration_sha256':digest}
    @staticmethod
    def adapt_reference_size(ref,identity):
        saved=ref['identity']
        for key in ('url','dictionary','calibration_sha256'):
            if saved.get(key)!=identity.get(key):
                labels={'url':'URL de cámara','dictionary':'diccionario ArUco','calibration_sha256':'archivo de calibración'}
                raise ValueError(f"Cambió {labels[key]}; guarda otra referencia.")
        old=float(saved['marker_mm']);new=float(identity['marker_mm'])
        if not (old>0 and new>0):raise ValueError('Tamaño de marcador inválido.')
        if old==new:return ref,False
        adapted=dict(ref)
        adapted['xyz_camera_mm']=[float(v)*new/old for v in ref['xyz_camera_mm']]
        if 'target_distance_mm' in ref:
            adapted['target_distance_mm']=ref['target_distance_mm']*new/old
        adapted['identity']=identity
        return adapted,True
    def save_reference(self):
        try:
            identity=self.reference_identity();now=time.monotonic()
            samples=[s for s in self.marker_readings if now-s['captured_at']<1.5]
            if len(samples)<5:raise ValueError('Se necesitan 5 lecturas recientes y estables del marcador 12.')
            import numpy as np
            positions=np.array([s['xyz_camera_mm'] for s in samples])
            if np.max(np.ptp(positions,axis=0))>5:raise ValueError('Las lecturas varían más de 5 mm; mantén el AGV quieto.')
            position=np.mean(positions,axis=0)
            distance=float(np.linalg.norm(position))
            if not 170<=distance<=220:
                direction='acerca' if distance>220 else 'aleja'
                raise ValueError(f'Distancia estimada: {distance:.0f} mm; se requieren 170–220 mm. {direction.capitalize()} el marcador y verifica que su lado exterior mida {self.marker.value():g} mm.')
            ref={'xyz_camera_mm':position.tolist(),'target_distance_mm':distance,'rvec':samples[-1]['rvec'],
                 'identity':identity,'image_size':samples[-1]['image_size']}
            temp=self.reference_path.with_suffix('.tmp');temp.write_text(json.dumps(ref,indent=2),encoding='utf8');temp.replace(self.reference_path)
            self.alignment_status.setText(f'Referencia guardada a {distance:.1f} mm de cámara. El brazo debe estar detenido para alinear.')
        except (OSError,ValueError,KeyError,TypeError) as exc:self.alignment_status.setText('No se guardó la referencia: '+str(exc))
    def start_alignment(self):
        self.request_alignment(sequence=True)
    def continue_alignment(self):
        self.request_alignment(sequence=False)
    def get_alignment_reference(self):
        ref=json.loads(self.reference_path.read_text(encoding='utf8'))
        ref,_=self.adapt_reference_size(ref,self.reference_identity())
        if self.proc.state()==QtCore.QProcess.ProcessState.NotRunning:raise ValueError('Conecta primero la cámara AGV.')
        if (self.marker_readings and time.monotonic()-self.marker_readings[-1]['captured_at']<=.5
                and ref['image_size']!=self.marker_readings[-1]['image_size']):raise ValueError('Cambió la resolución; guarda otra referencia.')
        return ref
    def request_alignment(self,sequence):
        try:
            ref=json.loads(self.reference_path.read_text(encoding='utf8'))
            ref,rescaled=self.adapt_reference_size(ref,self.reference_identity())
            if self.proc.state()==QtCore.QProcess.ProcessState.NotRunning:raise ValueError('Conecta primero la cámara AGV.')
            if (self.marker_readings and time.monotonic()-self.marker_readings[-1]['captured_at']<=.5
                    and ref['image_size']!=self.marker_readings[-1]['image_size']):raise ValueError('Cambió la resolución; guarda otra referencia.')
            if sequence:self.sequence_requested.emit(ref)
            else:self.alignment_requested.emit(ref)
            note=' Referencia ajustada al tamaño actual del marcador; verifica la medida física.' if rescaled else ''
            self.alignment_status.setText(('Secuencia solicitada: brazo y acercamiento.' if sequence else 'Continuar alineación solicitado; el brazo no se mueve.')+' STOP cancela.'+note)
        except (OSError,ValueError,KeyError,TypeError) as exc:self.alignment_status.setText('No se inició: '+str(exc))
    def select_calibration(self):
        path,_=QtWidgets.QFileDialog.getOpenFileName(self,'Calibración de esta cámara','','NumPy (*.npz)')
        if path:self.calibration.setText(path)
    def calibrate_camera(self):
        if not self.url.text().strip():
            self.status.setText('Introduce la URL de la cámara antes de calibrar.');return
        was_running = self.proc.state()!=QtCore.QProcess.ProcessState.NotRunning
        if was_running:
            self.stop()
            if not self.proc.waitForFinished(2000):
                self.proc.kill();self.proc.waitForFinished(1000)
        dialog = CalibrationDialog(self.config(), self)
        accepted = dialog.exec() == QtWidgets.QDialog.DialogCode.Accepted
        if accepted:
            self.calibration.setText(dialog.output)
            self.calibration_saved.emit()
        if was_running:self.start()
        if accepted:
            QtWidgets.QMessageBox.information(self, 'Calibración guardada', f'Calibración guardada. Error de reproyección RMS: {dialog.rms:.2f} px. Verifica las distancias con una regla.')
    def start(self):
        if self.proc.state()!=QtCore.QProcess.ProcessState.NotRunning:return
        self.preview_frame.emit(None)
        self.stopping=False;self.restart_on_finish=False;self.buffer=b'';self.last_frame=time.monotonic();self.last_activity=self.last_frame
        self.received_frame=False
        self.status.setText('Conectando…');self.connect_button.setEnabled(False)
        self.active_config=self.config();self.marker_readings=[];self.marker_sample.emit(None)
        try:self.active_identity=self.reference_identity()
        except (OSError,ValueError):self.active_identity=None
        self.proc.start(sys.executable,['-u',str(Path(__file__).with_name('camera_worker.py')),json.dumps(self.active_config)])
    def stop(self):
        self.marker_readings=[];self.marker_sample.emit(None)
        self.preview_frame.emit(None)
        self.distance_label.setText('Distancia aproximada: sin conexión')
        self.stopping=True;self.table.setRowCount(0);self.preview.clear();self.preview.setText('Sin imagen · detenida')
        self.proc.terminate();QtCore.QTimer.singleShot(500,self.kill_if_stopping)
        self.status.setText('Detenida')
    def kill_if_stopping(self):
        if self.stopping and self.proc.state()!=QtCore.QProcess.ProcessState.NotRunning:self.proc.kill()
    def finished(self,*_):
        self.marker_readings=[];self.marker_sample.emit(None)
        self.preview_frame.emit(None)
        self.distance_label.setText('Distancia aproximada: sin conexión')
        restart=self.restart_on_finish and not self.stopping
        self.restart_on_finish=False
        self.connect_button.setEnabled(True);self.table.setRowCount(0)
        self.preview.clear();self.preview.setText('Sin imagen · desconectada')
        if not self.stopping:self.status.setText('Desconectada: '+self.status.text())
        if restart:
            self.status.setText('Worker de cámara bloqueado; reintentando conexión')
            QtCore.QTimer.singleShot(1000,self.restart_if_needed)
    def restart_if_needed(self):
        if not self.stopping and self.proc.state()==QtCore.QProcess.ProcessState.NotRunning:self.start()
    def check_age(self):
        timeout=30
        if self.proc.state()!=QtCore.QProcess.ProcessState.NotRunning and time.monotonic()-self.last_activity>timeout:
            self.preview_frame.emit(None)
            self.table.setRowCount(0);self.preview.clear();self.preview.setText('Sin imagen reciente')
            self.status.setText(f'Sin actividad de cámara durante {timeout} s; cerrando el worker')
            self.restart_on_finish=True
            self.proc.kill()
    def read_output(self):
        self.buffer+=bytes(self.proc.readAllStandardOutput())
        if len(self.buffer)>4000000:
            self.proc.kill();self.status.setText('Buffer de cámara excedido');return
        lines=self.buffer.split(b'\n');self.buffer=lines.pop()
        # Drop old previews; only the newest complete frame belongs on screen.
        if not lines:return
        try:
            packet=json.loads(lines[-1])
            self.last_activity=time.monotonic()
            if 'error' in packet:self.preview_frame.emit(None);self.status.setText(packet['error']);return
            if 'status' in packet:
                if 'desconectada' in packet['status'].lower() or 'reintento' in packet['status'].lower():self.preview_frame.emit(None)
                self.status.setText(packet['status']);return
            image=QtGui.QImage.fromData(base64.b64decode(packet['jpeg']))
            if image.isNull():return
            self.preview_frame.emit(image)
            self.last_frame=time.monotonic()
            self.received_frame=True
            pixmap=QtGui.QPixmap.fromImage(image)
            scale=min(1.0,self.preview.width()/pixmap.width(),self.preview.height()/pixmap.height())
            pixmap=pixmap.scaled(max(1,round(pixmap.width()*scale)),max(1,round(pixmap.height()*scale)),
                                 QtCore.Qt.AspectRatioMode.KeepAspectRatio,QtCore.Qt.TransformationMode.FastTransformation)
            self.preview.setPixmap(pixmap)
            rows=packet['detections'];self.table.setRowCount(len(rows))
            markers=[r for r in rows if r.get('id')==12]
            valid=[r for r in markers if 'xyz_camera_mm' in r and 'rvec' in r]
            sample=None
            if len(markers)==len(valid)==1 and 'captured_at' in packet and 'image_size' in packet:
                sample=dict(valid[0],captured_at=packet['captured_at'],image_size=packet['image_size'],identity=self.active_identity)
                self.marker_readings=(self.marker_readings+[sample])[-10:]
            # One missed frame must not erase a recent valid pose. Explicit
            # disconnects still clear it, and the controller checks its age.
            if sample is not None:self.marker_sample.emit(sample)
            for i,row in enumerate(rows):
                values=[str(row['id'])]+[f'{v:.1f}' for v in row.get('xyz_camera_mm',[])]
                if len(values)==1:values+=['—']*3
                values+=[f"{row['distance_mm']:.1f}" if 'distance_mm' in row else '—']
                for j,value in enumerate(values):self.table.setItem(i,j,QtWidgets.QTableWidgetItem(value))
            errors=[r['error'] for r in rows if 'error' in r]
            if sample and 'distance_mm' in sample:distance_info=f" · ArUco 12: {sample['distance_mm']:.0f} mm"
            elif self.arm_camera_api:distance_info=''
            elif len(markers)>1:distance_info=' · ArUco 12 duplicado'
            elif markers:distance_info=' · ArUco 12 visible, sin pose válida'
            else:distance_info=' · ArUco 12 no detectado'
            self.distance_label.setText(f"Distancia cámara: {sample['distance_mm']/10:.1f} cm\nComparación +12 cm: {sample['distance_mm']/10+12:.1f} cm (no controla el AGV)" if sample and 'distance_mm' in sample else 'Distancia aproximada: sin lectura válida')
            self.status.setText(f'ONLINE · {len(rows)} marcadores · '+('calibrada' if packet['metric'] else 'sin calibración: solo IDs/píxeles')+distance_info+(' · '+errors[0] if errors else ''))
        except (ValueError,KeyError,TypeError):self.status.setText('Frame inválido')
    def shutdown(self):
        self.preview_frame.emit(None);self.stopping=True;self.watchdog.stop();self.proc.kill();self.proc.waitForFinished(1000)


class CamerasPage(QtWidgets.QWidget):
    def __init__(self):
        super().__init__();self.path=Path(__file__).with_name('cameras.json')
        try:configs=json.loads(self.path.read_text())
        except (OSError,ValueError):configs=[{},{}]
        if not isinstance(configs,list) or len(configs)!=2:configs=[{},{}]
        box=QtWidgets.QVBoxLayout(self)
        note=QtWidgets.QLabel('Cámara AGV · ArUco 12. Verifica la distancia con una regla antes de usar la secuencia. El recorrido debe estar despejado.')
        note.setWordWrap(True);box.addWidget(note)
        self.other_config=configs[1] if isinstance(configs[1],dict) else {}
        self.panels=[CameraPanel('Cámara AGV',configs[0] if isinstance(configs[0],dict) else {})]
        for panel in self.panels:panel.calibration_saved.connect(self.save)
        for panel in self.panels:box.addWidget(panel,1)
        button=QtWidgets.QPushButton('Guardar URLs y calibraciones');button.clicked.connect(self.save);box.addWidget(button)
        self.saved=QtWidgets.QLabel('');box.addWidget(self.saved)
    def save(self):
        try:
            temp=self.path.with_suffix('.tmp');temp.write_text(json.dumps([self.panels[0].config(),self.other_config],indent=2));temp.replace(self.path)
            self.saved.setText('Configuración guardada')
        except OSError as exc:self.saved.setText(f'No se pudo guardar: {exc}')
    def shutdown(self):
        self.save()
        for panel in self.panels:panel.shutdown()
