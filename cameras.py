"""Two independent camera processes; unavailable streams never block the Qt UI."""
import base64
import json
import sys
import time
from pathlib import Path
from PySide6 import QtCore, QtGui, QtWidgets


class CameraPanel(QtWidgets.QGroupBox):
    def __init__(self,title,config,parent=None):
        super().__init__(title,parent)
        self.proc=QtCore.QProcess(self);self.buffer=b'';self.last_frame=0;self.stopping=False
        self.proc.readyReadStandardOutput.connect(self.read_output)
        self.proc.readyReadStandardError.connect(lambda:self.proc.readAllStandardError())
        self.proc.finished.connect(self.finished)
        self.proc.errorOccurred.connect(lambda _:self.status.setText('Error al iniciar el proceso de cámara'))
        box=QtWidgets.QVBoxLayout(self);form=QtWidgets.QFormLayout();box.addLayout(form)
        self.url=QtWidgets.QLineEdit(config.get('url',''));self.url.setPlaceholderText('http://192.168.x.x:81/stream')
        form.addRow('ESP32-S3 · URL',self.url)
        self.dictionary=QtWidgets.QComboBox();self.dictionary.addItems(['DICT_4X4_50','DICT_4X4_100','DICT_5X5_100','DICT_6X6_250'])
        self.dictionary.setCurrentText(config.get('dictionary','DICT_4X4_50'));form.addRow('Diccionario ArUco',self.dictionary)
        self.marker=QtWidgets.QDoubleSpinBox();self.marker.setRange(1,1000);self.marker.setValue(config.get('marker_mm',35));self.marker.setSuffix(' mm');form.addRow('Lado exterior marcador',self.marker)
        self.calibration=QtWidgets.QLineEdit(config.get('calibration',''));form.addRow('Calibración .npz',self.calibration)
        select=QtWidgets.QPushButton('Elegir calibración');select.clicked.connect(self.select_calibration);box.addWidget(select)
        row=QtWidgets.QHBoxLayout();box.addLayout(row)
        self.connect_button=QtWidgets.QPushButton('Conectar');self.connect_button.clicked.connect(self.start);row.addWidget(self.connect_button)
        stop=QtWidgets.QPushButton('Detener');stop.clicked.connect(self.stop);row.addWidget(stop)
        self.status=QtWidgets.QLabel('Desconectada');self.status.setWordWrap(True);box.addWidget(self.status)
        self.preview=QtWidgets.QLabel('ESP32-S3 · sin imagen');self.preview.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.preview.setMinimumSize(280,180);self.preview.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,QtWidgets.QSizePolicy.Policy.Expanding);box.addWidget(self.preview,1)
        self.table=QtWidgets.QTableWidget(0,5);self.table.setHorizontalHeaderLabels(['ID','X cámara mm','Y cámara mm','Z cámara mm','Distancia mm'])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch);self.table.setMaximumHeight(160);box.addWidget(self.table)
        self.watchdog=QtCore.QTimer(self);self.watchdog.timeout.connect(self.check_age);self.watchdog.start(500)

    def config(self):return {'url':self.url.text().strip(),'dictionary':self.dictionary.currentText(),'marker_mm':self.marker.value(),'calibration':self.calibration.text().strip()}
    def select_calibration(self):
        path,_=QtWidgets.QFileDialog.getOpenFileName(self,'Calibración de esta cámara','','NumPy (*.npz)')
        if path:self.calibration.setText(path)
    def start(self):
        if self.proc.state()!=QtCore.QProcess.ProcessState.NotRunning:return
        self.stopping=False;self.buffer=b'';self.last_frame=time.monotonic()
        self.status.setText('Conectando…');self.connect_button.setEnabled(False)
        self.proc.start(sys.executable,['-u',str(Path(__file__).with_name('camera_worker.py')),json.dumps(self.config())])
    def stop(self):
        self.stopping=True;self.table.setRowCount(0);self.preview.clear();self.preview.setText('Sin imagen · detenida')
        self.proc.terminate();QtCore.QTimer.singleShot(500,self.kill_if_stopping)
        self.status.setText('Detenida')
    def kill_if_stopping(self):
        if self.stopping and self.proc.state()!=QtCore.QProcess.ProcessState.NotRunning:self.proc.kill()
    def finished(self,*_):
        self.connect_button.setEnabled(True);self.table.setRowCount(0)
        self.preview.clear();self.preview.setText('Sin imagen · desconectada')
        if not self.stopping:self.status.setText('Desconectada: '+self.status.text())
    def check_age(self):
        if self.proc.state()!=QtCore.QProcess.ProcessState.NotRunning and time.monotonic()-self.last_frame>4:
            self.table.setRowCount(0);self.preview.clear();self.preview.setText('Sin imagen reciente')
            self.status.setText('Sin frames >4 s; reconectar cámara');self.proc.kill()
    def read_output(self):
        self.buffer+=bytes(self.proc.readAllStandardOutput())
        if len(self.buffer)>4000000:
            self.proc.kill();self.status.setText('Buffer de cámara excedido');return
        lines=self.buffer.split(b'\n');self.buffer=lines.pop()
        # Drop old previews; only the newest complete frame belongs on screen.
        if not lines:return
        try:
            packet=json.loads(lines[-1])
            if 'error' in packet:self.status.setText(packet['error']);return
            image=QtGui.QImage.fromData(base64.b64decode(packet['jpeg']))
            if image.isNull():return
            self.last_frame=time.monotonic()
            pixmap=QtGui.QPixmap.fromImage(image).scaled(self.preview.size(),QtCore.Qt.AspectRatioMode.KeepAspectRatio,QtCore.Qt.TransformationMode.SmoothTransformation)
            self.preview.setPixmap(pixmap)
            rows=packet['detections'];self.table.setRowCount(len(rows))
            for i,row in enumerate(rows):
                values=[str(row['id'])]+[f'{v:.1f}' for v in row.get('xyz_camera_mm',[])]
                if len(values)==1:values+=['—']*3
                values+=[f"{row['distance_mm']:.1f}" if 'distance_mm' in row else '—']
                for j,value in enumerate(values):self.table.setItem(i,j,QtWidgets.QTableWidgetItem(value))
            errors=[r['error'] for r in rows if 'error' in r]
            self.status.setText(f'ONLINE · {len(rows)} marcadores · '+('calibrada' if packet['metric'] else 'sin calibración: solo IDs/píxeles')+(' · '+errors[0] if errors else ''))
        except (ValueError,KeyError,TypeError):self.status.setText('Frame inválido')
    def shutdown(self):
        self.watchdog.stop();self.proc.kill();self.proc.waitForFinished(1000)


class CamerasPage(QtWidgets.QWidget):
    def __init__(self):
        super().__init__();self.path=Path(__file__).with_name('cameras.json')
        try:configs=json.loads(self.path.read_text())
        except (OSError,ValueError):configs=[{},{}]
        if not isinstance(configs,list) or len(configs)!=2:configs=[{},{}]
        box=QtWidgets.QVBoxLayout(self)
        note=QtWidgets.QLabel('Dos ESP32-S3 independientes. Las distancias se expresan respecto a cada cámara. Para llevar un objeto al brazo falta la transformación cámara–base; no se envían movimientos desde esta vista.')
        note.setWordWrap(True);box.addWidget(note)
        split=QtWidgets.QSplitter();box.addWidget(split,1)
        self.panels=[CameraPanel(name,config if isinstance(config,dict) else {}) for name,config in zip(('Cámara AGV','Cámara brazo'),configs)]
        for panel in self.panels:split.addWidget(panel)
        button=QtWidgets.QPushButton('Guardar URLs y calibraciones');button.clicked.connect(self.save);box.addWidget(button)
        self.saved=QtWidgets.QLabel('');box.addWidget(self.saved)
    def save(self):
        try:
            temp=self.path.with_suffix('.tmp');temp.write_text(json.dumps([p.config() for p in self.panels],indent=2));temp.replace(self.path)
            self.saved.setText('Configuración guardada')
        except OSError as exc:self.saved.setText(f'No se pudo guardar: {exc}')
    def shutdown(self):
        self.save()
        for panel in self.panels:panel.shutdown()
