"""Adapter: V7 algorithms and 3D UI, SAR transport and global interlocks."""
import time
import numpy as np
from PySide6 import QtWidgets
from robotarm_v7 import MainWindow


class ArmWorkspace(MainWindow):
    arm_camera_api = True

    def __init__(self, engine):
        self.engine = engine
        self.state = {}
        self._last_draw = 0.0
        self._force_draw = True
        self._preview_segment = None
        super().__init__()
        self.setMinimumSize(0, 0)
        self.tabs.setMinimumWidth(400)
        self.tabs.setMaximumWidth(560)
        self.tabs.removeTab(self.tabs.count()-1)  # One shared connection, no competing USB link.
        self.chk_pendant.setChecked(False)
        self.chk_pendant.setEnabled(False)
        self.chk_pendant.setText('COCO: seleccionar en la barra global')
        self.btn_send_manual.setText('Enviar pose (prueba)')
        self.btn_send_manual.setToolTip('Envía la pose actual una sola vez; para control continuo usa Live Robot Arm movement.')
        self.slider_pinza.setMinimum(10)
        self.btn_home = QtWidgets.QPushButton('Pose segura (Home)')
        self.btn_home.setToolTip('Recupera la pose inicial en la simulación. Con Live activo también la envía al brazo.')
        self.btn_send_manual.parentWidget().layout().addWidget(self.btn_home)
        self.btn_home.clicked.connect(self.return_home)
        profile_layout = QtWidgets.QFormLayout()
        self.motion_profile = QtWidgets.QComboBox()
        self.motion_profile.addItem('Lineal', False)
        self.motion_profile.addItem('Quíntico', True)
        self.motion_profile.setToolTip('Vista articular, Home y poses de rutinas. Live y COCO mantienen el control continuo.')
        profile_layout.addRow('Perfil articular', self.motion_profile)
        self.btn_send_manual.parentWidget().layout().addLayout(profile_layout)
        self.motion_profile.currentIndexChanged.connect(self.change_motion_profile)
        self.mode_selector.currentIndexChanged.connect(self.clear_preview_segment)
        self.btn_stop_path.clicked.connect(self.stop_robot)
        self.camera_source.setText(self.engine.runtime.get('arm_camera_url', ''))
        self.camera_source.setPlaceholderText('ESP32-S3 IP or base URL, e.g. 192.168.3.17')
        self.camera_source.setToolTip('Discover /api/cameras, activate MJPEG if needed, then open /api/stream/{id}')
        camera_layout=self.camera_source.parentWidget().layout()
        camera_layout.itemAtPosition(0,0).widget().setText('Arm ESP32-S3 IP / URL')
        self.camera_source.editingFinished.connect(self.save_camera_url)
        self.link_status=QtWidgets.QLabel('Brazo: sin telemetría. Preview simulado.')
        self.link_status.setWordWrap(True)
        self.centralWidget().layout().insertWidget(1,self.link_status)
        for box in (self.chk_live,self.chk_path_physical):
            box.toggled.connect(lambda checked,b=box:self.physical_option(b,checked))
        self.engine.signals.cancelled.connect(self.cancel_motion)
        self._force_draw=True
        self._redraw_robot()
        self._update_text_results(self.current_physical_angles,self.current_xyz)

    def save_camera_url(self):
        self.engine.post('arm_camera_url', self.camera_source.text().strip())

    def clear_preview_segment(self):
        self._preview_segment = None

    def change_motion_profile(self):
        self.clear_preview_segment()
        self.engine.arm_quintic = self.motion_profile.currentData()

    def _next_fk_angles(self, diff):
        if not self.engine.arm_quintic or self.chk_live.isChecked():
            return super()._next_fk_angles(diff)
        if not np.any(diff):
            self.clear_preview_segment()
            return None
        now = time.monotonic()
        target = self.slider_target_fk
        if self._preview_segment is None or not np.array_equal(target, self._preview_segment[1]):
            # Scale duration so peak joint speed respects the preview speed setting.
            duration = max(0.05, 1.875 * np.max(np.abs(diff)) / self.max_speed_degs)
            self._preview_segment = (self.current_physical_angles.copy(), target.copy(), now, duration)
        start, goal, began, duration = self._preview_segment
        u = min(1.0, (now - began) / duration)
        if u >= 1:
            return goal.copy()
        return start + (goal - start) * (u**3 * (10 + u * (-15 + 6*u)))

    def return_home(self):
        if self.engine.source != 'PC' or self.path_active:
            self._show_msg('Selecciona control PC y detén la trayectoria antes de volver a Home.', True)
            return
        live = self.chk_live.isChecked()
        if live:
            reason = self.block_reason()
            if reason:
                self._show_msg(reason, True)
                return
        self.mode_selector.setCurrentIndex(0)
        self._force_draw = True
        self.robot.last_ik_theta = self.robot.convert_angles(self.robot.startup_angles)
        if self.engine.arm_quintic and not live:
            goal = self.robot.startup_angles.copy()
            for slider, angle in zip(self.fk_sliders, goal):
                slider.setValue(round(angle * 10))
            self.slider_pinza.setValue(round(self.robot.home_gripper))
            self._on_slider_changed()
            self._preview_segment = (self.current_physical_angles.copy(), goal, time.monotonic(), 2.0)
        else:
            self.sync_pose(self.robot.startup_angles, self.robot.home_gripper)
            self._on_slider_changed()
        if live:
            self.engine.post('arm_home', None)
        self._show_msg('Home solicitado al AGV.' if live else 'Pose inicial en la simulación.')

    def block_reason(self):
        reason=self.engine.arm_motion_block('PC')
        if reason:return reason
        if self.engine.live or self.engine.primitive or self.engine.routine or self.engine.capture:
            return 'Detener el AGV antes de mover el brazo'
        if not self.state.get('fresh') or not self.valid_angles(self.state.get('arm_angles')):
            return 'Falta telemetría válida de los servos'
        return ''

    @staticmethod
    def valid_angles(values):
        try:
            a=np.asarray(values,dtype=float)
            return a.shape==(4,) and np.all(np.isfinite(a)) and np.all((a>=0)&(a<=180))
        except (ValueError,TypeError):return False

    def physical_option(self,box,checked):
        if not checked:return
        reason=self.block_reason()
        if reason:
            box.setChecked(False);self._show_msg(reason,True);return
        # Rebase the virtual arm on firmware PWM estimates before live motion.
        self.cancel_motion(clear_options=False)
        self.sync_pose(self.state['arm_angles'],self.state.get('arm_gripper',60))
        self._show_msg('Control físico habilitado desde la pose PWM estimada. No es una medición articular.')

    def sync_pose(self,angles,gripper):
        if not self.valid_angles(angles):return
        self.clear_preview_segment()
        self._syncing=True
        self.current_physical_angles=np.array(angles,dtype=float)
        self.current_xyz,_=self.robot.compute_fk(self.current_physical_angles)
        self.slider_target_fk=self.current_physical_angles.copy()
        self.slider_target_xyz=self.current_xyz.copy()
        self.pendant_target_xyz=self.current_xyz.copy()
        for slider,value in zip(self.fk_sliders,angles):slider.setValue(round(value*10))
        for slider,value in zip(self.ik_sliders,self.current_xyz):slider.setValue(round(float(value)))
        self.slider_pinza.setValue(round(float(gripper)))
        self._syncing=False
        self._redraw_robot()
        self._update_text_results(self.current_physical_angles,self.current_xyz)

    def receive_state(self,state):
        self.state=state
        t=state['telemetry']
        ready=state.get('fresh') and t.get('pca_ready')==1
        # V7 uses this only as a connectivity guard. Never a transport object.
        self.arduino_serial=True if ready else None
        self.link_status.setText(f"PCA9685: {'OK' if ready else 'sin datos / no disponible'} | "
            f"Control: {self.engine.source} | Firmware: {'interpolando' if state.get('arm_busy') else 'en espera'} | "
            'Vista y telemetría PWM estimadas; sin encoders articulares')
        physical=any(b.isChecked() for b in (self.chk_live,self.chk_path_physical))
        if physical and self.block_reason():self.cancel_motion()
        if self.engine.source=='COCO' and state.get('fresh'):
            self.sync_pose(state.get('arm_angles'),state.get('arm_gripper',60))

    def cancel_motion(self,clear_options=True):
        self.path_active=False;self.planned_path=[]
        self.vision_motion_queue=[];self.vision_actions=[]
        self.vision_state='IDLE';self.recovery_active=False;self.recovery_stage='IDLE'
        self.vision_detection_armed=False;self.locked_detection=None
        self._force_hardware_send=False
        if clear_options:
            for box in (self.chk_live,self.chk_path_physical):box.setChecked(False)
        self.slider_target_fk=self.current_physical_angles.copy()
        self.slider_target_xyz=np.array(self.current_xyz).copy()
        self.pendant_target_xyz=np.array(self.current_xyz).copy()
        self.sync_pose(self.current_physical_angles,self.slider_pinza.value())

    def stop_robot(self):
        self.cancel_motion();self.engine.post('stop',False)

    def _send_to_arduino(self,live_mode=False):
        if not live_mode and self.chk_live.isChecked():
            self._show_msg('Desactiva Live Robot Arm movement para enviar una pose de prueba.',True);return
        reason=self.block_reason()
        if reason:
            self.cancel_motion();self._show_msg(reason,True);return
        grip=float(self.slider_pinza.value())
        if not 10<=grip<=75:
            self.cancel_motion();self._show_msg('Pinza fuera del rango SAR 10–75°',True);return
        self.engine.post('arm_stream_pose' if live_mode else 'arm_pose',(self.current_physical_angles.copy(),grip,150 if live_mode else 2000))
        if not live_mode:self._show_msg('Pose enviada a la cola SAR; confirmar estado del firmware en la barra global.')

    def _control_loop_tick(self):
        if self.engine.source!='PC':return
        super()._control_loop_tick()

    def _draw_robot(self,transforms):
        # Bound expensive Matplotlib rendering; control keeps its 50 ms cadence.
        if hasattr(self,'canvas') and not self.canvas.isVisible() and not self._force_draw:return
        now=time.monotonic()
        if not self._force_draw and now-self._last_draw<.20:return
        self._force_draw=False;self._last_draw=now
        super()._draw_robot(transforms)

    def _trigger_safe_recovery(self,reason):
        # Camera processing cannot initiate recovery or any physical movement.
        self.stop_robot();self._show_msg(reason,True)

    def _connect_arduino(self):self._show_msg('Usar Connect AGV en la barra global.')
    def _connect_pendant(self,automatic=False):self._show_msg('Usar Connect COCO en la barra global.')
    def _maintain_pendant_connection(self,current_time):pass
