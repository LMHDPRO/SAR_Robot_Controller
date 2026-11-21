import json
import math
import select
import socket
import sys
import threading
import time
from pathlib import Path
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtGui import QFont, QImage, QPixmap
from PySide6.QtWidgets import QApplication, QCheckBox, QComboBox, QFileDialog, QGridLayout, QGroupBox, QHeaderView, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPushButton, QScrollArea, QSlider, QSplitter, QStackedWidget, QTabWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget

def parse_camera_resolution(text: str) -> tuple[int, int]:
    try:
        width, height = text.lower().replace(' ', '').split('x', 1)
        return (max(64, int(width)), max(64, int(height)))
    except Exception:
        return (480, 480)

class RobotKinematics:

    def __init__(self):
        self.d = np.array([127.027, 0, 0, 0])
        self.a = np.array([13.916, 119.401, 122.229, 122.271])
        self.alpha = np.array([-np.pi / 2, 0, 0, 0])
        self.max_reach = np.sum(np.abs(self.d)) + np.sum(np.abs(self.a))
        self.servo_offset = np.array([0.0, 126.5, 74.2, 106.0])
        self.servo_dir = np.array([1.0, -1.0, 1.0, -1.0])
        self.startup_angles = np.array([108.5, 149.5, 180.0, 90.0])
        phys_min = np.array([0.0] * 4)
        phys_max = np.array([180.0] * 4)
        dh_bounds_1 = self.convert_angles(phys_min)
        dh_bounds_2 = self.convert_angles(phys_max)
        self.limit_min = np.minimum(dh_bounds_1, dh_bounds_2)
        self.limit_max = np.maximum(dh_bounds_1, dh_bounds_2)
        self.last_ik_theta = self.convert_angles(self.startup_angles)

    def convert_angles(self, values, to_servo=False):
        values = np.asarray(values, dtype=float)
        return np.rad2deg(values) * self.servo_dir + self.servo_offset if to_servo else np.deg2rad((values - self.servo_offset) * self.servo_dir)

    def compute_fk_dh(self, theta_input):
        theta = [theta_input[0], theta_input[1] - np.pi / 2, theta_input[2], theta_input[3]]
        T = np.eye(4)
        T_matrices = [T.copy()]
        for i in range(4):
            ct, st = (np.cos(theta[i]), np.sin(theta[i]))
            ca, sa = (np.cos(self.alpha[i]), np.sin(self.alpha[i]))
            A = np.array([[ct, -st * ca, st * sa, self.a[i] * ct], [st, ct * ca, -ct * sa, self.a[i] * st], [0, sa, ca, self.d[i]], [0, 0, 0, 1]])
            T = T @ A
            T_matrices.append(T.copy())
        return (T[0:3, 3], T_matrices)

    def compute_fk(self, servo_deg):
        return self.compute_fk_dh(self.convert_angles(servo_deg))

    def compute_ik(self, target_xyz):
        target_xyz = np.array(target_xyz, dtype=float)
        if not np.all(np.isfinite(target_xyz)):
            return (False, None, 'WARN:NON_FINITE_TARGET')
        safe_reach = self.max_reach * 0.98
        if np.linalg.norm(target_xyz) > safe_reach:
            return (False, None, 'WARN:SINGULARITY')
        theta = self.last_ik_theta.copy()
        best_theta = theta.copy()
        best_error = float('inf')
        accept_error_mm = 1.0
        max_final_error_mm = 2.0
        for _ in range(80):
            for i in range(3, -1, -1):
                pos, T_matrices = self.compute_fk_dh(theta)
                end_effector = pos
                current_error = float(np.linalg.norm(end_effector - target_xyz))
                if current_error < best_error:
                    best_error = current_error
                    best_theta = theta.copy()
                if current_error < accept_error_mm:
                    break
                P_i = T_matrices[i][0:3, 3]
                Z_i = T_matrices[i][0:3, 2]
                V_cur = end_effector - P_i
                V_tgt = target_xyz - P_i
                V_cur_proj = V_cur - np.dot(V_cur, Z_i) * Z_i
                V_tgt_proj = V_tgt - np.dot(V_tgt, Z_i) * Z_i
                mag_cur = np.linalg.norm(V_cur_proj)
                mag_tgt = np.linalg.norm(V_tgt_proj)
                if mag_cur < 0.0001 or mag_tgt < 0.0001:
                    continue
                V_cur_proj /= mag_cur
                V_tgt_proj /= mag_tgt
                cos_angle = np.clip(np.dot(V_cur_proj, V_tgt_proj), -1.0, 1.0)
                sin_angle = np.dot(Z_i, np.cross(V_cur_proj, V_tgt_proj))
                delta_theta = np.clip(np.arctan2(sin_angle, cos_angle), -0.1, 0.1)
                theta[i] += delta_theta
                theta[i] = np.clip(theta[i], self.limit_min[i], self.limit_max[i])
            if best_error < accept_error_mm:
                break
        final_pos, _ = self.compute_fk_dh(best_theta)
        error = float(np.linalg.norm(final_pos - target_xyz))
        final_servo_angles = self.convert_angles(best_theta, to_servo=True)
        if error > max_final_error_mm or not np.all(np.isfinite(final_servo_angles)) or np.any(final_servo_angles < -0.25) or np.any(final_servo_angles > 180.25):
            return (False, None, f'WARN:IK_ERROR={error:.1f}mm')
        self.last_ik_theta = best_theta
        final_servo_angles = np.clip(final_servo_angles, 0.0, 180.0)
        return (True, final_servo_angles, error)

class PendantLink:

    def __init__(self):
        self.mode = None
        self.serial_obj = None
        self.sock = None
        self.rx_buffer = bytearray()
        self.name = 'Disconnected'

    def __bool__(self):
        return self.mode == 'serial' and self.serial_obj is not None and self.serial_obj.is_open or (self.mode == 'tcp' and self.sock is not None)

    def connect_serial(self, port, baud=115200):
        self.close()
        serial_obj = serial.Serial(port, baud, timeout=0, write_timeout=0.25)
        self.serial_obj = serial_obj
        self.mode = 'serial'
        self.name = f'USB {port}'

    def connect_tcp(self, ip, port, timeout=3.0):
        self.close()
        sock = socket.create_connection((ip, int(port)), timeout=timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        try:
            sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 3000, 1000))
        except (AttributeError, OSError):
            pass
        sock.setblocking(False)
        self.sock = sock
        self.mode = 'tcp'
        self.name = f'WiFi TCP {ip}:{int(port)}'

    def close(self):
        try:
            if self.serial_obj is not None:
                self.serial_obj.close()
        except Exception:
            pass
        try:
            if self.sock is not None:
                self.sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass
        self.mode = None
        self.serial_obj = None
        self.sock = None
        self.rx_buffer.clear()
        self.name = 'Disconnected'

    @property
    def in_waiting(self):
        if self.mode == 'serial' and self.serial_obj is not None:
            try:
                return self.serial_obj.in_waiting
            except Exception:
                self.close()
                return 0
        if self.mode == 'tcp' and self.sock is not None:
            if b'\n' in self.rx_buffer:
                return 1
            try:
                readable, _, exceptional = select.select([self.sock], [], [self.sock], 0)
                if exceptional:
                    self.close()
                    return 0
                return 1 if readable else 0
            except Exception:
                self.close()
                return 0
        return 0

    def readline(self):
        if self.mode == 'serial' and self.serial_obj is not None:
            try:
                data = self.serial_obj.readline()
                return data
            except Exception:
                self.close()
                return b''
        if self.mode == 'tcp' and self.sock is not None:
            if b'\n' in self.rx_buffer:
                line, _, rest = self.rx_buffer.partition(b'\n')
                self.rx_buffer = bytearray(rest)
                return bytes(line).rstrip(b'\r') + b'\n'
            try:
                while True:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        self.close()
                        return b''
                    self.rx_buffer.extend(chunk)
                    if b'\n' in self.rx_buffer:
                        line, _, rest = self.rx_buffer.partition(b'\n')
                        self.rx_buffer = bytearray(rest)
                        return bytes(line).rstrip(b'\r') + b'\n'
            except BlockingIOError:
                return b''
            except Exception:
                self.close()
                return b''
        return b''

    def write(self, data):
        if isinstance(data, str):
            data = data.encode('utf-8')
        if self.mode == 'serial' and self.serial_obj is not None:
            try:
                sent = self.serial_obj.write(data)
                return sent
            except Exception:
                self.close()
                return 0
        if self.mode == 'tcp' and self.sock is not None:
            try:
                self.sock.sendall(data)
                return len(data)
            except Exception:
                self.close()
                return 0
        return 0

    def tcp_health_ok(self):
        if self.mode != 'tcp' or self.sock is None:
            return self.mode != 'tcp'
        try:
            if self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR) != 0:
                self.close()
                return False
            readable, _, exceptional = select.select([self.sock], [], [self.sock], 0)
            if exceptional:
                self.close()
                return False
            if readable:
                try:
                    peek = self.sock.recv(1, socket.MSG_PEEK)
                    if peek == b'':
                        self.close()
                        return False
                except BlockingIOError:
                    pass
            return True
        except Exception:
            self.close()
            return False

class VisionThread(QThread):
    frame_ready = Signal(QImage)
    status_ready = Signal(str)
    fps_ready = Signal(float)

    def __init__(self, stream_url: str, output_size=(320, 320), mirror_horizontal=False, preview_fps_limit=20.0, parent=None, arm_camera_api=False):
        super().__init__(parent)
        self.stream_url = stream_url
        self.output_size = tuple(output_size)
        self.mirror_horizontal = bool(mirror_horizontal)
        self.preview_fps_limit = max(1.0, float(preview_fps_limit))
        self.arm_camera_api = arm_camera_api
        self._stop_event = threading.Event()
        self.cap = None



    def stop(self):
        self._stop_event.set()
        self.requestInterruption()

    def _stop_requested(self) -> bool:
        return self._stop_event.is_set() or self.isInterruptionRequested()

    def _release_sources_in_worker(self):
        cap, self.cap = (self.cap, None)
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

    def _open_source(self, cv2):
        self._release_sources_in_worker()
        if self._stop_requested():
            return False
        source = int(self.stream_url) if str(self.stream_url).isdigit() else self.stream_url
        if self.arm_camera_api:
            from camera_connection import prepare_arm_stream
            source = prepare_arm_stream(self.stream_url,self._stop_event,self.status_ready.emit)
        if self._stop_requested():return False
        self.cap = cv2.VideoCapture(source) if isinstance(source, int) else cv2.VideoCapture(source, cv2.CAP_FFMPEG, [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000])
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        return self.cap.isOpened()

    def _read_frame(self, cv2):
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        return frame if ok else None





    def _emit_processed_frame(self, cv2, frame, previous_time, fps_ema, last_emit_time):
        height, width = frame.shape[:2]
        scale = min(self.output_size[0] / width, self.output_size[1] / height)
        frame = cv2.resize(frame, (max(1, round(width*scale)), max(1, round(height*scale))), interpolation=cv2.INTER_AREA)
        if self.mirror_horizontal:
            frame = cv2.flip(frame, 1)
        now = time.time()
        dt = now - previous_time
        fps = 1.0 / dt if dt > 0 else 0.0
        fps_ema = fps if fps_ema <= 0 else 0.85 * fps_ema + 0.15 * fps
        if now - last_emit_time >= 1.0 / self.preview_fps_limit:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, channels = rgb.shape
            self.frame_ready.emit(QImage(rgb.data, w, h, channels*w, QImage.Format_RGB888).copy())
            self.fps_ready.emit(fps_ema)
            last_emit_time = now
        return now, fps_ema, last_emit_time

    def run(self):
        try:
            import cv2
        except Exception as exc:
            self.status_ready.emit(f'OpenCV no está instalado: {exc}')
            return
        self._stop_event.clear()
        final_status = 'Cámara detenida'
        try:
            if self._stop_requested():
                return
            self.status_ready.emit(f'Abriendo cámara local {self.stream_url}')
            try:
                opened = self._open_source(cv2)
            except Exception as exc:
                final_status = f'No se pudo abrir la cámara local: {exc}'
                return
            if self._stop_requested():
                return
            if not opened:
                final_status = 'No se pudo abrir la cámara local. Revisa el índice de OpenCV.'
                return
            self.status_ready.emit('Cámara conectada · OpenCV')
            previous_time = time.time()
            last_emit_time = 0.0
            fps_ema = 0.0
            lost_frames = 0
            while not self._stop_requested():
                frame = self._read_frame(cv2)
                if self._stop_requested():
                    break
                if frame is None:
                    lost_frames += 1
                    if lost_frames == 1:
                        self.status_ready.emit('Sin frame; esperando cámara...')
                    if lost_frames >= 8:
                        self.status_ready.emit('Reabriendo cámara local...')
                        if self._stop_event.wait(0.25):
                            break
                        try:
                            if not self._open_source(cv2) and self._stop_event.wait(0.5):
                                break
                        except Exception as exc:
                            self.status_ready.emit(f'Reintento fallido: {exc}')
                            if self._stop_event.wait(0.75):
                                break
                        lost_frames = 0
                    elif self._stop_event.wait(0.05):
                        break
                    continue
                lost_frames = 0
                previous_time, fps_ema, last_emit_time = self._emit_processed_frame(cv2, frame, previous_time, fps_ema, last_emit_time)
        except Exception as exc:
            final_status = f'Error de cámara: {exc}'
        finally:
            self._release_sources_in_worker()
            self.status_ready.emit(final_status)

class MainWindow(QMainWindow):
    BG_MAIN = '#18181B'
    BG_SEC = '#27272A'
    ACCENT_BLUE = '#3B82F6'
    TEXT_LIGHT = '#F8FAFC'
    TEXT_MUTED = '#94A3B8'
    BTN_IDLE_BG = '#334155'
    STATUS_ERR = '#EF4444'
    STATUS_OK = '#10B981'
    ROBOT_BASE_COLLISION_RADIUS = 45.0
    ROBOT_LINK_COLLISION_RADIUS = 10.0
    ROBOT_JOINT_COLLISION_RADIUS = 18.0
    ROBOT_COLLISION_MARGIN = 8.0
    TOOL_COLLISION_BUFFER = 25.0
    PATH_COLLISION_SAMPLE_MM = 15.0
    OBSTACLE_SEGMENT_SAMPLE_MM = 4.0
    PATH_DETOUR_EXTRA_CLEARANCE_MM = 20.0
    PATH_BACKTRACK_DISTANCES_MM = (60.0, 110.0)
    PATH_MAX_HORIZONTAL_DETOUR_MM = 240.0
    PATH_MAX_CANDIDATES_PER_SEGMENT = 10
    MANUAL_GRIPPER_MAX_DEG = 75

    def __init__(self):
        super().__init__()
        self.setWindowTitle('RobotFKIKSim V6 · Real Vision')
        self.setMinimumSize(1120, 720)
        self.resize(1600, 900)
        self.ARDUINO_PORT = 'COM14'
        self.PENDANT_PORT = 'COM15'
        self.PENDANT_IP = '192.168.4.1'
        self.PENDANT_TCP_PORT = 4210
        self.vision_config_path = Path(__file__).resolve().with_name('RobotFKIKSim_vision_config.json')
        self.robot = RobotKinematics()
        self.current_physical_angles = self.robot.startup_angles.copy()
        self.current_xyz = [0.0, 0.0, 0.0]
        self.slider_target_fk = self.robot.startup_angles.copy()
        self.slider_target_xyz = np.array([0.0, 0.0, 0.0])
        self.pendant_target_xyz = np.array([0.0, 0.0, 0.0])
        self.max_speed_mms = 100.0
        self.max_speed_degs = 90.0
        self._syncing = False
        self.vision_state = 'IDLE'
        self.vision_motion_queue = []
        self.vision_actions = []
        self.recovery_active = False
        self.recovery_stage = 'IDLE'
        self._last_vision_draw_time = 0.0
        self.path_active = False
        self.planned_path = []
        self.obstacles = []
        self._force_hardware_send = False
        self.last_robot_send_time = 0.0
        self.last_pendant_send_time = 0.0
        self.arduino_serial = None
        self.pendant_serial = PendantLink()
        self._pendant_should_reconnect = False
        self._last_reconnect_attempt = 0.0
        self._reconnect_interval_s = 3.0
        self._pendant_heartbeat_interval_s = 5.0
        self._last_pendant_ping = 0.0
        self.vision_thread = None
        self._camera_stop_pending = False
        self._camera_clear_preview_on_stop = False
        self._camera_restart_requested = False
        self._closing_after_camera_stop = False
        self._build_ui()
        self._load_vision_config(silent=True, create_if_missing=True)
        self.current_xyz, _ = self.robot.compute_fk(self.current_physical_angles)
        self.slider_target_xyz = self.current_xyz.copy()
        self.pendant_target_xyz = self.current_xyz.copy()
        self._toggle_mode()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._control_loop_tick)
        self.timer.start(50)

    def _build_ui(self):
        root = QWidget()
        root.setObjectName('root')
        self.setCentralWidget(root)
        root.setStyleSheet(f'\n QWidget#root {{ background: {self.BG_MAIN}; }}\n QLabel, QCheckBox, QGroupBox {{ color: {self.TEXT_LIGHT}; }}\n QGroupBox {{ font-weight: bold; border: 1px solid #3F3F46; border-radius: 7px;\n margin-top: 9px; padding-top: 10px; }}\n QGroupBox::title {{ subcontrol-origin: margin; left: 9px; padding: 0 4px; }}\n QLineEdit, QComboBox {{ background: {self.BG_MAIN}; color: {self.TEXT_LIGHT};\n border: 1px solid #52525B; border-radius: 5px; padding: 6px; }}\n QPushButton {{ background: {self.BTN_IDLE_BG}; color: {self.TEXT_LIGHT};\n border: 1px solid #475569; border-radius: 6px; padding: 7px 10px;\n font-weight: bold; }}\n QPushButton:hover {{ background: #475569; }}\n QPushButton:disabled {{ color: #64748B; background: #27272A; }}\n QTabWidget::pane {{ border: 1px solid #3F3F46; background: {self.BG_SEC}; }}\n QTabBar::tab {{ background: #202024; color: {self.TEXT_MUTED}; padding: 8px 12px; }}\n QTabBar::tab:selected {{ background: {self.ACCENT_BLUE}; color: white; }}\n QScrollArea {{ border: none; background: transparent; }}\n')
        page = QVBoxLayout(root)
        page.setContentsMargins(14, 12, 14, 12)
        header = QHBoxLayout()
        title = QLabel('Brazo · RobotArmCodeV7')
        title.setFont(QFont('Arial', 19, QFont.Weight.Bold))
        header.addWidget(title)
        header.addStretch()
        header.addWidget(QLabel('FK / IK · Trayectorias · Visión · conexión SAR compartida'))
        page.addLayout(header)
        content = QSplitter(Qt.Orientation.Horizontal)
        content.setChildrenCollapsible(False)
        page.addWidget(content, stretch=1)
        self.tabs = QTabWidget()
        self.tabs.setMinimumWidth(500)
        self.tabs.setMaximumWidth(760)
        content.addWidget(self.tabs)

        def make(kind, attr, content='', state=None):
            if kind == 'tab':
                widget = QScrollArea()
                widget.setWidgetResizable(True)
                widget.setWidget(attr)
                self.tabs.addTab(widget, str(content))
                return widget
            if kind == 'edit':
                widget = QLineEdit(str(content))
                if state:
                    widget.setPlaceholderText(str(state))
            elif kind == 'combo':
                widget = QComboBox()
                widget.addItems(list(content))
                if isinstance(state, int):
                    widget.setCurrentIndex(state)
                elif state is not None:
                    widget.setCurrentText(str(state))
            else:
                widget = QCheckBox(str(content))
                widget.setChecked(bool(state))
            setattr(self, attr, widget)
            return widget
        manual = QWidget()
        manual_layout = QVBoxLayout(manual)
        kinematics = QGroupBox('Control cinemático')
        kin_layout = QVBoxLayout(kinematics)
        self.mode_selector = make('combo', 'mode_selector', ['Forward Kinematics (Physical Angles)', 'Inverse Kinematics (Position)'])
        self.mode_selector.currentIndexChanged.connect(self._toggle_mode)
        kin_layout.addWidget(self.mode_selector)
        self.input_stack = QStackedWidget()
        self.fk_widget, self.fk_sliders, self.fk_labels = (QWidget(), [], [])
        fk_layout = QVBoxLayout(self.fk_widget)
        for index, name in enumerate(('Base (Z)', 'Hombro', 'Codo', 'Muñeca Elev.')):
            label = QLabel(f'{name}: {self.current_physical_angles[index]}°')
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, 1800)
            slider.setValue(int(self.current_physical_angles[index] * 10))
            slider.valueChanged.connect(self._on_slider_changed)
            self.fk_labels.append(label)
            self.fk_sliders.append(slider)
            fk_layout.addWidget(label)
            fk_layout.addWidget(slider)
        self.ik_widget, self.ik_sliders, self.ik_labels = (QWidget(), [], [])
        ik_layout = QVBoxLayout(self.ik_widget)
        max_r = int(self.robot.max_reach)
        for name, low, high, value in (('X Target (Desk)', -max_r, max_r, 0), ('Y Target (Desk)', 0, max_r, 0), ('Z Target (Desk)', 0, max_r, 200)):
            label = QLabel(f'{name}: {value} mm')
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(low, high)
            slider.setValue(value)
            slider.valueChanged.connect(self._on_slider_changed)
            self.ik_labels.append(label)
            self.ik_sliders.append(slider)
            ik_layout.addWidget(label)
            ik_layout.addWidget(slider)
        self.input_stack.addWidget(self.fk_widget)
        self.input_stack.addWidget(self.ik_widget)
        kin_layout.addWidget(self.input_stack)
        manual_layout.addWidget(kinematics)
        motion = QGroupBox('Movimiento y efector final')
        motion_layout = QVBoxLayout(motion)
        self.lbl_vel = QLabel(f'Velocidad (Live): {self.max_speed_mms} mm/s')
        self.slider_vel = QSlider(Qt.Orientation.Horizontal)
        self.slider_vel.setRange(10, 300)
        self.slider_vel.setValue(int(self.max_speed_mms))
        self.slider_vel.valueChanged.connect(lambda value: self._update_runtime_option('speed', value))
        self.lbl_pinza = QLabel(f'Pinza (Gripper): {self.MANUAL_GRIPPER_MAX_DEG}°')
        self.slider_pinza = QSlider(Qt.Orientation.Horizontal)
        self.slider_pinza.setRange(0, self.MANUAL_GRIPPER_MAX_DEG)
        self.slider_pinza.setValue(60)
        self.slider_pinza.valueChanged.connect(lambda value: self.lbl_pinza.setText(f'Pinza (Gripper): {value}°'))
        for widget in (self.lbl_vel, self.slider_vel, self.lbl_pinza, self.slider_pinza):
            motion_layout.addWidget(widget)
        manual_layout.addWidget(motion)
        source = QGroupBox('Fuente de control')
        source_layout = QVBoxLayout(source)
        self.chk_live = make('check', 'chk_live', 'Control PC (Live Sliders)')
        self.chk_pendant = make('check', 'chk_pendant', 'Control Remoto (ESP32 Gamepad)')
        self.chk_pendant.toggled.connect(self._on_pendant_toggle)
        self.btn_send_manual = QPushButton('Enviar al Robot (Manual)')
        self.btn_send_manual.clicked.connect(lambda: self._send_to_arduino(live_mode=False))
        source_layout.addWidget(self.chk_live)
        source_layout.addWidget(self.chk_pendant)
        source_layout.addWidget(self.btn_send_manual)
        manual_layout.addWidget(source)
        manual_layout.addStretch()
        make('tab', manual, 'Control')
        path = QWidget()
        path_layout = QVBoxLayout(path)
        path_box = QGroupBox('Gestor de Obstáculos y Trayectorias')
        path_grid = QGridLayout(path_box)
        for row, (label_text, names, defaults) in enumerate((('Punto A (X,Y,Z)', ('in_pa_x', 'in_pa_y', 'in_pa_z'), (-150, 150, 50)), ('Punto B (X,Y,Z)', ('in_pb_x', 'in_pb_y', 'in_pb_z'), (150, 150, 50)))):
            path_grid.addWidget(QLabel(label_text), row, 0)
            for col, (name, value) in enumerate(zip(names, defaults), 1):
                path_grid.addWidget(make('edit', name, value), row, col)
        path_grid.addWidget(QLabel('Tipo de obstáculo'), 2, 0)
        self.combo_obstacle_type = make('combo', 'combo_obstacle_type', ('Esfera', 'Caja / Rectángulo', 'Cilindro', 'Cono'), 'Esfera')
        path_grid.addWidget(self.combo_obstacle_type, 2, 1, 1, 3)
        path_grid.addWidget(QLabel('Centro X/Y/Z [mm]'), 3, 0)
        for col, (name, value) in enumerate((('in_ob_x', 0), ('in_ob_y', 150), ('in_ob_z', 40)), 1):
            path_grid.addWidget(make('edit', name, value), 3, col)
        self.lbl_ob_dim1 = QLabel('Radio [mm]')
        self.lbl_ob_dim2 = QLabel('Altura [mm]')
        self.lbl_ob_dim3 = QLabel('Altura Z [mm]')
        path_grid.addWidget(self.lbl_ob_dim1, 4, 0)
        path_grid.addWidget(make('edit', 'in_ob_dim1', 40), 4, 1)
        path_grid.addWidget(self.lbl_ob_dim2, 4, 2)
        path_grid.addWidget(make('edit', 'in_ob_dim2', 80), 4, 3)
        path_grid.addWidget(self.lbl_ob_dim3, 5, 0)
        path_grid.addWidget(make('edit', 'in_ob_dim3', 80), 5, 1)
        self.combo_obstacle_type.currentTextChanged.connect(self._update_obstacle_fields)
        self._update_obstacle_fields(self.combo_obstacle_type.currentText())
        add_obstacle = QPushButton('Añadir obstáculo')
        add_obstacle.clicked.connect(self._add_obstacle)
        clear_obstacles = QPushButton('Limpiar obstáculos')
        clear_obstacles.clicked.connect(lambda _checked=False: self._reset_path('Objetos limpiados.', clear_obstacles=True))
        self.btn_run_path = QPushButton('Ejecutar trayectoria segura')
        self.btn_run_path.clicked.connect(self._execute_path_planning)
        self.btn_stop_path = QPushButton('Detener')
        self.btn_stop_path.clicked.connect(lambda _checked=False: self._reset_path('Trayectoria detenida.'))
        path_grid.addWidget(add_obstacle, 6, 0, 1, 2)
        path_grid.addWidget(clear_obstacles, 6, 2, 1, 2)
        path_grid.addWidget(self.btn_run_path, 7, 0, 1, 3)
        path_grid.addWidget(self.btn_stop_path, 7, 3)
        self.chk_path_physical = make('check', 'chk_path_physical', 'Reproducir también en el robot físico')
        path_grid.addWidget(self.chk_path_physical, 8, 0, 1, 4)
        self.chk_path_backward = make('check', 'chk_path_backward', 'Permitir desvíos laterales y hacia atrás', True)
        path_grid.addWidget(self.chk_path_backward, 9, 0, 1, 4)
        path_layout.addWidget(path_box)
        path_layout.addStretch()
        make('tab', path, 'Trayectorias')
        self.path_tab_index = self.tabs.count() - 1
        vision = QWidget()
        vision_layout = QVBoxLayout(vision)
        note = QLabel('Camera connection test only. No camera-driven movement.\nUse Cameras IP for ArUco IDs and calibrated camera-frame measurements.')
        note.setWordWrap(True)
        vision_layout.addWidget(note)
        camera = QGroupBox('Arm camera preview')
        camera_grid = QGridLayout(camera)
        camera_grid.addWidget(QLabel('Camera IP / URL'), 0, 0)
        self.camera_source = QLineEdit('')
        self.camera_source.setPlaceholderText('ESP32-S3 IP or base URL')
        camera_grid.addWidget(self.camera_source, 0, 1)
        self.combo_camera_resolution = make('combo', 'combo_camera_resolution', ('320x240', '640x480', '320x320'), '320x240')
        self.combo_camera_resolution.currentTextChanged.connect(lambda value: self._update_runtime_option('resolution', value))
        camera_grid.addWidget(QLabel('Preview bounds'), 1, 0)
        camera_grid.addWidget(self.combo_camera_resolution, 1, 1)
        self.chk_camera_mirror = make('check', 'chk_camera_mirror', 'Mirror preview')
        self.chk_camera_mirror.toggled.connect(lambda value: self._update_runtime_option('mirror', value))
        camera_grid.addWidget(self.chk_camera_mirror, 2, 0, 1, 2)
        self.btn_camera_connect = QPushButton('Connect camera')
        self.btn_camera_connect.clicked.connect(self._start_camera_stream)
        self.btn_camera_stop = QPushButton('Stop camera')
        self.btn_camera_stop.setEnabled(False)
        self.btn_camera_stop.clicked.connect(lambda: self._stop_camera_stream())
        camera_grid.addWidget(self.btn_camera_connect, 3, 0)
        camera_grid.addWidget(self.btn_camera_stop, 3, 1)
        self.camera_preview = QLabel('No camera image')
        self.camera_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_preview.setFixedSize(320, 320)
        camera_grid.addWidget(self.camera_preview, 4, 0, 1, 2, Qt.AlignmentFlag.AlignHCenter)
        self.lbl_camera_status = QLabel('Camera disconnected')
        self.lbl_camera_status.setWordWrap(True)
        self.lbl_camera_fps = QLabel('FPS: 0.0')
        camera_grid.addWidget(self.lbl_camera_status, 5, 0)
        camera_grid.addWidget(self.lbl_camera_fps, 5, 1)
        vision_layout.addWidget(camera)
        vision_layout.addStretch()
        make('tab', vision, 'Camera')
        self.vision_tab_index = self.tabs.count() - 1
        connections = QWidget()
        connections_layout = QVBoxLayout(connections)
        robot_box = QGroupBox('Robot por USB Serial')
        robot_grid = QGridLayout(robot_box)
        robot_grid.addWidget(QLabel('Puerto'), 0, 0)
        robot_grid.addWidget(make('edit', 'input_arduino_port', self.ARDUINO_PORT), 0, 1)
        connect_robot = QPushButton('Conectar robot')
        connect_robot.clicked.connect(self._connect_arduino)
        disconnect_robot = QPushButton('Desconectar')
        disconnect_robot.clicked.connect(lambda _checked=False: self._disconnect_hardware('robot'))
        robot_grid.addWidget(connect_robot, 1, 0)
        robot_grid.addWidget(disconnect_robot, 1, 1)
        self.lbl_arduino_status = QLabel('Robot: desconectado')
        robot_grid.addWidget(self.lbl_arduino_status, 2, 0, 1, 2)
        connections_layout.addWidget(robot_box)
        pendant_box = QGroupBox('Gamepad / Pendant ESP32')
        pendant_grid = QGridLayout(pendant_box)
        self.pendant_mode = make('combo', 'pendant_mode', ('USB Serial', 'WiFi TCP'))
        self.pendant_mode.currentIndexChanged.connect(lambda _index: self._update_runtime_option('pendant_fields', None))
        pendant_grid.addWidget(QLabel('Transporte'), 0, 0)
        pendant_grid.addWidget(self.pendant_mode, 0, 1)
        for row, (label_text, attr, value) in enumerate((('Puerto USB', 'input_pendant_port', self.PENDANT_PORT), ('IP ESP32', 'input_pendant_ip', self.PENDANT_IP), ('Puerto TCP', 'input_pendant_tcp_port', self.PENDANT_TCP_PORT)), 1):
            pendant_grid.addWidget(QLabel(label_text), row, 0)
            pendant_grid.addWidget(make('edit', attr, value), row, 1)
        self.chk_auto_reconnect = make('check', 'chk_auto_reconnect', 'Reconexión automática')
        pendant_grid.addWidget(self.chk_auto_reconnect, 4, 0, 1, 2)
        connect_pendant = QPushButton('Conectar pendant')
        connect_pendant.clicked.connect(self._connect_pendant)
        disconnect_pendant = QPushButton('Desconectar')
        disconnect_pendant.clicked.connect(lambda _checked=False: self._disconnect_hardware('pendant'))
        pendant_grid.addWidget(connect_pendant, 5, 0)
        pendant_grid.addWidget(disconnect_pendant, 5, 1)
        self.lbl_pendant_status = QLabel('Pendant: desconectado')
        self.lbl_pendant_status.setWordWrap(True)
        pendant_grid.addWidget(self.lbl_pendant_status, 6, 0, 1, 2)
        connections_layout.addWidget(pendant_box)
        connections_layout.addStretch()
        make('tab', connections, 'Conexiones')
        self._update_runtime_option('pendant_fields', None)
        self.tabs.currentChanged.connect(lambda _index: self._redraw_robot())
        right_panel = QWidget()
        right = QVBoxLayout(right_panel)
        view_hint = QLabel('Vista 3D · arrastra para rotar · botón central para desplazar · rueda para zoom')
        view_hint.setStyleSheet(f'color: {self.TEXT_MUTED};')
        right.addWidget(view_hint)
        self.fig = plt.figure(figsize=(6, 6))
        self.fig.patch.set_facecolor(self.BG_MAIN)
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.view_init(elev=30.0, azim=135.0)
        self.ax.mouse_init(rotate_btn=1, pan_btn=2, zoom_btn=3)
        right.addWidget(self.canvas, stretch=1)
        status = QHBoxLayout()
        self.lbl_angles = self._status_label('Ángulos simulados', '[0.0, 0.0, 0.0, 0.0] °')
        self.lbl_pos = self._status_label('TCP simulado (sin encoder)', '[X: 0.0, Y: 0.0, Z: 0.0] mm')
        status.addWidget(self.lbl_angles)
        status.addWidget(self.lbl_pos)
        right.addLayout(status)
        content.addWidget(right_panel)
        content.setStretchFactor(1, 1)
        content.setSizes((600, 1000))
        self.error_bar = QLabel('')
        self.error_bar.setWordWrap(True)
        self.error_bar.hide()
        page.addWidget(self.error_bar)

    def _status_label(self, title: str, default_val: str) -> QWidget:
        row = QWidget()
        layout = QVBoxLayout(row)
        lbl_title = QLabel(title)
        lbl_title.setStyleSheet(f'color: {self.TEXT_MUTED}; font-weight: bold;')
        lbl_val = QLabel(default_val)
        lbl_val.setStyleSheet(f'color: {self.ACCENT_BLUE}; font-weight: bold; font-size: 14px;')
        layout.addWidget(lbl_title)
        layout.addWidget(lbl_val)
        row.setProperty('value_label', lbl_val)
        return row

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'canvas'):
            QTimer.singleShot(0, self.canvas.draw_idle)

    def closeEvent(self, event):
        try:
            self._save_vision_config(silent=True)
        except Exception:
            pass
        worker = self.vision_thread
        if worker is not None and worker.isRunning():
            self._closing_after_camera_stop = True
            self._stop_camera_stream(clear_preview=False)
            event.ignore()
            return
        try:
            if self.arduino_serial is not None:
                self.arduino_serial.close()
        except Exception:
            pass
        try:
            self.pendant_serial.close()
        except Exception:
            pass
        event.accept()

    def _show_msg(self, msg, is_error=False):
        color = self.STATUS_ERR if is_error else self.STATUS_OK
        self.error_bar.setStyleSheet(f'color: #FFF; background-color: {color}; border-radius: 6px; padding: 10px; font-weight: bold;')
        self.error_bar.setText(msg)
        self.error_bar.show()

    def _toggle_mode(self):
        self._syncing = True
        mode = self.mode_selector.currentIndex()
        if mode == 0:
            for i in range(4):
                self.fk_sliders[i].setValue(int(self.current_physical_angles[i] * 10))
        else:
            self.ik_sliders[0].setValue(int(self.current_xyz[0]))
            self.ik_sliders[1].setValue(int(max(0, self.current_xyz[1])))
            self.ik_sliders[2].setValue(int(max(0, self.current_xyz[2])))
        self.input_stack.setCurrentIndex(mode)
        self._syncing = False
        self._on_slider_changed()

    def _on_slider_changed(self):
        if self._syncing:
            return
        mode = self.mode_selector.currentIndex()
        if mode == 0:
            self.slider_target_fk = np.array([s.value() / 10.0 for s in self.fk_sliders])
            joint_names = ['Base (Z)', 'Hombro', 'Codo', 'Muñeca Elev.']
            for i, val in enumerate(self.slider_target_fk):
                self.fk_labels[i].setText(f'{joint_names[i]}: {val:.1f}°')
        else:
            self.slider_target_xyz = np.array([s.value() for s in self.ik_sliders])
            names = ['X Target (Desk)', 'Y Target (Desk)', 'Z Target (Desk)']
            for i, val in enumerate(self.slider_target_xyz):
                self.ik_labels[i].setText(f'{names[i]}: {val} mm')

    def _enforce_safety_zone(self, target_xyz):
        x, y, z = target_xyz
        r = math.hypot(x, y)
        if r < 70.0:
            if r < 0.1:
                x, y = (70.0, 0.0)
            else:
                x, y = (x / r * 70.0, y / r * 70.0)
        return np.array([x, y, z])

    def _update_runtime_option(self, option, value):
        worker = self.vision_thread
        if option == 'pendant_fields':
            wifi_mode = self.pendant_mode.currentText() == 'WiFi TCP'
            self.input_pendant_port.setEnabled(not wifi_mode)
            self.input_pendant_ip.setEnabled(wifi_mode)
            self.input_pendant_tcp_port.setEnabled(wifi_mode)
        elif option == 'speed':
            self.max_speed_mms = float(value)
            self.max_speed_degs = float(value) * 0.9
            self.lbl_vel.setText(f'Velocidad (Live): {value} mm/s')
        elif option == 'status':
            text = str(value)
            self.lbl_camera_status.setText(f'Cámara: {text}')
            if 'no se pudo' in text.lower() or 'opencv no' in text.lower():
                self.btn_camera_connect.setEnabled(True)
        elif option == 'fps':
            self.lbl_camera_fps.setText(f'FPS: {float(value):.1f}')
        elif option == 'resolution' and worker is not None:
            worker.output_size = parse_camera_resolution(str(value))
        elif option == 'mirror' and worker is not None:
            worker.mirror_horizontal = bool(value)

    def _on_pendant_toggle(self, checked):
        if checked:
            self.chk_live.setChecked(False)
            self.mode_selector.setCurrentIndex(1)
            self.input_stack.setEnabled(False)
            self.pendant_target_xyz = self.current_xyz.copy()
            if self.pendant_serial:
                self._pendant_write_line('CTRL:1')
        else:
            self.input_stack.setEnabled(True)
            self._toggle_mode()
            if self.pendant_serial:
                self._pendant_write_line('CTRL:0')

    def _connect_arduino(self):
        port = self.input_arduino_port.text().strip() or self.ARDUINO_PORT
        try:
            if self.arduino_serial is not None:
                try:
                    self.arduino_serial.close()
                except Exception:
                    pass
            self.arduino_serial = serial.Serial(port, 115200, timeout=0)
            self.ARDUINO_PORT = port
            self.lbl_arduino_status.setText(f'Robot: conectado por USB {port}')
            self.lbl_arduino_status.setStyleSheet(f'color: {self.STATUS_OK}; font-weight: bold;')
            self._show_msg(f'✅ Conectado al Robot en {port}')
        except Exception as e:
            self.arduino_serial = None
            self.lbl_arduino_status.setText('Robot: desconectado')
            self.lbl_arduino_status.setStyleSheet(f'color: {self.STATUS_ERR};')
            self._show_msg(f'Error Arduino ({port}): {e}', is_error=True)

    def _connect_pendant(self, automatic=False):
        self._last_reconnect_attempt = time.time()
        try:
            mode = self.pendant_mode.currentText()
            if self.pendant_serial and (not automatic):
                if mode == 'USB Serial':
                    requested_name = f'USB {self.input_pendant_port.text().strip() or self.PENDANT_PORT}'
                else:
                    requested_ip = self.input_pendant_ip.text().strip() or self.PENDANT_IP
                    requested_port = int(self.input_pendant_tcp_port.text().strip() or self.PENDANT_TCP_PORT)
                    requested_name = f'WiFi TCP {requested_ip}:{requested_port}'
                if self.pendant_serial.name == requested_name:
                    self._set_pendant_status(f'Pendant: conectado por {self.pendant_serial.name}', connected=True)
                    self._show_msg(f'✅ El Pendant ya está conectado por {self.pendant_serial.name}')
                    return
            if mode == 'USB Serial':
                port = self.input_pendant_port.text().strip() or self.PENDANT_PORT
                self.pendant_serial.connect_serial(port, 115200)
                self.PENDANT_PORT = port
            else:
                ip = self.input_pendant_ip.text().strip() or self.PENDANT_IP
                port_text = self.input_pendant_tcp_port.text().strip()
                port = int(port_text or self.PENDANT_TCP_PORT)
                if not 1 <= port <= 65535:
                    raise ValueError('el puerto TCP debe estar entre 1 y 65535')
                connect_timeout = 1.5 if automatic else 3.0
                self.pendant_serial.connect_tcp(ip, port, timeout=connect_timeout)
                self.PENDANT_IP = ip
                self.PENDANT_TCP_PORT = port
            self._pendant_should_reconnect = self.chk_auto_reconnect.isChecked()
            self._last_pendant_ping = 0.0
            self._set_pendant_status(f'Pendant: conectado por {self.pendant_serial.name}', connected=True)
            self._pendant_write_line('CTRL:1' if self.chk_pendant.isChecked() else 'CTRL:0')
            self._pendant_write_line('PING')
            if automatic:
                self._show_msg(f'✅ Pendant reconectado por {self.pendant_serial.name}')
            else:
                self._show_msg(f'✅ Conectado al Pendant por {self.pendant_serial.name}')
        except Exception as e:
            self.pendant_serial.close()
            self._pendant_should_reconnect = self.chk_auto_reconnect.isChecked()
            if self._pendant_should_reconnect:
                self._set_pendant_status(f'Pendant: sin conexión; reintentando cada {self._reconnect_interval_s:.0f} s', reconnecting=True)
            else:
                self._set_pendant_status('Pendant: desconectado')
            if not automatic:
                self._show_msg(f'Error Gamepad/Pendant: {e}', is_error=True)

    def _disconnect_hardware(self, device):
        if device == 'robot':
            try:
                if self.arduino_serial is not None:
                    self.arduino_serial.close()
            except Exception:
                pass
            self.arduino_serial = None
            self.lbl_arduino_status.setText('Robot: desconectado')
            self.lbl_arduino_status.setStyleSheet(f'color: {self.TEXT_MUTED};')
            self._show_msg('Robot desconectado.')
        else:
            self._pendant_should_reconnect = False
            self.pendant_serial.close()
            self._set_pendant_status('Pendant: desconectado')
            self._show_msg('Pendant desconectado.')

    def _set_pendant_status(self, text, connected=False, reconnecting=False):
        if connected:
            color = self.STATUS_OK
        elif reconnecting:
            color = '#F59E0B'
        else:
            color = self.TEXT_MUTED
        self.lbl_pendant_status.setText(text)
        self.lbl_pendant_status.setStyleSheet(f'color: {color}; font-weight: bold;')

    def _maintain_pendant_connection(self, current_time):
        if self.pendant_serial:
            if self.pendant_serial.mode == 'tcp':
                if not self.pendant_serial.tcp_health_ok():
                    self._set_pendant_status(f'Pendant: enlace perdido; reintentando cada {self._reconnect_interval_s:.0f} s', reconnecting=True)
                elif current_time - self._last_pendant_ping >= self._pendant_heartbeat_interval_s:
                    self._last_pendant_ping = current_time
                    self._pendant_write_line('PING')
            return
        if not self._pendant_should_reconnect:
            return
        if not self.chk_auto_reconnect.isChecked():
            self._pendant_should_reconnect = False
            return
        if current_time - self._last_reconnect_attempt < self._reconnect_interval_s:
            return
        self._set_pendant_status(f'Pendant: reconectando por {self.pendant_mode.currentText()}...', reconnecting=True)
        self._connect_pendant(automatic=True)

    def _pendant_write_line(self, line: str):
        if not self.pendant_serial:
            return False
        if not line.endswith('\n'):
            line += '\n'
        sent = self.pendant_serial.write(line.encode('utf-8'))
        if sent <= 0 and (not self.pendant_serial):
            if self._pendant_should_reconnect:
                self._set_pendant_status(f'Pendant: enlace perdido; reintentando cada {self._reconnect_interval_s:.0f} s', reconnecting=True)
            else:
                self._set_pendant_status('Pendant: desconectado')
            return False
        return True

    def _read_pendant(self, allow_motion=True):
        manual_override = False
        if not self.pendant_serial or not self.pendant_serial.in_waiting:
            return manual_override
        try:
            while self.pendant_serial.in_waiting:
                raw = self.pendant_serial.readline()
                if not raw:
                    break
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                if line.startswith('REQ:STATE'):
                    x, y, z = self.current_xyz
                    grip = self.slider_pinza.value()
                    self._pendant_write_line(f'STATE:{x:.1f},{y:.1f},{z:.1f},0.0,0.0,{grip:.1f}')
                    continue
                if line.startswith('HELLO:') or line.startswith('ACK:') or line == 'PONG' or line.startswith('NET:'):
                    continue
                if line.startswith('PEND:'):
                    if not allow_motion:
                        continue
                    parts = line.split(':', 1)[1].split(',')
                    if len(parts) < 5:
                        continue
                    joyMode = int(parts[0])
                    encMode = int(parts[1])
                    stepX = int(parts[2])
                    stepY = int(parts[3])
                    encDelta = int(parts[4])
                    if joyMode == 0:
                        tx = self.pendant_target_xyz[0]
                        ty = self.pendant_target_xyz[1]
                        r = math.hypot(tx, ty)
                        theta = math.atan2(ty, tx)
                        r += stepY
                        r = max(70.0, r)
                        theta -= stepX * 0.02
                        self.pendant_target_xyz[0] = r * math.cos(theta)
                        self.pendant_target_xyz[1] = r * math.sin(theta)
                    elif joyMode == 1:
                        deadzone = 0
                        sensitivity = 0.1
                        if abs(stepY) > deadzone:
                            new_wrist = np.clip(self.current_physical_angles[3] - stepY * sensitivity, 0, 180)
                            if new_wrist != self.current_physical_angles[3]:
                                self.current_physical_angles[3] = new_wrist
                                manual_override = True
                                final_pos, T_matrices = self.robot.compute_fk(self.current_physical_angles)
                                self.current_xyz = final_pos
                                self.pendant_target_xyz = final_pos.copy()
                                self._draw_robot(T_matrices)
                                self._update_text_results(self.current_physical_angles, self.current_xyz)
                                self._syncing = True
                                self.ik_sliders[0].setValue(int(self.current_xyz[0]))
                                self.ik_sliders[1].setValue(int(self.current_xyz[1]))
                                self.ik_sliders[2].setValue(int(self.current_xyz[2]))
                                self.fk_sliders[3].setValue(int(self.current_physical_angles[3] * 10))
                                self._syncing = False
                        if abs(stepX) > deadzone:
                            new_grip = np.clip(self.slider_pinza.value() - stepX * 3, 0, 75)
                            if int(new_grip) != self.slider_pinza.value():
                                self.slider_pinza.setValue(int(new_grip))
                                manual_override = True
                    if encMode == 0:
                        self.pendant_target_xyz[2] += encDelta * 5.0
                    if encMode == 1:
                        if encDelta != 0:
                            new_grip = np.clip(self.slider_pinza.value() + encDelta * 5, 0, 75)
                            self.slider_pinza.setValue(int(new_grip))
                            manual_override = True
        except Exception as e:
            print(f'Pendant read error: {e}')
            if not self.pendant_serial and self._pendant_should_reconnect:
                self._set_pendant_status(f'Pendant: enlace perdido; reintentando cada {self._reconnect_interval_s:.0f} s', reconnecting=True)
        return manual_override

    def _send_to_arduino(self, live_mode=False):
        if not self.arduino_serial:
            if not live_mode:
                self._show_msg('Robot hardware no conectado.', is_error=True)
            return
        b = int(np.round(self.current_physical_angles[0]))
        h = int(np.round(self.current_physical_angles[1]))
        c = int(np.round(self.current_physical_angles[2]))
        me = int(np.round(self.current_physical_angles[3]))
        p = int(self.slider_pinza.value())
        t_ms = 150 if live_mode else 2000
        comando = f'{b},{h},{c},90,{me},{p},{t_ms}\n'
        try:
            self.arduino_serial.write(comando.encode('utf-8'))
            if not live_mode:
                self._show_msg(f'✅ Comando enviado: {comando.strip()}')
        except Exception as e:
            if not live_mode:
                self._show_msg(f'Error de envío Arduino: {e}', is_error=True)

    def _update_obstacle_fields(self, obstacle_type=None):
        obstacle_type = obstacle_type or self.combo_obstacle_type.currentText()
        if obstacle_type == 'Esfera':
            labels = ('Radio [mm]', '', '')
            visible = (True, False, False)
        elif obstacle_type == 'Caja / Rectángulo':
            labels = ('Ancho X [mm]', 'Profundidad Y [mm]', 'Altura Z [mm]')
            visible = (True, True, True)
        else:
            labels = ('Radio [mm]', 'Altura [mm]', '')
            visible = (True, True, False)
        label_widgets = (self.lbl_ob_dim1, self.lbl_ob_dim2, self.lbl_ob_dim3)
        input_widgets = (self.in_ob_dim1, self.in_ob_dim2, self.in_ob_dim3)
        for label_widget, input_widget, label_text, is_visible in zip(label_widgets, input_widgets, labels, visible):
            label_widget.setText(label_text)
            label_widget.setVisible(is_visible)
            input_widget.setVisible(is_visible)

    def _add_obstacle(self):
        try:
            x = float(self.in_ob_x.text())
            y = float(self.in_ob_y.text())
            z = float(self.in_ob_z.text())
            dim1 = float(self.in_ob_dim1.text())
            dim2 = float(self.in_ob_dim2.text()) if self.in_ob_dim2.isVisible() else 0.0
            dim3 = float(self.in_ob_dim3.text()) if self.in_ob_dim3.isVisible() else 0.0
        except ValueError:
            self._show_msg('Error: ingresa números válidos para el objeto.', is_error=True)
            return
        obstacle_type_text = self.combo_obstacle_type.currentText()
        type_map = {'Esfera': 'sphere', 'Caja / Rectángulo': 'box', 'Cilindro': 'cylinder', 'Cono': 'cone'}
        obstacle_type = type_map[obstacle_type_text]
        values = np.array([x, y, z, dim1, dim2, dim3], dtype=float)
        if not np.all(np.isfinite(values)):
            self._show_msg('Error: la posición y las dimensiones deben ser valores finitos.', is_error=True)
            return
        if obstacle_type == 'sphere':
            if dim1 <= 0.0:
                self._show_msg('Error: el radio debe ser mayor que cero.', is_error=True)
                return
            obstacle = {'type': 'sphere', 'pos': np.array([x, y, z], dtype=float), 'radius': float(dim1)}
        elif obstacle_type == 'box':
            if dim1 <= 0.0 or dim2 <= 0.0 or dim3 <= 0.0:
                self._show_msg('Error: ancho, profundidad y altura deben ser mayores que cero.', is_error=True)
                return
            obstacle = {'type': 'box', 'pos': np.array([x, y, z], dtype=float), 'size': np.array([dim1, dim2, dim3], dtype=float)}
        else:
            if dim1 <= 0.0 or dim2 <= 0.0:
                self._show_msg('Error: radio y altura deben ser mayores que cero.', is_error=True)
                return
            obstacle = {'type': obstacle_type, 'pos': np.array([x, y, z], dtype=float), 'radius': float(dim1), 'height': float(dim2)}
        self.obstacles.append(obstacle)
        self._redraw_robot()
        collision_reason = self._obstacle_robot_collision_reason(obstacle, self.current_physical_angles)
        if collision_reason:
            self._reset_path(redraw=False)
            self._show_msg(f'Objeto o trayectoria imposible de realizar debido a riesgo de choque: {collision_reason}. Retira o reubica el objeto antes de mover el robot.', is_error=True)
            return
        if self.path_active and self.planned_path:
            safe, reason = self._validate_planned_path_collision(np.array(self.current_xyz, dtype=float), [np.array(point, dtype=float) for point in self.planned_path])
            if not safe:
                self._reset_path(redraw=False)
                self._show_msg(f'Objeto o trayectoria imposible de realizar debido a riesgo de choque: {reason}. La trayectoria activa fue detenida.', is_error=True)
                self._redraw_robot()
                return
        self._show_msg(f'{self._obstacle_description(obstacle)} añadido en ({x:.1f}, {y:.1f}, {z:.1f}) mm.')

    def _reset_path(self, message=None, clear_obstacles=False, redraw=True):
        self.path_active = False
        self.planned_path.clear()
        if clear_obstacles:
            self.obstacles.clear()
        if message:
            self._show_msg(message)
        if redraw:
            self._redraw_robot()

    @staticmethod
    def _obstacle_description(obstacle):
        obstacle_type = obstacle.get('type', 'sphere')
        if obstacle_type == 'sphere':
            return f"Esfera R={obstacle['radius']:.1f} mm"
        if obstacle_type == 'box':
            sx, sy, sz = obstacle['size']
            return f'Caja {sx:.1f}×{sy:.1f}×{sz:.1f} mm'
        if obstacle_type == 'cylinder':
            return f"Cilindro R={obstacle['radius']:.1f} mm, H={obstacle['height']:.1f} mm"
        return f"Cono R={obstacle['radius']:.1f} mm, H={obstacle['height']:.1f} mm"

    @staticmethod
    def _obstacle_top_z(obstacle):
        center_z = float(obstacle['pos'][2])
        obstacle_type = obstacle.get('type', 'sphere')
        if obstacle_type == 'sphere':
            return center_z + float(obstacle['radius'])
        if obstacle_type == 'box':
            return center_z + float(obstacle['size'][2]) / 2.0
        return center_z + float(obstacle['height']) / 2.0

    @staticmethod
    def _distance_2d_point_to_segment(point, segment_start, segment_end):
        point = np.array(point, dtype=float)
        segment_start = np.array(segment_start, dtype=float)
        segment_end = np.array(segment_end, dtype=float)
        segment = segment_end - segment_start
        length_squared = float(np.dot(segment, segment))
        if length_squared <= 1e-12:
            return float(np.linalg.norm(point - segment_start))
        t = float(np.dot(point - segment_start, segment) / length_squared)
        t = float(np.clip(t, 0.0, 1.0))
        closest = segment_start + t * segment
        return float(np.linalg.norm(point - closest))

    def _obstacle_signed_distance(self, point, obstacle):
        point = np.array(point, dtype=float)
        local = point - np.array(obstacle['pos'], dtype=float)
        obstacle_type = obstacle.get('type', 'sphere')
        if obstacle_type == 'sphere':
            return float(np.linalg.norm(local) - float(obstacle['radius']))
        if obstacle_type == 'box':
            half_size = np.array(obstacle['size'], dtype=float) / 2.0
            q = np.abs(local) - half_size
            outside = float(np.linalg.norm(np.maximum(q, 0.0)))
            inside = float(min(np.max(q), 0.0))
            return outside + inside
        radial = float(math.hypot(local[0], local[1]))
        half_height = float(obstacle['height']) / 2.0
        radius = float(obstacle['radius'])
        if obstacle_type == 'cylinder':
            d = np.array([radial - radius, abs(float(local[2])) - half_height])
            outside = float(np.linalg.norm(np.maximum(d, 0.0)))
            inside = float(min(np.max(d), 0.0))
            return outside + inside
        height = float(obstacle['height'])
        z_local = float(local[2])
        local_radius = radius * (half_height - z_local) / height
        inside = -half_height <= z_local <= half_height and radial <= max(0.0, local_radius)
        if inside:
            return -float(min(max(0.0, local_radius - radial), z_local + half_height, half_height - z_local))
        profile_point = np.array([radial, z_local], dtype=float)
        side_start = np.array([radius, -half_height], dtype=float)
        side_end = np.array([0.0, half_height], dtype=float)
        side_distance = self._distance_2d_point_to_segment(profile_point, side_start, side_end)
        if radial <= radius:
            base_distance = abs(z_local + half_height)
        else:
            base_distance = math.hypot(radial - radius, z_local + half_height)
        return float(min(side_distance, base_distance))

    def _segment_hits_obstacle(self, p1, p2, obstacle, clearance=0.0):
        p1 = np.array(p1, dtype=float)
        p2 = np.array(p2, dtype=float)
        length = float(np.linalg.norm(p2 - p1))
        sample_count = max(1, int(math.ceil(length / self.OBSTACLE_SEGMENT_SAMPLE_MM)))
        for sample_index in range(sample_count + 1):
            fraction = sample_index / sample_count
            point = p1 + fraction * (p2 - p1)
            if self._obstacle_signed_distance(point, obstacle) <= float(clearance):
                return True
        return False

    def _obstacle_robot_collision_reason(self, obstacle, servo_angles=None):
        if servo_angles is None:
            servo_angles = self.current_physical_angles
        _, matrices = self.robot.compute_fk(np.array(servo_angles, dtype=float))
        margin = float(self.ROBOT_COLLISION_MARGIN)
        base_bottom = np.array([0.0, 0.0, -10.0], dtype=float)
        base_top = np.array([0.0, 0.0, float(self.robot.d[0])], dtype=float)
        if self._segment_hits_obstacle(base_bottom, base_top, obstacle, self.ROBOT_BASE_COLLISION_RADIUS + margin):
            return 'el objeto está en contacto con el centro o la base del robot'
        joint_points = [matrix[0:3, 3] for matrix in matrices]
        for index, joint_point in enumerate(joint_points):
            if self._obstacle_signed_distance(joint_point, obstacle) <= self.ROBOT_JOINT_COLLISION_RADIUS + margin:
                return f'el objeto está en contacto con la articulación {index + 1}'
        for index in range(len(matrices) - 1):
            p1 = matrices[index][0:3, 3]
            p2 = matrices[index + 1][0:3, 3]
            if self._segment_hits_obstacle(p1, p2, obstacle, self.ROBOT_LINK_COLLISION_RADIUS + margin):
                return f'el objeto está en contacto con el eslabón {index + 1}'
        return None

    def _trajectory_point_collision_reason(self, point, point_name):
        point = np.array(point, dtype=float)
        for index, obstacle in enumerate(self.obstacles, start=1):
            distance = self._obstacle_signed_distance(point, obstacle)
            if distance <= self.TOOL_COLLISION_BUFFER:
                return f'el punto {point_name} invade el objeto {index} ({self._obstacle_description(obstacle)}; distancia al sólido {distance:.1f} mm, margen mínimo {self.TOOL_COLLISION_BUFFER:.1f} mm)'
        return None

    @staticmethod
    def _polyline_length(start_point, waypoints):
        points = [np.array(start_point, dtype=float)] + [np.array(point, dtype=float) for point in waypoints]
        return float(sum(np.linalg.norm(points[index + 1] - points[index]) for index in range(len(points) - 1)))

    def _obstacle_planar_clearance_radius(self, obstacle):
        obstacle_type = obstacle.get('type', 'sphere')
        if obstacle_type == 'box':
            half_xy = np.array(obstacle['size'][0:2], dtype=float) / 2.0
            obstacle_radius = float(np.linalg.norm(half_xy))
        else:
            obstacle_radius = float(obstacle['radius'])
        return obstacle_radius + float(self.TOOL_COLLISION_BUFFER) + float(self.PATH_DETOUR_EXTRA_CLEARANCE_MM)

    def _tool_path_is_clear(self, start_point, waypoints):
        points = [np.array(start_point, dtype=float)] + [np.array(point, dtype=float) for point in waypoints]
        for point in points[1:]:
            if not self._workspace_point_is_valid(point, include_table=True):
                return False
        for index in range(len(points) - 1):
            p1 = points[index]
            p2 = points[index + 1]
            for obstacle in self.obstacles:
                if self._segment_hits_obstacle(p1, p2, obstacle, self.TOOL_COLLISION_BUFFER):
                    return False
        return True

    def _get_safe_segment_candidates(self, p1, p2, allow_backward=True):
        p1 = np.array(p1, dtype=float)
        p2 = np.array(p2, dtype=float)
        candidates = [('directa', [p2.copy()])]
        blocking_obstacles = [
            obstacle for obstacle in self.obstacles
            if self._segment_hits_obstacle(p1, p2, obstacle, self.TOOL_COLLISION_BUFFER)
        ]
        if not blocking_obstacles:
            return candidates

        highest_z = max(self._obstacle_top_z(obstacle) for obstacle in blocking_obstacles)
        safe_z = highest_z + self.TOOL_COLLISION_BUFFER + 15.0
        safe_z = min(safe_z, self.robot.max_reach * 0.9)
        wp_up = np.array([p1[0], p1[1], max(float(p1[2]), safe_z)], dtype=float)
        wp_cross = np.array([p2[0], p2[1], safe_z], dtype=float)
        candidates.append(('por arriba', [wp_up, wp_cross, p2.copy()]))

        if allow_backward:
            delta_xy = p2[0:2] - p1[0:2]
            distance_xy = float(np.linalg.norm(delta_xy))
            if distance_xy > 1e-6:
                direction = delta_xy / distance_xy
                normal = np.array([-direction[1], direction[0]], dtype=float)
                travel_z = max(float(p1[2]), float(p2[2]))

                for side_sign, side_name in ((1.0, 'izquierda'), (-1.0, 'derecha')):
                    required_offset = 40.0
                    for obstacle in blocking_obstacles:
                        obstacle_xy = np.array(obstacle['pos'][0:2], dtype=float)
                        signed_distance = float(np.dot(obstacle_xy - p1[0:2], normal))
                        obstacle_clearance = self._obstacle_planar_clearance_radius(obstacle)
                        required_offset = max(required_offset, obstacle_clearance - side_sign * signed_distance)

                    for extra_offset in (0.0, 55.0):
                        offset = min(required_offset + extra_offset, self.PATH_MAX_HORIZONTAL_DETOUR_MM)
                        lateral = normal * side_sign * offset
                        side_start = np.array([p1[0] + lateral[0], p1[1] + lateral[1], travel_z], dtype=float)
                        side_end = np.array([p2[0] + lateral[0], p2[1] + lateral[1], travel_z], dtype=float)
                        candidates.append((f'rodeando por {side_name}', [side_start, side_end, p2.copy()]))

                        for back_distance in self.PATH_BACKTRACK_DISTANCES_MM:
                            back_xy = p1[0:2] - direction * float(back_distance)
                            back_point = np.array([back_xy[0], back_xy[1], travel_z], dtype=float)
                            back_side = np.array([back_xy[0] + lateral[0], back_xy[1] + lateral[1], travel_z], dtype=float)
                            candidates.append((
                                f'hacia atrás y luego por {side_name}',
                                [back_point, back_side, side_end, p2.copy()]
                            ))

        unique_candidates = []
        seen = set()
        for label, waypoints in candidates:
            key = tuple(tuple(np.round(np.array(point, dtype=float), 2)) for point in waypoints)
            if key in seen:
                continue
            seen.add(key)
            unique_candidates.append((label, [np.array(point, dtype=float) for point in waypoints]))

        clear_candidates = [
            (label, waypoints) for label, waypoints in unique_candidates
            if self._tool_path_is_clear(p1, waypoints)
        ]
        if not clear_candidates:
            clear_candidates = unique_candidates

        clear_candidates.sort(key=lambda item: (self._polyline_length(p1, item[1]), len(item[1])))
        return clear_candidates[:max(1, int(self.PATH_MAX_CANDIDATES_PER_SEGMENT))]

    def _find_safe_path_via_points(self, start_point, point_a, point_b):
        allow_backward = bool(self.chk_path_backward.isChecked())
        candidates_to_a = self._get_safe_segment_candidates(start_point, point_a, allow_backward)
        candidates_to_b = self._get_safe_segment_candidates(point_a, point_b, allow_backward)

        def route_priority(label):
            if label == 'directa':
                return 0
            if label == 'por arriba':
                return 1
            if label.startswith('rodeando'):
                return 2
            return 3

        combinations = []
        for label_a, path_a in candidates_to_a:
            for label_b, path_b in candidates_to_b:
                combined_path = [np.array(point, dtype=float) for point in path_a + path_b]
                score = self._polyline_length(start_point, combined_path)
                priority = route_priority(label_a) + route_priority(label_b)
                combinations.append((priority, score, len(combined_path), label_a, label_b, combined_path))
        combinations.sort(key=lambda item: (item[0], item[1], item[2]))

        last_reason = 'no se encontró una combinación de desvíos válida'
        for _priority, _score, _count, label_a, label_b, combined_path in combinations:
            safe, reason = self._validate_planned_path_collision(start_point, combined_path)
            if safe:
                labels = []
                for label in (label_a, label_b):
                    if label != 'directa' and label not in labels:
                        labels.append(label)
                route_description = 'directa' if not labels else ' + '.join(labels)
                return (combined_path, route_description, '')
            last_reason = reason
        return (None, '', last_reason)

    def _validate_planned_path_collision(self, start_point, waypoints):
        points = [np.array(start_point, dtype=float)] + [np.array(point, dtype=float) for point in waypoints]
        saved_theta = self.robot.last_ik_theta.copy()
        try:
            for obs_index, obstacle in enumerate(self.obstacles, start=1):
                reason = self._obstacle_robot_collision_reason(obstacle, self.current_physical_angles)
                if reason:
                    return (False, f'el objeto {obs_index} ya colisiona con el robot: {reason}')
            for segment_index in range(len(points) - 1):
                p1 = points[segment_index]
                p2 = points[segment_index + 1]
                for obs_index, obstacle in enumerate(self.obstacles, start=1):
                    if self._segment_hits_obstacle(p1, p2, obstacle, self.TOOL_COLLISION_BUFFER):
                        return (False, f'el tramo {segment_index + 1} todavía atraviesa el objeto {obs_index} ({self._obstacle_description(obstacle)}), incluso con el margen del efector final')
                segment_length = float(np.linalg.norm(p2 - p1))
                sample_count = max(1, int(math.ceil(segment_length / self.PATH_COLLISION_SAMPLE_MM)))
                for sample_index in range(1, sample_count + 1):
                    fraction = sample_index / sample_count
                    sample = p1 + fraction * (p2 - p1)
                    if not self._workspace_point_is_valid(sample, include_table=True):
                        return (False, f'el tramo {segment_index + 1} sale del espacio seguro cerca de ({sample[0]:.1f}, {sample[1]:.1f}, {sample[2]:.1f}) mm')
                    success, servo_angles, ik_detail = self.robot.compute_ik(sample)
                    if not success:
                        return (False, f'no existe una postura segura en el tramo {segment_index + 1} cerca de ({sample[0]:.1f}, {sample[1]:.1f}, {sample[2]:.1f}) mm [{ik_detail}]')
                    for obs_index, obstacle in enumerate(self.obstacles, start=1):
                        reason = self._obstacle_robot_collision_reason(obstacle, servo_angles)
                        if reason:
                            return (False, f'en el tramo {segment_index + 1}, el objeto {obs_index} provocaría choque porque {reason}')
            return (True, '')
        finally:
            self.robot.last_ik_theta = saved_theta

    def _execute_path_planning(self):
        self._reset_path(redraw=False)
        try:
            ptA = np.array([float(self.in_pa_x.text()), float(self.in_pa_y.text()), float(self.in_pa_z.text())], dtype=float)
            ptB = np.array([float(self.in_pb_x.text()), float(self.in_pb_y.text()), float(self.in_pb_z.text())], dtype=float)
        except ValueError:
            self._show_msg('Error: ingresa coordenadas numéricas válidas para A y B.', is_error=True)
            return
        if not self._workspace_point_is_valid(ptA) or not self._workspace_point_is_valid(ptB):
            self._show_msg('Objeto o trayectoria imposible de realizar debido a riesgo de choque: A o B queda fuera del espacio seguro del robot.', is_error=True)
            return
        for point_name, point in (('A', ptA), ('B', ptB)):
            reason = self._trajectory_point_collision_reason(point, point_name)
            if reason:
                self._show_msg(f'Objeto o trayectoria imposible de realizar debido a riesgo de choque: {reason}.', is_error=True)
                return
        for obs_index, obs in enumerate(self.obstacles, start=1):
            reason = self._obstacle_robot_collision_reason(obs, self.current_physical_angles)
            if reason:
                self._show_msg(f'Objeto o trayectoria imposible de realizar debido a riesgo de choque: el objeto {obs_index} {reason}.', is_error=True)
                return
        if self.chk_path_physical.isChecked() and self.arduino_serial is None:
            self._show_msg('Activaste trayectoria física, pero el robot no está conectado por USB.', is_error=True)
            return
        self.mode_selector.setCurrentIndex(1)
        start_point = np.array(self.current_xyz, dtype=float)
        candidate_path, route_description, reason = self._find_safe_path_via_points(start_point, ptA, ptB)
        if candidate_path is None:
            backward_hint = '' if self.chk_path_backward.isChecked() else ' Activa “Permitir desvíos laterales y hacia atrás” para probar rutas alrededor del objeto.'
            self._show_msg(f'Objeto o trayectoria imposible de realizar debido a riesgo de choque: {reason}. No se inició ningún movimiento.{backward_hint}', is_error=True)
            self._redraw_robot()
            return
        self.planned_path = candidate_path
        if route_description == 'directa':
            route_text = 'Ruta directa validada sin colisión detectada en el modelo.'
        else:
            route_text = f'Ruta ajustada y validada: {route_description}.'
        mode_text = ' Se reproducirá en el robot físico.' if self.chk_path_physical.isChecked() else ' Se ejecutará solo en la vista virtual.'
        self._show_msg(route_text + mode_text)
        self.path_active = True
        self._redraw_robot()





    def _start_camera_stream(self):
        if self.vision_thread is not None:
            if self.vision_thread.isRunning():
                self._camera_restart_requested = True
                self._stop_camera_stream(clear_preview=False)
                return
            self.vision_thread.deleteLater()
            self.vision_thread = None
        self._save_vision_config(silent=True)
        worker = VisionThread(self.camera_source.text().strip(),
                              output_size=parse_camera_resolution(self.combo_camera_resolution.currentText()),
                              mirror_horizontal=self.chk_camera_mirror.isChecked(), parent=self,
                              arm_camera_api=getattr(self, 'arm_camera_api', False))
        self.vision_thread = worker
        self._camera_stop_pending = False
        self._camera_clear_preview_on_stop = False
        self._camera_restart_requested = False
        worker.frame_ready.connect(self._update_camera_frame)
        worker.status_ready.connect(lambda value: self._update_runtime_option('status', value))
        worker.fps_ready.connect(lambda value: self._update_runtime_option('fps', value))
        worker.finished.connect(self._camera_thread_finished)
        self.camera_preview.setText('Connecting camera...')
        self.lbl_camera_status.setText('Camera: opening configured source')
        self.btn_camera_connect.setEnabled(False)
        self.btn_camera_stop.setEnabled(True)
        worker.start()

    def _reset_camera_runtime_state(self, clear_preview=True):
        if clear_preview:
            self.camera_preview.setPixmap(QPixmap())
            self.camera_preview.setText('No camera image')
            self.lbl_camera_status.setText('Camera stopped')
        self.lbl_camera_fps.setText('FPS: 0.0')

    def _stop_camera_stream(self, clear_preview=True):
        worker = self.vision_thread
        if self._camera_stop_pending:
            self._camera_clear_preview_on_stop = self._camera_clear_preview_on_stop or bool(clear_preview)
        else:
            self._camera_clear_preview_on_stop = bool(clear_preview)
        if worker is not None and worker.isRunning():
            self._camera_stop_pending = True
            worker.stop()
            if hasattr(self, 'btn_camera_connect'):
                self.btn_camera_connect.setEnabled(False)
            if hasattr(self, 'btn_camera_stop'):
                self.btn_camera_stop.setEnabled(False)
            if hasattr(self, 'lbl_camera_status'):
                self.lbl_camera_status.setText('Cámara: deteniendo de forma segura (máx. ~1–3 s)...')
            return
        if worker is not None:
            worker.deleteLater()
            self.vision_thread = None
        self._camera_stop_pending = False
        self._reset_camera_runtime_state(clear_preview=clear_preview)
        if hasattr(self, 'btn_camera_connect'):
            self.btn_camera_connect.setEnabled(True)
        if hasattr(self, 'btn_camera_stop'):
            self.btn_camera_stop.setEnabled(False)

    def _camera_thread_finished(self):
        worker = self.sender()
        if worker is self.vision_thread:
            self.vision_thread = None
        if worker is not None:
            worker.deleteLater()
        clear_preview = self._camera_clear_preview_on_stop
        restart_requested = self._camera_restart_requested
        closing = self._closing_after_camera_stop
        self._camera_stop_pending = False
        self._camera_clear_preview_on_stop = True
        self._camera_restart_requested = False
        self._reset_camera_runtime_state(clear_preview=clear_preview)
        if hasattr(self, 'btn_camera_connect'):
            self.btn_camera_connect.setEnabled(not closing)
        if hasattr(self, 'btn_camera_stop'):
            self.btn_camera_stop.setEnabled(False)
        if closing:
            self._closing_after_camera_stop = False
            QTimer.singleShot(0, self.close)
        elif restart_requested:
            QTimer.singleShot(0, self._start_camera_stream)

    def _update_camera_frame(self, image: QImage):
        pixmap = QPixmap.fromImage(image).scaled(self.camera_preview.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        self.camera_preview.setPixmap(pixmap)

    def _vision_config_defaults(self):
        return {'version': 11, 'camera': {'resolution': '320x240', 'mirror_horizontal': False}}

    @staticmethod
    def _number(value, default, minimum, maximum):
        try:
            if hasattr(value, 'text'):
                value = value.text().strip()
            number = float(value)
        except Exception:
            number = float(default)
        return float(np.clip(number, minimum, maximum))

    def _collect_vision_config(self):
        return {'version': 11, 'camera': {'resolution': self.combo_camera_resolution.currentText(),
                                        'mirror_horizontal': self.chk_camera_mirror.isChecked()}}

    def _save_vision_config(self, _checked=False, silent=False):
        try:
            config = self._collect_vision_config()
            self.vision_config_path.write_text(json.dumps(config, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
            if not silent:
                self._show_msg(f'Configuración guardada en {self.vision_config_path.name}.')
            return True
        except Exception as exc:
            if not silent:
                self._show_msg(f'No se pudo guardar el JSON: {exc}', is_error=True)
            return False

    def _load_vision_config(self, _checked=False, silent=False, create_if_missing=False):
        config = self._vision_config_defaults()
        if self.vision_config_path.exists():
            try:
                loaded = json.loads(self.vision_config_path.read_text(encoding='utf8'))
                camera = loaded.get('camera', {})
                for key in config['camera']:
                    if key in camera: config['camera'][key] = camera[key]
            except (OSError, ValueError, TypeError, AttributeError) as exc:
                if not silent: self._show_msg(f'Invalid camera settings: {exc}', is_error=True)
        camera = config['camera']
        resolution = str(camera['resolution'])
        if self.combo_camera_resolution.findText(resolution) >= 0:
            self.combo_camera_resolution.setCurrentText(resolution)
        self.chk_camera_mirror.setChecked(bool(camera['mirror_horizontal']))
        if create_if_missing: self._save_vision_config(silent=True)
        return True

    def _workspace_point_is_valid(self, point, include_table=False):
        point = np.array(point, dtype=float)
        if not np.all(np.isfinite(point)):
            return False
        if point[1] < 0.0:
            return False
        if point[2] < (0.0 if include_table else 5.0):
            return False
        planar_radius = math.hypot(point[0], point[1])
        if planar_radius < 70.0:
            return False
        return np.linalg.norm(point) <= self.robot.max_reach * 0.98


    def _trigger_safe_recovery(self, reason):
        # A software error stops targets; it never creates a recovery trajectory.
        self.path_active = False
        self.planned_path = []
        self.slider_target_fk = self.current_physical_angles.copy()
        self.slider_target_xyz = np.array(self.current_xyz).copy()
        self.chk_live.setChecked(False)
        self.chk_path_physical.setChecked(False)
        self._show_msg(str(reason), is_error=True)

    def _control_loop_tick(self):
        if self._syncing:
            return
        try:
            self._control_loop_tick_guarded()
        except Exception as exc:
            self._trigger_safe_recovery(f'Excepción de control {type(exc).__name__}: {exc}')

    def _control_loop_tick_guarded(self):
        pendant_control_active = self.chk_pendant.isChecked() and not self.path_active
        manual_override = False
        if self.pendant_serial:
            manual_override = self._read_pendant(allow_motion=pendant_control_active)
        self._maintain_pendant_connection(time.time())
        moved = False
        if self.path_active:
            if self.planned_path:
                current_target = self.planned_path[0]
                self.slider_target_xyz = current_target.copy()
                if np.linalg.norm(self.slider_target_xyz - self.current_xyz) < 5.0:
                    self.planned_path.pop(0)
                    if not self.planned_path:
                        self.path_active = False
                        self._show_msg('¡Trayectoria completada!')
                        self._redraw_robot()
                self._syncing = True
                self.ik_sliders[0].setValue(int(self.slider_target_xyz[0]))
                self.ik_sliders[1].setValue(int(self.slider_target_xyz[1]))
                self.ik_sliders[2].setValue(int(self.slider_target_xyz[2]))
                self._syncing = False
        if pendant_control_active:
            if manual_override:
                moved = True
            max_lead = 15.0
            leash_diff = self.pendant_target_xyz - self.current_xyz
            leash_dist = np.linalg.norm(leash_diff)
            if leash_dist > max_lead:
                self.pendant_target_xyz = self.current_xyz + leash_diff / leash_dist * max_lead
            max_r = self.robot.max_reach
            self.pendant_target_xyz[0] = np.clip(self.pendant_target_xyz[0], -max_r, max_r)
            self.pendant_target_xyz[1] = np.clip(self.pendant_target_xyz[1], 0, max_r)
            self.pendant_target_xyz[2] = np.clip(self.pendant_target_xyz[2], 0, max_r)
            self.pendant_target_xyz = self._enforce_safety_zone(self.pendant_target_xyz)
            diff = self.pendant_target_xyz - self.current_xyz
            dist = np.linalg.norm(diff)
            if dist > 0.5:
                max_step = self.max_speed_mms * 0.05
                next_xyz = self.current_xyz + diff / dist * min(dist, max_step)
                success, servo_deg, msg = self.robot.compute_ik(next_xyz)
                if success:
                    final_pos, matrices = self.robot.compute_fk(servo_deg)
                    self.current_physical_angles = servo_deg
                    self.current_xyz = final_pos
                    self._draw_robot(matrices)
                    self._update_text_results(self.current_physical_angles, self.current_xyz)
                    moved = True
                else:
                    if self.pendant_serial:
                        self._pendant_write_line(f'LIMIT:{msg}')
                    self.pendant_target_xyz = self.current_xyz.copy()
        else:
            mode = self.mode_selector.currentIndex()
            if mode == 0 and (not self.path_active):
                diff = self.slider_target_fk - self.current_physical_angles
                max_step = self.max_speed_degs * 0.05
                step = np.clip(diff, -max_step, max_step)
                if np.any(np.abs(step) > 0.1):
                    next_angles = self.current_physical_angles + step
                    final_pos, matrices = self.robot.compute_fk(next_angles)
                    self.current_physical_angles = next_angles
                    self.current_xyz = final_pos
                    self._draw_robot(matrices)
                    self._update_text_results(self.current_physical_angles, self.current_xyz)
                    moved = True
            else:
                self.slider_target_xyz = self._enforce_safety_zone(self.slider_target_xyz)
                diff = self.slider_target_xyz - self.current_xyz
                dist = np.linalg.norm(diff)
                if dist > 0.5:
                    max_step = self.max_speed_mms * 0.05
                    next_xyz = self.current_xyz + diff / dist * min(dist, max_step)
                    success, servo_deg, msg = self.robot.compute_ik(next_xyz)
                    if success:
                        final_pos, matrices = self.robot.compute_fk(servo_deg)
                        self.current_physical_angles = servo_deg
                        self.current_xyz = final_pos
                        self._draw_robot(matrices)
                        self._update_text_results(self.current_physical_angles, self.current_xyz)
                        moved = True
                    else:
                        self._show_msg(msg, is_error=True)
        current_time = time.time()
        path_physical = self.path_active and self.chk_path_physical.isChecked()
        send_requested = moved or self._force_hardware_send
        send_enabled = self.chk_live.isChecked() or self.chk_pendant.isChecked() or path_physical
        if send_requested and send_enabled and (current_time - self.last_robot_send_time > 0.1):
            self._send_to_arduino(live_mode=True)
            self.last_robot_send_time = current_time
            self._force_hardware_send = False
        now_mono = time.monotonic()
        if now_mono - self._last_vision_draw_time >= 0.25:
            self._last_vision_draw_time = now_mono
            self._redraw_robot()
        if self.pendant_serial and current_time - self.last_pendant_send_time > 0.1:
            x, y, z = self.current_xyz
            grip = self.slider_pinza.value()
            state_cmd = f'STATE:{x:.1f},{y:.1f},{z:.1f},0.0,0.0,{grip:.1f}'
            if self._pendant_write_line(state_cmd):
                self.last_pendant_send_time = current_time

    def _update_text_results(self, servo_deg, final_pos):
        ang_str = ', '.join([f'{a:.1f}' for a in servo_deg])
        self.lbl_angles.property('value_label').setText(f'[{ang_str}] °')
        pos_str = f'X: {final_pos[0]:.0f}, Y: {final_pos[1]:.0f}, Z: {final_pos[2]:.0f}'
        self.lbl_pos.property('value_label').setText(f'[{pos_str}] mm')

    def _redraw_robot(self):
        try:
            _, matrices = self.robot.compute_fk(self.current_physical_angles)
            self._draw_robot(matrices)
        except Exception:
            pass

    def _draw_robot(self, T_matrices):
        view_elev = float(getattr(self.ax, 'elev', 30.0))
        view_azim = float(getattr(self.ax, 'azim', 135.0))
        self.ax.cla()
        self.ax.set_facecolor(self.BG_MAIN)

        def draw_cylinder(p0, p1, R, color):
            v = p1 - p0
            mag = np.linalg.norm(v)
            if mag < 1e-05:
                return
            v = v / mag
            not_v = np.array([1, 0, 0])
            if np.abs(np.dot(v, not_v)) > 0.99:
                not_v = np.array([0, 1, 0])
            n1 = np.cross(v, not_v)
            n1 /= np.linalg.norm(n1)
            n2 = np.cross(v, n1)
            t = np.linspace(0, mag, 5)
            theta = np.linspace(0, 2 * np.pi, 12)
            t, theta = np.meshgrid(t, theta)
            X = p0[0] + v[0] * t + R * np.sin(theta) * n1[0] + R * np.cos(theta) * n2[0]
            Y = p0[1] + v[1] * t + R * np.sin(theta) * n1[1] + R * np.cos(theta) * n2[1]
            Z = p0[2] + v[2] * t + R * np.sin(theta) * n1[2] + R * np.cos(theta) * n2[2]
            self.ax.plot_surface(X, Y, Z, color=color, shade=True, linewidth=0, antialiased=False)
        draw_cylinder(np.array([0, 0, -10]), np.array([0, 0, 0]), 45, '#475569')
        self.ax.plot([-400, 400], [0, 0], [0, 0], color='#EF4444', linewidth=2, alpha=0.7)
        self.ax.plot([0, 0], [0, 400], [0, 0], color='#10B981', linewidth=2, alpha=0.7)
        self.ax.plot([0, 0], [0, 0], [0, 400], color='#3B82F6', linewidth=2, alpha=0.7)
        self.ax.text(420, 0, 0, 'Mesa +X (0°)', color='#EF4444', fontweight='bold')
        self.ax.text(-420, 0, 0, 'Mesa -X (180°)', color='#EF4444', fontweight='bold')
        self.ax.text(0, 420, 0, 'Mesa +Y (90°)', color='#10B981', fontweight='bold')
        self.ax.text(0, 0, 420, 'Arriba (Z)', color='#3B82F6', fontweight='bold')
        for i in range(4):
            p_curr = T_matrices[i][0:3, 3]
            p_next = T_matrices[i + 1][0:3, 3]
            z_axis = T_matrices[i][0:3, 2]
            p0_j = p_curr - z_axis * 18
            p1_j = p_curr + z_axis * 18
            draw_cylinder(p0_j, p1_j, 16, '#3B82F6')
            draw_cylinder(p_curr, p_next, 6, '#94A3B8')
        p_ee = T_matrices[-1][0:3, 3]
        self.ax.scatter([p_ee[0]], [p_ee[1]], [p_ee[2]], color='#EF4444', s=100, marker='o', edgecolor='w')
        show_obstacles = hasattr(self, 'tabs') and hasattr(self, 'path_tab_index') and (self.tabs.currentIndex() == self.path_tab_index)
        if show_obstacles:
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection
            for obstacle_index, obs in enumerate(self.obstacles, start=1):
                obstacle_type = obs.get('type', 'sphere')
                cx, cy, cz = np.array(obs['pos'], dtype=float)
                if obstacle_type == 'sphere':
                    radius = float(obs['radius'])
                    u, v = np.mgrid[0:2 * np.pi:18j, 0:np.pi:12j]
                    x = cx + radius * np.cos(u) * np.sin(v)
                    y = cy + radius * np.sin(u) * np.sin(v)
                    z = cz + radius * np.cos(v)
                    self.ax.plot_surface(x, y, z, color='#EF4444', alpha=0.4, edgecolor='none')
                elif obstacle_type == 'box':
                    hx, hy, hz = np.array(obs['size'], dtype=float) / 2.0
                    vertices = np.array([[cx - hx, cy - hy, cz - hz], [cx + hx, cy - hy, cz - hz], [cx + hx, cy + hy, cz - hz], [cx - hx, cy + hy, cz - hz], [cx - hx, cy - hy, cz + hz], [cx + hx, cy - hy, cz + hz], [cx + hx, cy + hy, cz + hz], [cx - hx, cy + hy, cz + hz]])
                    faces = [[vertices[i] for i in (0, 1, 2, 3)], [vertices[i] for i in (4, 5, 6, 7)], [vertices[i] for i in (0, 1, 5, 4)], [vertices[i] for i in (1, 2, 6, 5)], [vertices[i] for i in (2, 3, 7, 6)], [vertices[i] for i in (3, 0, 4, 7)]]
                    collection = Poly3DCollection(faces, facecolor='#EF4444', edgecolor='#FCA5A5', alpha=0.4)
                    self.ax.add_collection3d(collection)
                else:
                    radius = float(obs['radius'])
                    height = float(obs['height'])
                    theta = np.linspace(0.0, 2.0 * np.pi, 24)
                    z_values = np.linspace(cz - height / 2.0, cz + height / 2.0, 10)
                    theta_grid, z_grid = np.meshgrid(theta, z_values)
                    if obstacle_type == 'cylinder':
                        radius_grid = np.full_like(z_grid, radius)
                    else:
                        radius_grid = radius * ((cz + height / 2.0 - z_grid) / height)
                    x = cx + radius_grid * np.cos(theta_grid)
                    y = cy + radius_grid * np.sin(theta_grid)
                    self.ax.plot_surface(x, y, z_grid, color='#EF4444', alpha=0.4, edgecolor='none')
                    base_x = cx + radius * np.cos(theta)
                    base_y = cy + radius * np.sin(theta)
                    base_z = np.full_like(theta, cz - height / 2.0)
                    base_vertices = [list(zip(base_x, base_y, base_z))]
                    self.ax.add_collection3d(Poly3DCollection(base_vertices, facecolor='#EF4444', edgecolor='#FCA5A5', alpha=0.4))
                self.ax.text(cx, cy, self._obstacle_top_z(obs) + 10.0, f'O{obstacle_index}', color='#FCA5A5', fontweight='bold')
        if self.path_active and len(self.planned_path) > 0:
            path_pts = [self.current_xyz] + self.planned_path
            xs, ys, zs = zip(*path_pts)
            self.ax.plot(xs, ys, zs, color='#F59E0B', linestyle='--', linewidth=2, marker='o')
        r = self.robot.max_reach
        self.ax.set_xlim([-r, r])
        self.ax.set_ylim([-100, r])
        self.ax.set_zlim([0, r])
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        self.ax.set_zticklabels([])
        self.ax.grid(color='#3F3F46', linestyle='--', linewidth=0.5)
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.view_init(elev=view_elev, azim=view_azim)
        self.canvas.draw_idle()
if __name__ == '__main__':
    print('[ROBOT FK/IK V6.2 · OPENCV LOCAL] Ejecutando:', __file__)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
