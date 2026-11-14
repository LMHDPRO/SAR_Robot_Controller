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
        self.startup_angles = np.array([90.0, 126.5, 74.2, 106.0])
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
    detections_ready = Signal(list)
    status_ready = Signal(str)
    fps_ready = Signal(float)

    def __init__(self, stream_url: str, output_size=(320, 320), max_objects: int=4, use_yolo: bool=False, model_path: str='best.pt', mirror_horizontal: bool=False, yolo_stride: int=2, confidence: float=0.70, preview_fps_limit: float=20.0, parent=None):
        super().__init__(parent)
        self.stream_url = stream_url
        self.output_size = tuple(output_size)
        self.max_objects = max(1, int(max_objects))
        self.use_yolo = bool(use_yolo)
        self.model_path = model_path.strip() or 'best.pt'
        self.mirror_horizontal = bool(mirror_horizontal)
        self.yolo_stride = max(1, int(yolo_stride))
        self.confidence = float(np.clip(confidence, 0.05, 0.95))
        self.preview_fps_limit = max(1.0, float(preview_fps_limit))
        self._stop_event = threading.Event()
        self.cap = None
        self.model = None
        self.frame_counter = 0
        self.last_detections = []
        self.yolo_paused = False

    def set_yolo_paused(self, paused: bool, reason: str=''):
        self.yolo_paused = bool(paused)
        if self.yolo_paused:
            self.last_detections = []

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

    def _load_yolo(self):
        if not self.use_yolo:
            return
        try:
            from ultralytics import YOLO
            self.status_ready.emit(f'Cargando YOLO: {self.model_path}')
            self.model = YOLO(self.model_path)
            try:
                self.model.fuse()
            except Exception:
                pass
            self.status_ready.emit('YOLO listo')
        except Exception as exc:
            self.model = None
            self.status_ready.emit(f'YOLO no disponible ({exc}). El preview continuará sin detección.')

    def _run_yolo(self, frame):
        if self.yolo_paused:
            self.last_detections = []
            return []
        if self.model is None or self.frame_counter % self.yolo_stride != 0:
            return self.last_detections
        try:
            device = 'cpu'
            half = False
            try:
                import torch
                if torch.cuda.is_available():
                    device = 0
                    half = True
            except Exception:
                pass
            results = self.model.predict(source=frame, imgsz=320, conf=self.confidence, device=device, half=half, verbose=False)
            result = results[0]
            names = result.names
            detections = []
            if result.boxes is not None:
                for box in result.boxes:
                    cls_id = int(box.cls[0].item())
                    confidence = float(box.conf[0].item())
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    x1 = int(max(0, x1))
                    y1 = int(max(0, y1))
                    x2 = int(min(frame.shape[1] - 1, x2))
                    y2 = int(min(frame.shape[0] - 1, y2))
                    detections.append({'class': names.get(cls_id, str(cls_id)), 'confidence': confidence, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'cx': int((x1 + x2) / 2), 'cy': int((y1 + y2) / 2), 'w': int(x2 - x1), 'h': int(y2 - y1), 'frame_w': int(frame.shape[1]), 'frame_h': int(frame.shape[0])})
            detections.sort(key=lambda item: item['confidence'], reverse=True)
            self.last_detections = detections[:max(1, int(self.max_objects))]
        except Exception as exc:
            self.status_ready.emit(f'Error YOLO: {exc}')
        return self.last_detections

    def _emit_processed_frame(self, cv2, frame, previous_time, fps_ema, last_emit_time):
        width, height = self.output_size
        frame = cv2.resize(frame, (int(width), int(height)), interpolation=cv2.INTER_AREA)
        if self.mirror_horizontal:
            frame = cv2.flip(frame, 1)
        detections = self._run_yolo(frame) if self.use_yolo else []
        visible_detections = detections[:max(1, int(self.max_objects))]
        now = time.time()
        dt = now - previous_time
        instantaneous_fps = 1.0 / dt if dt > 0 else 0.0
        fps_ema = instantaneous_fps if fps_ema <= 0 else 0.85 * fps_ema + 0.15 * instantaneous_fps
        if now - last_emit_time >= 1.0 / self.preview_fps_limit:
            annotated = frame.copy()
            for det in visible_detections:
                cv2.rectangle(annotated, (det['x1'], det['y1']), (det['x2'], det['y2']), (0, 255, 0), 2)
                cv2.circle(annotated, (det['cx'], det['cy']), 4, (0, 0, 255), -1)
            rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            h, w, channels = rgb.shape
            self.frame_ready.emit(QImage(rgb.data, w, h, channels * w, QImage.Format_RGB888).copy())
            self.detections_ready.emit(list(visible_detections))
            self.fps_ready.emit(fps_ema)
            last_emit_time = now
        self.frame_counter += 1
        return (now, fps_ema, last_emit_time)

    def run(self):
        try:
            import cv2
        except Exception as exc:
            self.status_ready.emit(f'OpenCV no está instalado: {exc}')
            return
        self._stop_event.clear()
        final_status = 'Cámara detenida'
        try:
            self._load_yolo()
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
    SCAN_RADIUS_DEFAULT = 120.0
    BLOCK_HEIGHT_MM = 20.0
    BLOCK_PICK_Z_MM = 15.0
    CAMERA_TO_GRIPPER_X_MM = -10.0
    CAMERA_TO_GRIPPER_Y_MM = 20.0
    FINAL_RADIAL_PICK_TRIM_MM = 2.5
    RADIAL_COMP_Q1_MM = 10.0
    RADIAL_COMP_CENTER_MM = 10
    RADIAL_COMP_Q2_MM = -5.5
    RADIAL_COMP_Q1_ANGLE_DEG = 45.0
    RADIAL_COMP_CENTER_ANGLE_DEG = 90.0
    RADIAL_COMP_Q2_ANGLE_DEG = 135.0
    PICK_XY_COMP_Q1_X_MM = 5.0
    PICK_XY_COMP_Q1_Y_MM = 13.0
    PICK_XY_COMP_CENTER_X_MM = 38.0
    PICK_XY_COMP_CENTER_Y_MM = -9.0
    PICK_XY_COMP_Q2_X_MM = 30.0
    PICK_XY_COMP_Q2_Y_MM = 20.0
    ANGLE_EDGE_XY_MAX_MM = 30.0
    YOLO_CONFIDENCE = 0.70
    VISION_BASE_STEP_DEG = 1.0
    VISION_BASE_MIN_STEP_DEG = 1.0
    VISION_BASE_STABLE_HITS = 2
    VISION_RADIAL_SERVO_MARGIN_DEG = 0.5
    AUTOMATIC_SPEED_DEFAULT_DPS = 130.0
    SCAN_DWELL_TICKS = 45
    DROP_ZONE_DEFAULT = (-210.0, 70.0, 35.0)
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
    SCAN_Z_MM = 140.0
    SCAN_PITCH_DEG = 85.0
    PICK_PITCH_DEG = 80.0
    PREGRASP_CLEARANCE_MM = 40.0
    LIFT_DISTANCE_MM = 90.0
    VISION_STEP_MM = 5.0
    JOINT_MARGIN_DEG = 5.0
    RECOVERY_Z_MM = 120.0
    RECOVERY_SPEED_DPS = 15.0
    CARRY_PITCH_DEG = 70.0
    ALLOWED_CLASSES = ('Linea', 'L', 'T')
    MANUAL_GRIPPER_MAX_DEG = 75
    VISION_GRIPPER_OPEN_DEG = 90

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
        self.vision_timer = 0
        self.drop_zone_xyz = np.array(self.DROP_ZONE_DEFAULT, dtype=float)
        self._stable_detection = None
        self._stable_detection_hits = 0
        self.target_object_xyz = np.zeros(3)
        self.active_detection_id = None
        self.observe_mode_active = False
        self._last_vision_draw_time = 0.0
        self._last_scene_table_update = 0.0
        self._scene_table_dirty = False
        self.scene_objects = {}
        self._next_scene_object_id = 1
        self.locked_detection = None
        self.vision_detection_armed = False
        self.vision_motion_queue = []
        self.vision_actions = []
        self.vision_segment_next_state = None
        self.vision_segment_label = ''
        self.vision_motion_speed_dps = self.AUTOMATIC_SPEED_DEFAULT_DPS
        self.scan_radius_mm = self.SCAN_RADIUS_DEFAULT
        self.scan_z_mm = self.SCAN_Z_MM
        self.scan_pitch_deg = self.SCAN_PITCH_DEG
        self.recovery_active = False
        self.recovery_stage = 'IDLE'
        self.recovery_joint_target = None
        self.recovery_reason = ''
        self.recovery_speed_dps = 15.0
        self.recovery_safe_z = 120.0
        self.scan_points = []
        self.scan_servo_targets = []
        self.scan_point_index = 0
        self.scan_direction = 1
        self.completed_pick_count = 0
        self.vision_centering_scene_id = None
        self.vision_centering_attempts = 0
        self.vision_centering_limit = 60
        self.vision_centering_pixel_deadzone = 10.0
        self.vision_centering_required_stable_hits = 3
        self.vision_centering_stable_hits = 0
        self.vision_centering_phase = 'BASE'
        self.vision_locked_base_servo_deg = None
        self.vision_pending_relock_base_deg = None
        self.required_live_detection_hits = 1
        self.vision_tracking_pitch_deg = None
        self.vision_last_center_candidate = None
        self.vision_last_center_pose = None
        self.vision_center_loss_ticks = 0
        self.vision_center_occlusion_grace_ticks = 8
        self.vision_center_occlusion_lock_delay = 2
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
        self.slider_pinza.setValue(self.MANUAL_GRIPPER_MAX_DEG)
        self.slider_pinza.valueChanged.connect(lambda value: self.lbl_pinza.setText(f'Pinza (Gripper): {value}°'))
        for widget in (self.lbl_vel, self.slider_vel, self.lbl_pinza, self.slider_pinza):
            motion_layout.addWidget(widget)
        manual_layout.addWidget(motion)
        source = QGroupBox('Fuente de control')
        source_layout = QVBoxLayout(source)
        self.chk_live = make('check', 'chk_live', 'Control PC (Live Sliders)')
        self.chk_pendant = make('check', 'chk_pendant', 'Control Remoto (ESP32 Gamepad)')
        self.chk_pendant.toggled.connect(self._on_pendant_toggle)
        send = QPushButton('Enviar al Robot (Manual)')
        send.clicked.connect(lambda: self._send_to_arduino(live_mode=False))
        source_layout.addWidget(self.chk_live)
        source_layout.addWidget(self.chk_pendant)
        source_layout.addWidget(send)
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
        auto_box = QGroupBox('Visión Artificial · Pick & Place')
        auto_layout = QVBoxLayout(auto_box)
        auto_buttons = QHBoxLayout()
        self.btn_vision = QPushButton('Iniciar búsqueda continua')
        self.btn_vision.clicked.connect(self._toggle_vision)
        self.btn_observe_mode = QPushButton('Modo observar')
        self.btn_observe_mode.setCheckable(True)
        self.btn_observe_mode.toggled.connect(self._toggle_observe_mode)
        auto_buttons.addWidget(self.btn_vision, 2)
        auto_buttons.addWidget(self.btn_observe_mode, 1)
        auto_layout.addLayout(auto_buttons)
        self.chk_vision_physical = make('check', 'chk_vision_physical', 'Permitir movimiento físico durante Pick & Place')
        auto_layout.addWidget(self.chk_vision_physical)
        self.lbl_vision_phase = QLabel('Fase: IDLE')
        self.lbl_vision_phase.setWordWrap(True)
        self.lbl_yolo_runtime = QLabel('YOLO: disponible al preparar cámara')
        self.lbl_vision_target = QLabel('Objetivo: sin detección estable')
        self.lbl_vision_target.setWordWrap(True)
        auto_layout.addWidget(self.lbl_vision_phase)
        auto_layout.addWidget(self.lbl_yolo_runtime)
        auto_layout.addWidget(self.lbl_vision_target)
        recovery = QPushButton('Recuperación lenta a posición segura')
        recovery.clicked.connect(lambda: self._trigger_safe_recovery('Recuperación solicitada por el operador'))
        auto_layout.addWidget(recovery)
        vision_layout.addWidget(auto_box)
        camera = QGroupBox('Cámara ESP32 · Live Feed + YOLO')
        camera_grid = QGridLayout(camera)
        camera_grid.addWidget(QLabel('URL IP / índice USB'), 0, 0)
        self.camera_source = QLineEdit('1')
        self.camera_source.setPlaceholderText('http://IP:81/stream o índice USB')
        camera_grid.addWidget(self.camera_source, 0, 1)
        self.combo_camera_resolution = make('combo', 'combo_camera_resolution', ('256x256', '320x320', '416x416', '480x480', '640x640'), '320x320')
        self.combo_camera_resolution.currentTextChanged.connect(lambda value: self._update_runtime_option('resolution', value))
        camera_grid.addWidget(QLabel('Resolución'), 0, 2)
        camera_grid.addWidget(self.combo_camera_resolution, 0, 3)
        self.chk_camera_mirror = make('check', 'chk_camera_mirror', 'Espejo horizontal')
        self.chk_camera_mirror.toggled.connect(lambda value: self._update_runtime_option('mirror', value))
        camera_grid.addWidget(self.chk_camera_mirror, 1, 0, 1, 2)
        self.lbl_camera_objects = QLabel('Máximo de objetos: 4')
        self.slider_camera_objects = QSlider(Qt.Orientation.Horizontal)
        self.slider_camera_objects.setRange(1, 8)
        self.slider_camera_objects.setValue(4)
        self.slider_camera_objects.valueChanged.connect(lambda value: self._update_runtime_option('objects', value))
        camera_grid.addWidget(self.lbl_camera_objects, 1, 2)
        camera_grid.addWidget(self.slider_camera_objects, 1, 3)
        self.chk_camera_yolo = make('check', 'chk_camera_yolo', 'Activar YOLO')
        camera_grid.addWidget(self.chk_camera_yolo, 2, 0, 1, 4)
        camera_grid.addWidget(QLabel('Modelo YOLO'), 3, 0)
        model_path_row = QHBoxLayout()
        model_path_row.setContentsMargins(0, 0, 0, 0)
        model_path_row.addWidget(make('edit', 'input_camera_model', 'best.pt'), 1)
        self.btn_select_yolo_model = QPushButton('Seleccionar .pt')
        self.btn_select_yolo_model.clicked.connect(self._select_yolo_model)
        model_path_row.addWidget(self.btn_select_yolo_model)
        camera_grid.addLayout(model_path_row, 3, 1, 1, 2)
        self.combo_yolo_stride = make('combo', 'combo_yolo_stride', ('Cada frame', 'Cada 2 frames', 'Cada 3 frames', 'Cada 4 frames'), 1)
        camera_grid.addWidget(self.combo_yolo_stride, 3, 3)
        axis_row = QHBoxLayout()
        self.chk_camera_axis_x_invert = make('check', 'chk_camera_axis_x_invert', 'Invertir giro base')
        self.chk_camera_axis_y_invert = make('check', 'chk_camera_axis_y_invert', 'Invertir avance radial')
        axis_row.addWidget(self.chk_camera_axis_x_invert)
        axis_row.addWidget(self.chk_camera_axis_y_invert)
        camera_grid.addLayout(axis_row, 4, 0, 1, 2)
        self.lbl_overhead_alignment = QLabel('Control visual: giro de base → avance radial → descenso Z')
        self.lbl_overhead_alignment.setWordWrap(True)
        camera_grid.addWidget(self.lbl_overhead_alignment, 4, 2, 1, 2)
        camera_buttons = QHBoxLayout()
        self.btn_camera_connect = QPushButton('Conectar cámara')
        self.btn_camera_connect.clicked.connect(self._start_camera_stream)
        self.btn_camera_stop = QPushButton('Detener')
        self.btn_camera_stop.setEnabled(False)
        self.btn_camera_stop.clicked.connect(lambda: self._stop_camera_stream())
        camera_buttons.addWidget(self.btn_camera_connect)
        camera_buttons.addWidget(self.btn_camera_stop)
        camera_grid.addLayout(camera_buttons, 5, 0, 1, 4)
        self.camera_preview = QLabel('Preview de cámara\nSin conexión')
        self.camera_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_preview.setFixedSize(320, 320)
        camera_grid.addWidget(self.camera_preview, 6, 0, 1, 4, Qt.AlignmentFlag.AlignHCenter)
        camera_status = QHBoxLayout()
        self.lbl_camera_status = QLabel('Cámara: desconectada')
        self.lbl_camera_status.setWordWrap(True)
        self.lbl_camera_fps = QLabel('FPS: 0.0')
        self.lbl_detected_objects = QLabel('Detecciones válidas: 0')
        camera_status.addWidget(self.lbl_camera_status, 3)
        camera_status.addWidget(self.lbl_camera_fps)
        camera_status.addWidget(self.lbl_detected_objects)
        camera_grid.addLayout(camera_status, 7, 0, 1, 4)
        vision_layout.addWidget(camera)
        objects = QGroupBox('Objetos Tetris identificados')
        objects_layout = QVBoxLayout(objects)
        self.table_scene_objects = QTableWidget(0, 7)
        self.table_scene_objects.setHorizontalHeaderLabels(('ID', 'Clase', 'X', 'Y', 'Z', 'Conf.', 'Estado'))
        self.table_scene_objects.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table_scene_objects.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table_scene_objects.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.lbl_seen_classes = QLabel('Objetos vistos: ninguno')
        clear_objects = QPushButton('Limpiar mapa de objetos')
        clear_objects.clicked.connect(self._clear_scene_objects)
        objects_layout.addWidget(self.table_scene_objects)
        objects_layout.addWidget(self.lbl_seen_classes)
        objects_layout.addWidget(clear_objects)
        vision_layout.addWidget(objects)
        drop = QGroupBox('Lugar para dejar el objeto')
        drop_grid = QGridLayout(drop)
        for col, (axis, attr, value) in enumerate(zip('XYZ', ('input_drop_x', 'input_drop_y', 'input_drop_z'), self.drop_zone_xyz)):
            drop_grid.addWidget(QLabel(f'{axis} [mm]'), 0, col)
            drop_grid.addWidget(make('edit', attr, f'{value:.0f}'), 1, col)
        apply_drop = QPushButton('Aplicar lugar de entrega')
        apply_drop.clicked.connect(self._apply_drop_zone)
        drop_grid.addWidget(apply_drop, 2, 0, 1, 3)
        self.lbl_drop_zone = QLabel(f'Destino actual: X={self.drop_zone_xyz[0]:.0f}, Y={self.drop_zone_xyz[1]:.0f}, Z={self.drop_zone_xyz[2]:.0f} mm')
        drop_grid.addWidget(self.lbl_drop_zone, 3, 0, 1, 3)
        vision_layout.addWidget(drop)
        vision_layout.addStretch()
        make('tab', vision, 'Visión')
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
        elif option == 'objects':
            self.lbl_camera_objects.setText(f'Máximo de objetos: {value}')
            if worker is not None:
                worker.max_objects = int(value)
        elif option == 'resolution' and worker is not None:
            width, height = parse_camera_resolution(str(value))
            side = min(width, height)
            worker.output_size = (side, side)
        elif option == 'mirror':
            if worker is not None:
                worker.mirror_horizontal = bool(value)
            self._reset_detection_runtime()

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
        self.vision_state = 'IDLE'
        self.vision_centering_phase = 'BASE'
        self.vision_locked_base_servo_deg = None
        self.vision_pending_relock_base_deg = None
        self.vision_tracking_pitch_deg = None
        self.btn_vision.setText('Iniciar búsqueda continua')
        self.btn_vision.setStyleSheet('background-color: #8B5CF6; color: #FFF; border: none; border-radius: 6px; font-weight: bold; font-size: 14px;')
        if self.observe_mode_active:
            self.btn_observe_mode.setChecked(False)
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

    def _resolve_yolo_model_path(self, model_path=None):
        raw_path = (model_path if model_path is not None else self.input_camera_model.text()).strip()
        raw_path = raw_path or 'best.pt'
        path = Path(raw_path).expanduser()
        if not path.is_absolute():
            path = Path(__file__).resolve().parent / path
        try:
            return path.resolve()
        except Exception:
            return path.absolute()

    def _select_yolo_model(self):
        current_path = self._resolve_yolo_model_path()
        if current_path.exists():
            start_location = str(current_path)
        elif current_path.parent.exists():
            start_location = str(current_path.parent)
        else:
            start_location = str(Path(__file__).resolve().parent)
        selected_path, _selected_filter = QFileDialog.getOpenFileName(self, 'Seleccionar modelo YOLO', start_location, 'Modelo YOLO / PyTorch (*.pt);;Todos los archivos (*)')
        if not selected_path:
            return
        selected = Path(selected_path)
        if selected.suffix.lower() != '.pt':
            self._show_msg('Selecciona un archivo de modelo con extensión .pt.', is_error=True)
            return
        self.input_camera_model.setText(str(selected.resolve()))
        self._save_vision_config(silent=True)
        self.lbl_camera_status.setText(f'Modelo YOLO seleccionado: {selected.name}')
        self._show_msg(f'Modelo YOLO seleccionado: {selected.name}')

    def _start_camera_stream(self):
        if self.vision_thread is not None:
            if self.vision_thread.isRunning():
                self._camera_restart_requested = True
                self._stop_camera_stream(clear_preview=False)
                self.lbl_camera_status.setText('Camara: reiniciando de forma segura...')
                return
            self.vision_thread.deleteLater()
            self.vision_thread = None
        self._save_vision_config(silent=True)
        use_yolo = self.chk_camera_yolo.isChecked()
        model_path = self._resolve_yolo_model_path()
        if use_yolo:
            if model_path.suffix.lower() != '.pt':
                self._show_msg("El modelo YOLO debe ser un archivo .pt. Usa 'Seleccionar .pt' para elegirlo.", is_error=True)
                return
            if not model_path.is_file():
                self.lbl_camera_status.setText('Camara: no se encontro el modelo YOLO')
                self._show_msg(f"No se encontro el modelo YOLO:\n{model_path}\nUsa el boton 'Seleccionar .pt' para elegir el archivo correcto.", is_error=True)
                return
            self.input_camera_model.setText(str(model_path))
            self._save_vision_config(silent=True)
        camera_index = 1
        stream_url = getattr(self, 'camera_source', None).text().strip() if hasattr(self, 'camera_source') else str(camera_index)
        resolution = parse_camera_resolution(self.combo_camera_resolution.currentText())
        side = max(128, min(int(resolution[0]), int(resolution[1])))
        resolution = (side, side)
        rotation = 0
        yolo_stride = self.combo_yolo_stride.currentIndex() + 1
        self._reset_detection_runtime()
        self.active_detection_id = None
        worker = VisionThread(stream_url=stream_url, output_size=resolution, max_objects=self.slider_camera_objects.value(), use_yolo=use_yolo, model_path=str(model_path), mirror_horizontal=self.chk_camera_mirror.isChecked(), yolo_stride=yolo_stride, confidence=self.YOLO_CONFIDENCE, parent=self)
        self.vision_thread = worker
        self._camera_stop_pending = False
        self._camera_clear_preview_on_stop = False
        self._camera_restart_requested = False
        worker.frame_ready.connect(self._update_camera_frame)
        worker.detections_ready.connect(self._update_detections)
        worker.status_ready.connect(lambda value: self._update_runtime_option('status', value))
        worker.fps_ready.connect(lambda value: self._update_runtime_option('fps', value))
        worker.finished.connect(self._camera_thread_finished)
        worker.set_yolo_paused(False)
        self.lbl_yolo_runtime.setText(f'YOLO: activo con fuente configurada · confianza mínima {self.YOLO_CONFIDENCE * 100:.0f}%')
        self.camera_preview.setPixmap(QPixmap())
        self.camera_preview.setText('Conectando fuente configurada...')
        self.lbl_camera_status.setText('Cámara: abriendo fuente configurada')
        self.btn_camera_connect.setEnabled(False)
        self.btn_camera_stop.setEnabled(True)
        worker.start()

    def _reset_camera_runtime_state(self, clear_preview=True):
        self._reset_detection_runtime()
        self.active_detection_id = None
        self.locked_detection = None
        if hasattr(self, 'lbl_yolo_runtime'):
            self.lbl_yolo_runtime.setText('YOLO: cámara detenida')
        if hasattr(self, 'lbl_detected_objects'):
            self.lbl_detected_objects.setText('Estables: 0')
        if hasattr(self, 'lbl_camera_fps'):
            self.lbl_camera_fps.setText('FPS: 0.0')
        if hasattr(self, 'lbl_vision_target'):
            self.lbl_vision_target.setText('Objetivo: sin detección estable')
        if clear_preview and hasattr(self, 'camera_preview'):
            self.camera_preview.setPixmap(QPixmap())
            self.camera_preview.setText('Preview de cámara\nSin conexión')
            self.lbl_camera_status.setText('Cámara: desconectada')
        self._redraw_robot()

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
        return {'version': 10, 'camera': {'source': 'opencv_index', 'camera_index': 1, 'resolution': '320x320', 'mirror_horizontal': False, 'invert_image_x': False, 'invert_image_y': False, 'model_path': 'best.pt', 'yolo_stride': 1, 'max_objects': 4}, 'drop_zone': {'x_mm': self.DROP_ZONE_DEFAULT[0], 'y_mm': self.DROP_ZONE_DEFAULT[1], 'z_mm': self.DROP_ZONE_DEFAULT[2]}}

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
        return {'version': 10, 'camera': {'source': 'opencv_index', 'camera_index': 1, 'resolution': self.combo_camera_resolution.currentText(), 'mirror_horizontal': self.chk_camera_mirror.isChecked(), 'invert_image_x': self.chk_camera_axis_x_invert.isChecked(), 'invert_image_y': self.chk_camera_axis_y_invert.isChecked(), 'model_path': self.input_camera_model.text().strip() or 'best.pt', 'yolo_stride': 1, 'max_objects': int(self.slider_camera_objects.value())}, 'drop_zone': {'x_mm': self._number(self.input_drop_x, self.DROP_ZONE_DEFAULT[0], -280, 280), 'y_mm': self._number(self.input_drop_y, self.DROP_ZONE_DEFAULT[1], 35, 300), 'z_mm': self._number(self.input_drop_z, self.DROP_ZONE_DEFAULT[2], 10, 100)}}

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
                loaded = json.loads(self.vision_config_path.read_text(encoding='utf-8'))
                for section in ('camera', 'drop_zone'):
                    if isinstance(loaded.get(section), dict):
                        config[section].update(loaded[section])
            except Exception as exc:
                if not silent:
                    self._show_msg(f'JSON inválido; se cargaron valores seguros: {exc}', is_error=True)
        elif create_if_missing:
            try:
                self.vision_config_path.write_text(json.dumps(config, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
            except Exception:
                pass
        cam, drop = (config['camera'], config['drop_zone'])
        resolution = str(cam.get('resolution', '320x320'))
        if self.combo_camera_resolution.findText(resolution) >= 0:
            self.combo_camera_resolution.setCurrentText(resolution)
        self.chk_camera_mirror.setChecked(bool(cam.get('mirror_horizontal', False)))
        self.chk_camera_axis_x_invert.setChecked(bool(cam.get('invert_image_x', False)))
        self.chk_camera_axis_y_invert.setChecked(bool(cam.get('invert_image_y', False)))
        self.input_camera_model.setText(str(cam.get('model_path', 'best.pt')))
        self.combo_yolo_stride.setCurrentIndex(0)
        self.slider_camera_objects.setValue(int(self._number(cam.get('max_objects'), 4, 1, 8)))
        values = (self._number(drop.get('x_mm'), self.DROP_ZONE_DEFAULT[0], -280, 280), self._number(drop.get('y_mm'), self.DROP_ZONE_DEFAULT[1], 35, 300), self._number(drop.get('z_mm'), self.DROP_ZONE_DEFAULT[2], 10, 100))
        for widget, value in zip((self.input_drop_x, self.input_drop_y, self.input_drop_z), values):
            widget.setText(f'{value:g}')
        self.drop_zone_xyz = np.array(values, dtype=float)
        self.lbl_drop_zone.setText(f'Destino actual: X={values[0]:.0f}, Y={values[1]:.0f}, Z={values[2]:.0f} mm')
        self.vision_motion_speed_dps = self.AUTOMATIC_SPEED_DEFAULT_DPS
        self.recovery_speed_dps = self.RECOVERY_SPEED_DPS
        self._update_overhead_alignment_label()
        if not silent:
            self._show_msg(f'Configuración leída desde {self.vision_config_path.name}.')
        return True

    def _update_overhead_alignment_label(self):
        if not hasattr(self, 'lbl_overhead_alignment'):
            return
        tool_pitch = self._current_tool_pitch_deg()
        optical_pitch = tool_pitch
        error = abs((optical_pitch - 90.0 + 180.0) % 360.0 - 180.0)
        if error <= 4.0:
            self.lbl_overhead_alignment.setText(f'Vista superior confirmada · herramienta={tool_pitch:.1f}° · cámara={optical_pitch:.1f}°')
            self.lbl_overhead_alignment.setStyleSheet('color: #22C55E; font-weight: bold;')
        else:
            self.lbl_overhead_alignment.setText(f'Vista no cenital · herramienta={tool_pitch:.1f}° · cámara={optical_pitch:.1f}° · error={error:.1f}°')
            self.lbl_overhead_alignment.setStyleSheet('color: #F59E0B; font-weight: bold;')

    def _apply_drop_zone(self):
        try:
            new_zone = np.array([float(self.input_drop_x.text()), float(self.input_drop_y.text()), float(self.input_drop_z.text())])
        except ValueError:
            self._show_msg('La zona de entrega requiere valores numéricos X, Y y Z.', is_error=True)
            return
        if not self._workspace_point_is_valid(new_zone, include_table=True):
            self._show_msg('La zona de entrega queda fuera del workspace seguro.', is_error=True)
            return
        self.drop_zone_xyz = new_zone
        self.lbl_drop_zone.setText(f'Destino actual: X={new_zone[0]:.0f}, Y={new_zone[1]:.0f}, Z={new_zone[2]:.0f} mm')
        self._redraw_robot()
        self._save_vision_config(silent=True)
        self._show_msg(f'Zona de entrega actualizada: ({new_zone[0]:.0f}, {new_zone[1]:.0f}, {new_zone[2]:.0f}) mm')

    def _toggle_observe_mode(self, checked: bool):
        self.observe_mode_active = bool(checked)
        if checked:
            self._reset_path(redraw=False)
            if self.vision_state != 'IDLE':
                self._toggle_vision()
            self.btn_observe_mode.setText('Detener observación')
            self._show_msg('Modo observar activo: el live feed muestra detecciones sin mover el robot.')
        else:
            self.btn_observe_mode.setText('Modo observar')
            self._show_msg('Modo observar detenido.')

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

    def _pickup_z_mm(self):
        return float(self.BLOCK_PICK_Z_MM)

    def _camera_to_gripper_offset_xy(self):
        return np.array([self.CAMERA_TO_GRIPPER_X_MM, self.CAMERA_TO_GRIPPER_Y_MM], dtype=float)

    def _final_radial_pick_trim_mm(self):
        return float(self.FINAL_RADIAL_PICK_TRIM_MM)

    def _radial_compensation_anchor_mm(self, anchor):
        return float({'q1': self.RADIAL_COMP_Q1_MM, 'center': self.RADIAL_COMP_CENTER_MM, 'q2': self.RADIAL_COMP_Q2_MM}[str(anchor)])

    def _angle_dependent_radial_compensation_mm(self, base_deg):
        angle = float(np.clip(base_deg, self.RADIAL_COMP_Q1_ANGLE_DEG, self.RADIAL_COMP_Q2_ANGLE_DEG))
        return float(np.interp(angle, [self.RADIAL_COMP_Q1_ANGLE_DEG, self.RADIAL_COMP_CENTER_ANGLE_DEG, self.RADIAL_COMP_Q2_ANGLE_DEG], [self._radial_compensation_anchor_mm('q1'), self._radial_compensation_anchor_mm('center'), self._radial_compensation_anchor_mm('q2')]))

    def _xy_compensation_anchor_mm(self, anchor):
        return np.array({'q1': [self.PICK_XY_COMP_Q1_X_MM, self.PICK_XY_COMP_Q1_Y_MM], 'center': [self.PICK_XY_COMP_CENTER_X_MM, self.PICK_XY_COMP_CENTER_Y_MM], 'q2': [self.PICK_XY_COMP_Q2_X_MM, self.PICK_XY_COMP_Q2_Y_MM]}[str(anchor)], dtype=float)

    def _angle_dependent_xy_compensation_mm(self, base_deg):
        angle = float(np.clip(base_deg, self.RADIAL_COMP_Q1_ANGLE_DEG, self.RADIAL_COMP_Q2_ANGLE_DEG))
        angles = [self.RADIAL_COMP_Q1_ANGLE_DEG, self.RADIAL_COMP_CENTER_ANGLE_DEG, self.RADIAL_COMP_Q2_ANGLE_DEG]
        anchors = [self._xy_compensation_anchor_mm('q1'), self._xy_compensation_anchor_mm('center'), self._xy_compensation_anchor_mm('q2')]
        return np.array([np.interp(angle, angles, [point[0] for point in anchors]), np.interp(angle, angles, [point[1] for point in anchors])], dtype=float)

    def _angle_edge_xy_max_mm(self):
        return float(self.ANGLE_EDGE_XY_MAX_MM)

    def _angle_edge_xy_compensation_mm(self, base_deg):
        angle = float(np.clip(base_deg, self.RADIAL_COMP_Q1_ANGLE_DEG, self.RADIAL_COMP_Q2_ANGLE_DEG))
        half_span = max(1e-06, self.RADIAL_COMP_CENTER_ANGLE_DEG - self.RADIAL_COMP_Q1_ANGLE_DEG)
        fraction = float(np.clip(abs(angle - self.RADIAL_COMP_CENTER_ANGLE_DEG) / half_span, 0.0, 1.0))
        magnitude = self._angle_edge_xy_max_mm() * fraction
        angle_rad = math.radians(angle)
        return magnitude * np.array([math.cos(angle_rad), math.sin(angle_rad)], dtype=float)

    def _set_vision_state(self, state: str, detail: str=''):
        previous_state = self.vision_state
        self.vision_state = str(state)
        if self.vision_state == 'SCAN_DWELL':
            self.vision_timer = int(self.SCAN_DWELL_TICKS)
        if self.vision_state == 'SCAN_DWELL' and previous_state == 'MOVE_TO_SCAN_START' and (not self.vision_detection_armed):
            self.vision_detection_armed = True
            self._reset_detection_runtime()
            self._set_yolo_paused(False, 'primer punto alcanzado; detección en vivo habilitada')
            detail = (detail + ' · ' if detail else '') + 'visión habilitada'
        if hasattr(self, 'lbl_vision_phase'):
            suffix = f' · {detail}' if detail else ''
            self.lbl_vision_phase.setText(f'Fase: {self.vision_state}{suffix}')

    def _set_yolo_paused(self, paused: bool, reason: str=''):
        if self.vision_thread is not None:
            try:
                self.vision_thread.set_yolo_paused(paused, reason)
            except Exception:
                pass
        if hasattr(self, 'lbl_yolo_runtime'):
            if paused:
                self.lbl_yolo_runtime.setText(f"YOLO: pausado · {reason or 'objetivo bloqueado'}")
            else:
                suffix = f' · {reason}' if reason else ''
                self.lbl_yolo_runtime.setText(f'YOLO: detección en vivo{suffix}')

    def _allowed_tetris_classes(self):
        values = [self._normalize_tetris_class(name) for name in self.ALLOWED_CLASSES]
        return {value for value in values if value}

    def _normalize_tetris_class(self, class_name):
        name = str(class_name or '').strip()
        compact = name.lower().replace('í', 'i').replace(' ', '').replace('_', '')
        aliases = {'linea': 'Línea', 'line': 'Línea', 'i': 'Línea', 'straight': 'Línea', 'l': 'L', 'pieza-l': 'L', 'lpiece': 'L', 't': 'T', 'pieza-t': 'T', 'tpiece': 'T'}
        return aliases.get(compact, name)

    def _clear_scene_objects(self):
        self.scene_objects.clear()
        self._next_scene_object_id = 1
        self.locked_detection = None
        self._update_scene_object_table()
        self._redraw_robot()
        self._show_msg('Mapa de objetos YOLO limpiado.')

    def _register_scene_object(self, detection):
        item = dict(detection)
        item['class'] = self._normalize_tetris_class(item.get('class', 'objeto'))
        now = time.monotonic()
        cx = float(item.get('cx', 0.0))
        cy = float(item.get('cy', 0.0))
        frame_w = max(2.0, float(item.get('frame_w', 416.0)))
        frame_h = max(2.0, float(item.get('frame_h', 416.0)))
        anchor_xyz = np.array(self.current_xyz, dtype=float)
        anchor_xyz[2] = self._pickup_z_mm()
        match_id = None
        active_id = self.vision_centering_scene_id
        if active_id in self.scene_objects:
            active = self.scene_objects[active_id]
            if active.get('class') == item['class'] and active.get('status') != 'Entregado':
                match_id = active_id
        if match_id is None:
            candidates = []
            pixel_gate = max(55.0, 0.22 * min(frame_w, frame_h))
            for scene_id, stored in self.scene_objects.items():
                if stored.get('class') != item['class'] or stored.get('status') == 'Entregado':
                    continue
                if now - float(stored.get('last_seen', 0.0)) > 3.0:
                    continue
                old_cx = float(stored.get('last_cx', cx))
                old_cy = float(stored.get('last_cy', cy))
                pixel_distance = math.hypot(cx - old_cx, cy - old_cy)
                if pixel_distance <= pixel_gate:
                    candidates.append((pixel_distance, scene_id))
            if candidates:
                match_id = min(candidates)[1]
        if match_id is None:
            match_id = self._next_scene_object_id
            self._next_scene_object_id += 1
            self.scene_objects[match_id] = {'scene_id': match_id, 'class': item['class'], 'xyz': anchor_xyz.copy(), 'confidence': float(item.get('confidence', 0.0)), 'status': 'Visible', 'pickable': None, 'position_confirmed': False, 'last_seen': now, 'last_cx': cx, 'last_cy': cy, 'frame_w': frame_w, 'frame_h': frame_h}
        else:
            stored = self.scene_objects[match_id]
            stored['xyz'] = anchor_xyz.copy()
            stored['confidence'] = max(float(stored.get('confidence', 0.0)), float(item.get('confidence', 0.0)))
            stored['last_seen'] = now
            stored['last_cx'] = cx
            stored['last_cy'] = cy
            stored['frame_w'] = frame_w
            stored['frame_h'] = frame_h
            if stored.get('status') not in {'Bloqueado', 'En agarre', 'En traslado', 'Entregado'}:
                stored['status'] = 'Visible'
        if len(self.scene_objects) > 200:
            removable = sorted((obj for obj in self.scene_objects.values() if obj['scene_id'] != self.active_detection_id and obj.get('status') != 'Entregado'), key=lambda obj: float(obj.get('last_seen', 0.0)))
            for old in removable[:len(self.scene_objects) - 200]:
                self.scene_objects.pop(old['scene_id'], None)
        item['scene_id'] = match_id
        item['workspace_xyz'] = anchor_xyz.copy()
        item['workspace_valid'] = True
        self._scene_table_dirty = True
        return item

    def _update_scene_object_table(self):
        if not hasattr(self, 'table_scene_objects'):
            return
        records = sorted(self.scene_objects.values(), key=lambda obj: int(obj['scene_id']))
        self.table_scene_objects.setRowCount(len(records))
        classes = []
        for row, obj in enumerate(records):
            xyz = np.array(obj.get('xyz', [0.0, 0.0, 0.0]), dtype=float)
            confirmed = bool(obj.get('position_confirmed', False))
            xyz_text = (f'{xyz[0]:.1f}', f'{xyz[1]:.1f}', f'{xyz[2]:.1f}') if confirmed else ('—', '—', '—')
            status = str(obj.get('status', 'Visible'))
            if not confirmed and 'sin XYZ' not in status:
                status += ' · sin XYZ'
            values = (obj['scene_id'], obj.get('class', 'objeto'), *xyz_text, f"{float(obj.get('confidence', 0.0)):.2f}", status)
            for column, value in enumerate(values):
                self.table_scene_objects.setItem(row, column, QTableWidgetItem(str(value)))
            classes.append(str(obj.get('class', 'objeto')))
        class_text = ', '.join(dict.fromkeys(classes)) if classes else 'ninguna'
        self.lbl_seen_classes.setText(f'Objetos detectados: {len(records)} · clases: {class_text} · XYZ aparece después del centrado visual')
        if self.vision_state != 'IDLE' and hasattr(self, 'btn_vision'):
            self.btn_vision.setText(f'Detener búsqueda · {len(records)} detectados')
        self._scene_table_dirty = False
        self._last_scene_table_update = time.monotonic()

    def _current_tool_pitch_deg(self):
        theta = self.robot.convert_angles(self.current_physical_angles)
        phi = theta[1] - np.pi / 2.0 + theta[2] + theta[3]
        return float(np.rad2deg(phi))

    def _pitch_candidates(self, preferred_pitch_deg):
        preferred = float(preferred_pitch_deg)
        raw = [preferred, self._current_tool_pitch_deg(), 90.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 85.0, 80.0, 75.0, 70.0, 65.0, 60.0, 55.0, 50.0, 45.0, 40.0]
        unique = []
        for value in raw:
            value = float(np.clip(value, 35.0, 135.0))
            if all((abs(value - existing) > 0.25 for existing in unique)):
                unique.append(value)
        unique.sort(key=lambda value: abs(value - preferred))
        return unique

    def _solve_pose_at_exact_pitch(self, target_xyz, tool_pitch_deg, reference_angles=None, margin_override=None):
        target = np.array(target_xyz, dtype=float)
        if not np.all(np.isfinite(target)):
            return (False, None, 'POSE:NON_FINITE')
        x, y, z = [float(v) for v in target]
        radius = math.hypot(x, y)
        if radius < 1e-06 or y < -1e-06:
            return (False, None, 'POSE:BASE_DOMAIN')
        q0 = math.atan2(y, x)
        phi = math.radians(float(tool_pitch_deg))
        a0, a1, a2, a3 = [float(v) for v in self.robot.a]
        d0 = float(self.robot.d[0])
        wrist_r = radius - a0 - a3 * math.cos(phi)
        wrist_down = d0 - z - a3 * math.sin(phi)
        c2 = (wrist_r ** 2 + wrist_down ** 2 - a1 ** 2 - a2 ** 2) / (2.0 * a1 * a2)
        if c2 < -1.000001 or c2 > 1.000001:
            return (False, None, 'POSE:UNREACHABLE')
        c2 = float(np.clip(c2, -1.0, 1.0))
        margin = float(margin_override) if margin_override is not None else self.JOINT_MARGIN_DEG
        reference = np.array(reference_angles, dtype=float) if reference_angles is not None else np.array(self.current_physical_angles, dtype=float)
        candidates = []
        for sign in (1.0, -1.0):
            q2 = sign * math.acos(c2)
            q1 = math.atan2(wrist_down, wrist_r) - math.atan2(a2 * math.sin(q2), a1 + a2 * math.cos(q2))
            q3 = phi - q1 - q2
            theta_input = np.array([q0, q1 + np.pi / 2.0, q2, q3], dtype=float)
            servo = np.array(self.robot.convert_angles(theta_input, to_servo=True), dtype=float)
            if not np.all(np.isfinite(servo)):
                continue
            if np.any(servo < margin) or np.any(servo > 180.0 - margin):
                continue
            final_pos, matrices = self.robot.compute_fk(servo)
            position_error = float(np.linalg.norm(np.array(final_pos) - target))
            tool_axis = np.array(matrices[-1][0:3, 0], dtype=float)
            radial_axis = np.array([math.cos(q0), math.sin(q0), 0.0], dtype=float)
            solved_pitch = math.degrees(math.atan2(-tool_axis[2], np.dot(tool_axis, radial_axis)))
            pitch_error = abs((solved_pitch - tool_pitch_deg + 180.0) % 360.0 - 180.0)
            distance_from_reference = float(np.linalg.norm(servo - reference))
            if position_error <= 2.0 and pitch_error <= 1.25:
                candidates.append((distance_from_reference, servo))
        if not candidates:
            return (False, None, 'POSE:JOINT_LIMIT_OR_ORIENTATION')
        candidates.sort(key=lambda record: record[0])
        return (True, candidates[0][1], f'OK:PITCH={float(tool_pitch_deg):.1f}')

    def _solve_position_only(self, target_xyz, reference_angles=None, lock_base=False, position_tolerance_mm=3.0, joint_margin_deg=None):
        target = np.array(target_xyz, dtype=float)
        if not np.all(np.isfinite(target)):
            return (False, None, 'POSE:NON_FINITE')
        if not lock_base and np.linalg.norm(target) > self.robot.max_reach * 0.98:
            return (False, None, 'POSE:UNREACHABLE')
        locked_base = None
        radial = None
        reference = np.array(reference_angles if reference_angles is not None else self.current_physical_angles, dtype=float)
        if lock_base:
            if self.vision_locked_base_servo_deg is None:
                return (False, None, 'POSE:BASE_NOT_LOCKED')
            locked_base = float(self.vision_locked_base_servo_deg)
            q0 = math.radians(locked_base)
            radial = np.array([math.cos(q0), math.sin(q0), 0.0], dtype=float)
            tangential = np.array([-radial[1], radial[0], 0.0], dtype=float)
            if abs(float(np.dot(target, tangential))) > 2.0:
                return (False, None, 'POSE:TARGET_OUTSIDE_LOCKED_RADIAL_LINE')
            reference[0] = locked_base
        theta = np.array(self.robot.convert_angles(reference), dtype=float)
        locked_theta0 = float(theta[0])
        best_theta = theta.copy()
        best_error = float('inf')
        joints = range(3, 0, -1) if lock_base else range(3, -1, -1)
        for _ in range(100 if lock_base else 80):
            for joint in joints:
                end_effector, transforms = self.robot.compute_fk_dh(theta)
                error = float(np.linalg.norm(end_effector - target))
                if error < best_error:
                    best_error, best_theta = (error, theta.copy())
                if error < 0.8:
                    break
                pivot, axis = (transforms[joint][0:3, 3], transforms[joint][0:3, 2])
                current_vec, target_vec = (end_effector - pivot, target - pivot)
                current_vec -= np.dot(current_vec, axis) * axis
                target_vec -= np.dot(target_vec, axis) * axis
                current_mag, target_mag = (float(np.linalg.norm(current_vec)), float(np.linalg.norm(target_vec)))
                if current_mag < 1e-06 or target_mag < 1e-06:
                    continue
                current_vec /= current_mag
                target_vec /= target_mag
                cosine = float(np.clip(np.dot(current_vec, target_vec), -1.0, 1.0))
                sine = float(np.dot(axis, np.cross(current_vec, target_vec)))
                theta[joint] = np.clip(theta[joint] + float(np.clip(math.atan2(sine, cosine), -0.12, 0.12)), self.robot.limit_min[joint], self.robot.limit_max[joint])
                if lock_base:
                    theta[0] = locked_theta0
        final_pos, _ = self.robot.compute_fk_dh(theta)
        final_error = float(np.linalg.norm(final_pos - target))
        if best_error < final_error:
            theta, final_error = (best_theta, best_error)
        if lock_base:
            theta[0] = locked_theta0
        servo = np.array(self.robot.convert_angles(theta, to_servo=True), dtype=float)
        if lock_base:
            servo[0] = locked_base
            margin = self.JOINT_MARGIN_DEG if joint_margin_deg is None else max(0.25, float(joint_margin_deg))
            valid = final_error <= float(position_tolerance_mm) and np.all(np.isfinite(servo)) and np.all(servo >= margin) and np.all(servo <= 180.0 - margin)
            if valid:
                _, matrices = self.robot.compute_fk(servo)
                tool_axis = np.array(matrices[-1][0:3, 0], dtype=float)
                solved_pitch = math.degrees(math.atan2(-tool_axis[2], float(np.dot(tool_axis, radial))))
                if 30.0 <= solved_pitch <= 145.0:
                    return (True, servo, f'OK:BASE_LOCKED_FREE_PITCH={solved_pitch:.1f}')
            return (False, None, f'POSE:LOCKED_BASE_POSITION_ERROR={final_error:.1f}')
        if final_error <= 3.0 and np.all(np.isfinite(servo)) and np.all(servo >= -0.25) and np.all(servo <= 180.25):
            return (True, np.clip(servo, 0.0, 180.0), 'OK:POSITION_ONLY')
        return (False, None, f'POSE:POSITION_ERROR={final_error:.1f}')

    def _solve_pose(self, target_xyz, pitch_deg, reference_angles=None, position_tolerance_mm=3.0, allow_pitch_fallback=True, joint_margin_deg=None, lock_base=False):
        pitches = self._pitch_candidates(pitch_deg) if allow_pitch_fallback else [float(pitch_deg)]
        reference = np.array(reference_angles if reference_angles is not None else self.current_physical_angles, dtype=float)
        if not lock_base:
            failures = []
            for candidate_pitch in pitches:
                ok, servo, reason = self._solve_pose_at_exact_pitch(target_xyz, candidate_pitch, reference_angles=reference)
                if ok:
                    return (True, servo, reason)
                failures.append(reason)
            if allow_pitch_fallback:
                ok, servo, reason = self._solve_position_only(target_xyz, reference_angles=reference)
                if ok:
                    servo = np.array(servo, dtype=float)
                    if np.all(servo >= self.JOINT_MARGIN_DEG) and np.all(servo <= 180.0 - self.JOINT_MARGIN_DEG):
                        return (True, servo, reason)
                    reason = 'POSE:POSITION_ONLY_OUTSIDE_JOINT_MARGIN'
                failures.append(reason)
            return (False, None, failures[-1] if failures else 'POSE:UNREACHABLE')
        if self.vision_locked_base_servo_deg is None:
            return (False, None, 'POSE:BASE_NOT_LOCKED')
        target = np.array(target_xyz, dtype=float)
        locked_base = float(self.vision_locked_base_servo_deg)
        q0 = math.radians(locked_base)
        radial = np.array([math.cos(q0), math.sin(q0)], dtype=float)
        tangential = np.array([-radial[1], radial[0]], dtype=float)
        if abs(float(np.dot(target[:2], tangential))) > 2.0:
            return (False, None, 'POSE:TARGET_OUTSIDE_LOCKED_RADIAL_LINE')
        candidates = []
        last_reason = 'POSE:UNREACHABLE'
        for candidate_pitch in pitches:
            ok, servo, reason = self._solve_pose_at_exact_pitch(target, float(candidate_pitch), reference_angles=reference, margin_override=joint_margin_deg)
            if not ok:
                last_reason = reason
                continue
            servo = np.array(servo, dtype=float)
            if abs(float(servo[0]) - locked_base) > 0.4:
                last_reason = 'POSE:BASE_DRIFT'
                continue
            servo[0] = locked_base
            final_pos, _ = self.robot.compute_fk(servo)
            error = float(np.linalg.norm(np.array(final_pos, dtype=float) - target))
            if not np.all(np.isfinite(final_pos)) or error > float(position_tolerance_mm):
                last_reason = f'POSE:LOCKED_BASE_ERROR={error:.1f}'
                continue
            score = abs(float(candidate_pitch) - float(pitch_deg)) * 0.15 + float(np.linalg.norm(servo - reference))
            candidates.append((score, servo, float(candidate_pitch)))
        if allow_pitch_fallback:
            ok, servo, reason = self._solve_position_only(target, reference_angles=reference, lock_base=True, position_tolerance_mm=position_tolerance_mm, joint_margin_deg=joint_margin_deg)
            if ok:
                servo = np.array(servo, dtype=float)
                _, matrices = self.robot.compute_fk(servo)
                axis = np.array(matrices[-1][0:3, 0], dtype=float)
                free_pitch = math.degrees(math.atan2(-axis[2], float(np.dot(axis[:2], radial))))
                candidates.append((1.5 + float(np.linalg.norm(servo - reference)), servo, free_pitch))
            else:
                last_reason = reason
        if not candidates:
            return (False, None, last_reason)
        _, servo, solved_pitch = min(candidates, key=lambda item: item[0])
        return (True, servo, f'OK:BASE_LOCKED:PITCH={solved_pitch:.1f}')

    def _axis_waypoints(self, start_xyz, target_xyz, axis_order=('z', 'x', 'y'), step_mm=None):
        start = np.array(start_xyz, dtype=float)
        target = np.array(target_xyz, dtype=float)
        step = float(step_mm or self.VISION_STEP_MM)
        axis_index = {'x': 0, 'y': 1, 'z': 2}
        cursor = start.copy()
        points = []
        for axis_name in axis_order:
            idx = axis_index[axis_name]
            delta = float(target[idx] - cursor[idx])
            count = max(1, int(math.ceil(abs(delta) / step))) if abs(delta) > 1e-06 else 0
            for part in range(1, count + 1):
                point = cursor.copy()
                point[idx] = cursor[idx] + delta * (part / count)
                points.append(point)
            cursor[idx] = target[idx]
        return points

    def _begin_visual_base_segment(self, direction, step_deg, label):
        if self.vision_motion_queue:
            return True
        reference = np.array(self.current_physical_angles, dtype=float)
        target = reference.copy()
        target[0] += float(direction) * float(step_deg)
        margin = self.JOINT_MARGIN_DEG
        if target[0] < margin or target[0] > 180.0 - margin:
            return False
        final_pos, _matrices = self.robot.compute_fk(target)
        final_pos = np.array(final_pos, dtype=float)
        if not np.all(np.isfinite(final_pos)) or not self._workspace_point_is_valid(final_pos, include_table=True):
            return False
        self.vision_motion_queue = [{'xyz': final_pos, 'servo': target, 'pitch': self._current_tool_pitch_deg(), 'visual_centering': True, 'base_only': True}]
        self.vision_segment_next_state = 'SCAN_DWELL'
        self.vision_segment_label = str(label)
        self.lbl_vision_phase.setText(f'Fase: {self.vision_state} · {label} · solo servo base')
        return True

    def _begin_visual_radial_segment(self, target_radius, pitch_deg, label):
        if self.vision_motion_queue:
            return True
        if self.vision_locked_base_servo_deg is None:
            return False
        q0 = math.radians(float(self.vision_locked_base_servo_deg))
        target = np.array([float(target_radius) * math.cos(q0), float(target_radius) * math.sin(q0), float(self.current_xyz[2])])
        if not self._workspace_point_is_valid(target, include_table=True):
            return False
        reference = np.array(self.current_physical_angles, dtype=float)
        ok, servo, reason = self._solve_pose(target, float(pitch_deg), reference_angles=reference, position_tolerance_mm=4.0, allow_pitch_fallback=True, joint_margin_deg=self.VISION_RADIAL_SERVO_MARGIN_DEG, lock_base=True)
        if not ok:
            return False
        servo = np.array(servo, dtype=float)
        joint_delta = float(np.max(np.abs(servo - reference)))
        if joint_delta > 35.0:
            return False
        solved_pitch = float(pitch_deg)
        try:
            solved_pitch = float(reason.rsplit('=', 1)[-1])
        except Exception:
            pass
        self.vision_motion_queue = [{'xyz': target.copy(), 'servo': servo, 'pitch': solved_pitch, 'visual_centering': True, 'base_locked': True, 'joint_margin_deg': self.VISION_RADIAL_SERVO_MARGIN_DEG}]
        self.vision_segment_next_state = 'SCAN_DWELL'
        self.vision_segment_label = str(label)
        self.lbl_vision_phase.setText(f'Fase: {self.vision_state} · {label} · base={self.vision_locked_base_servo_deg:.1f}° · pitch adaptativo')
        return True

    def _begin_locked_base_segment(self, target_xyz, pitch_deg, next_state, label, step_mm=None, max_joint_delta_deg=42.0):
        if self.vision_motion_queue:
            return True
        if self.vision_locked_base_servo_deg is None:
            self._trigger_safe_recovery(f'La base no estaba bloqueada durante {label}')
            return False
        target = np.array(target_xyz, dtype=float)
        start = np.array(self.current_xyz, dtype=float)
        q0 = math.radians(float(self.vision_locked_base_servo_deg))
        radial = np.array([math.cos(q0), math.sin(q0)], dtype=float)
        start_r = float(np.dot(start[:2], radial))
        target_r = float(np.dot(target[:2], radial))
        step = float(step_mm or self.VISION_STEP_MM)
        points = []
        radial_delta = target_r - start_r
        radial_count = max(1, int(math.ceil(abs(radial_delta) / step))) if abs(radial_delta) > 1e-06 else 0
        for part in range(1, radial_count + 1):
            radius = start_r + radial_delta * (part / radial_count)
            points.append(np.array([radius * radial[0], radius * radial[1], start[2]], dtype=float))
        z_start = float(start[2])
        z_delta = float(target[2] - z_start)
        z_count = max(1, int(math.ceil(abs(z_delta) / step))) if abs(z_delta) > 1e-06 else 0
        for part in range(1, z_count + 1):
            z_value = z_start + z_delta * (part / z_count)
            points.append(np.array([target_r * radial[0], target_r * radial[1], z_value], dtype=float))
        reference = np.array(self.current_physical_angles, dtype=float)
        queue = []
        ok, servo, reason = self._solve_pose(start, float(pitch_deg), reference_angles=reference, allow_pitch_fallback=True, lock_base=True)
        if not ok:
            self._trigger_safe_recovery(f'No se pudo conservar la base durante {label}: {reason}')
            return False
        if float(np.max(np.abs(servo - reference))) > 0.35:
            queue.append({'xyz': start.copy(), 'servo': servo, 'pitch': float(pitch_deg), 'base_locked': True})
            reference = np.array(servo, dtype=float)
        for point in points:
            if not self._workspace_point_is_valid(point, include_table=True):
                self._trigger_safe_recovery(f'Waypoint fuera de workspace durante {label}: {point.round(1).tolist()}')
                return False
            ok, servo, reason = self._solve_pose(point, float(pitch_deg), reference_angles=reference, allow_pitch_fallback=True, lock_base=True)
            if not ok:
                self._trigger_safe_recovery(f'No existe pose con base bloqueada para {label}: {reason}')
                return False
            if float(np.max(np.abs(servo - reference))) > float(max_joint_delta_deg):
                self._trigger_safe_recovery(f'Salto articular excesivo con base bloqueada en {label}')
                return False
            queue.append({'xyz': point.copy(), 'servo': servo, 'pitch': float(pitch_deg), 'base_locked': True})
            reference = np.array(servo, dtype=float)
        if not queue:
            self._set_vision_state(next_state, f'sin desplazamiento: {label}')
            return True
        self.vision_motion_queue = queue
        self.vision_segment_next_state = str(next_state)
        self.vision_segment_label = str(label)
        self.lbl_vision_phase.setText(f'Fase: {self.vision_state} · {label} · base fija {self.vision_locked_base_servo_deg:.1f}° · {len(queue)} puntos')
        return True

    def _begin_vision_segment(self, target_xyz, pitch_deg, next_state, axis_order, label, step_mm=None, max_joint_delta_deg=42.0):
        if self.vision_motion_queue:
            return True
        target = np.array(target_xyz, dtype=float)
        points = self._axis_waypoints(self.current_xyz, target, axis_order=tuple(axis_order), step_mm=step_mm)
        if not points:
            points = [target.copy()]
        queue = []
        reference = np.array(self.current_physical_angles, dtype=float)
        ok, servo, reason = self._solve_pose(self.current_xyz, pitch_deg, reference_angles=reference)
        if ok and np.max(np.abs(servo - reference)) > 0.35:
            queue.append({'xyz': np.array(self.current_xyz, dtype=float), 'servo': servo, 'pitch': pitch_deg})
            reference = np.array(servo, dtype=float)
        for point in points:
            if not self._workspace_point_is_valid(point, include_table=True):
                self._trigger_safe_recovery(f'Waypoint fuera de workspace: {point.round(1).tolist()}')
                return False
            ok, servo, reason = self._solve_pose(point, pitch_deg, reference_angles=reference)
            if not ok:
                self._trigger_safe_recovery(f'No existe pose segura para {label}: {reason}; XYZ={point.round(1).tolist()}, pitch={pitch_deg:.1f}°')
                return False
            if float(np.max(np.abs(np.array(servo) - reference))) > float(max_joint_delta_deg):
                self._trigger_safe_recovery(f'Salto articular excesivo en {label}; se canceló la trayectoria')
                return False
            queue.append({'xyz': point.copy(), 'servo': servo, 'pitch': pitch_deg})
            reference = np.array(servo, dtype=float)
        self.vision_motion_queue = queue
        self.vision_segment_next_state = str(next_state)
        self.vision_segment_label = str(label)
        self.lbl_vision_phase.setText(f'Fase: {self.vision_state} · {label} · {len(queue)} puntos')
        return True

    def _run_vision_motion_queue(self):
        if not self.vision_motion_queue:
            return False
        step_record = self.vision_motion_queue[0]
        target_servo = np.array(step_record['servo'], dtype=float)
        diff = target_servo - self.current_physical_angles
        max_abs = float(np.max(np.abs(diff)))
        self.vision_motion_speed_dps = self.AUTOMATIC_SPEED_DEFAULT_DPS
        max_step = max(0.15, float(self.vision_motion_speed_dps) * 0.05)
        if max_abs <= 0.25:
            next_angles = target_servo
        else:
            next_angles = self.current_physical_angles + diff * min(1.0, max_step / max_abs)
        if step_record.get('base_only', False):
            next_angles[1:] = self.current_physical_angles[1:]
        if step_record.get('base_locked', False) and self.vision_locked_base_servo_deg is not None:
            next_angles[0] = float(self.vision_locked_base_servo_deg)
        margin = float(step_record.get('joint_margin_deg', self.JOINT_MARGIN_DEG))
        margin = max(0.25, margin)
        if step_record.get('initial_escape', False):
            outside_limits = np.any(next_angles < -0.5) or np.any(next_angles > 180.5)
        else:
            outside_limits = np.any(next_angles < margin - 0.5) or np.any(next_angles > 180.5 - margin)
        if outside_limits:
            self._trigger_safe_recovery('La trayectoria intentó exceder un límite de servo')
            return False
        final_pos, matrices = self.robot.compute_fk(next_angles)
        if not np.all(np.isfinite(final_pos)):
            self._trigger_safe_recovery('FK no finita durante movimiento de visión')
            return False
        self.current_physical_angles = np.array(next_angles, dtype=float)
        self.current_xyz = np.array(final_pos, dtype=float)
        self._draw_robot(matrices)
        self._update_text_results(self.current_physical_angles, self.current_xyz)
        if max_abs <= 0.25:
            if not step_record.get('servo_only', False):
                expected = np.array(step_record['xyz'], dtype=float)
                if float(np.linalg.norm(self.current_xyz - expected)) > 3.0:
                    self._trigger_safe_recovery('Error cartesiano excesivo al completar waypoint')
                    return True
            self.vision_motion_queue.pop(0)
            if not self.vision_motion_queue:
                if self.vision_pending_relock_base_deg is not None:
                    self.vision_locked_base_servo_deg = float(self.current_physical_angles[0])
                    self.vision_pending_relock_base_deg = None
                next_state = self.vision_segment_next_state or self.vision_state
                label = self.vision_segment_label
                self.vision_segment_next_state = None
                self.vision_segment_label = ''
                self._set_vision_state(next_state, f'completado: {label}')
        return True

    def _queue_vision_points(self, points, pitch_deg, next_state, label, max_joint_delta_deg=42.0):
        if self.vision_motion_queue:
            return True
        points = [np.array(point, dtype=float) for point in points]
        if not points:
            self._set_vision_state(next_state, f'sin desplazamiento: {label}')
            return True
        queue = []
        reference = np.array(self.current_physical_angles, dtype=float)
        ok, servo, _reason = self._solve_pose(self.current_xyz, pitch_deg, reference_angles=reference)
        if ok and float(np.max(np.abs(np.array(servo) - reference))) > 0.35:
            queue.append({'xyz': np.array(self.current_xyz, dtype=float), 'servo': np.array(servo, dtype=float), 'pitch': float(pitch_deg)})
            reference = np.array(servo, dtype=float)
        for point in points:
            if not self._workspace_point_is_valid(point, include_table=True):
                return False
            ok, servo, _reason = self._solve_pose(point, pitch_deg, reference_angles=reference)
            if not ok:
                return False
            if float(np.max(np.abs(np.array(servo) - reference))) > float(max_joint_delta_deg):
                return False
            queue.append({'xyz': point.copy(), 'servo': np.array(servo, dtype=float), 'pitch': float(pitch_deg)})
            reference = np.array(servo, dtype=float)
        self.vision_motion_queue = queue
        self.vision_segment_next_state = str(next_state)
        self.vision_segment_label = str(label)
        self.lbl_vision_phase.setText(f'Fase: {self.vision_state} · {label} · {len(queue)} puntos')
        return True

    def _begin_servo_segment(self, target_servo, next_state, label, max_joint_delta_deg=42.0, allow_initial_escape=False):
        if self.vision_motion_queue:
            return True
        target = np.array(target_servo, dtype=float)
        if not np.all(np.isfinite(target)):
            self._trigger_safe_recovery(f'Objetivo articular inválido en {label}')
            return False
        target = np.clip(target, 0.0, 180.0)
        reference = np.array(self.current_physical_angles, dtype=float)
        if not np.all(np.isfinite(reference)):
            reference = np.array(self.robot.startup_angles, dtype=float)
            self.current_physical_angles = reference.copy()
            self.current_xyz, _ = self.robot.compute_fk(reference)
        if not allow_initial_escape and float(np.max(np.abs(target - reference))) > float(max_joint_delta_deg):
            self._trigger_safe_recovery(f'Salto articular excesivo en {label}; se canceló la trayectoria')
            return False
        self.vision_motion_queue = [{'servo': target, 'servo_only': True, 'initial_escape': bool(allow_initial_escape), 'pitch': self._current_tool_pitch_deg()}]
        self.vision_segment_next_state = str(next_state)
        self.vision_segment_label = str(label)
        self.lbl_vision_phase.setText(f'Fase: {self.vision_state} · {label} · 1 punto')
        return True

    def _polar_transfer_points(self, start_xyz, target_xyz, transfer_radius, step_mm=None):
        start = np.array(start_xyz, dtype=float)
        target = np.array(target_xyz, dtype=float)
        step = float(step_mm or self.VISION_STEP_MM)
        z = float(start[2])
        target = target.copy()
        target[2] = z
        r0 = math.hypot(float(start[0]), float(start[1]))
        r1 = math.hypot(float(target[0]), float(target[1]))
        a0 = math.atan2(float(start[1]), float(start[0]))
        a1 = math.atan2(float(target[1]), float(target[0]))
        radius = float(max(transfer_radius, r0, r1))
        points = []

        def append_radial(r_start, r_end, angle):
            distance = abs(r_end - r_start)
            count = max(1, int(math.ceil(distance / step))) if distance > 1e-06 else 0
            for i in range(1, count + 1):
                r = r_start + (r_end - r_start) * (i / count)
                points.append(np.array([r * math.cos(angle), r * math.sin(angle), z]))
        append_radial(r0, radius, a0)
        delta = a1 - a0
        arc_length = abs(delta) * radius
        count = max(1, int(math.ceil(arc_length / step))) if arc_length > 1e-06 else 0
        for i in range(1, count + 1):
            angle = a0 + delta * (i / count)
            points.append(np.array([radius * math.cos(angle), radius * math.sin(angle), z]))
        append_radial(radius, r1, a1)
        if not points or float(np.linalg.norm(points[-1] - target)) > 1e-06:
            points.append(target.copy())
        return points

    def _begin_safe_polar_transfer(self, target_xyz, pitch_deg, next_state, label):
        start = np.array(self.current_xyz, dtype=float)
        target = np.array(target_xyz, dtype=float)
        target[2] = start[2]
        r0 = math.hypot(float(start[0]), float(start[1]))
        r1 = math.hypot(float(target[0]), float(target[1]))
        scan_radius = self.scan_radius_mm
        minimum_safe_radius = max(190.0, min(230.0, scan_radius - 10.0))
        raw_candidates = [max(r0, r1, minimum_safe_radius), max(r0, r1, scan_radius), 200.0, 210.0, 220.0, 230.0, 240.0]
        candidates = []
        for value in raw_candidates:
            value = float(np.clip(value, 185.0, 250.0))
            if all((abs(value - existing) > 0.5 for existing in candidates)):
                candidates.append(value)
        for radius in candidates:
            points = self._polar_transfer_points(start, target, radius)
            if self._queue_vision_points(points, pitch_deg, next_state, f'{label} · arco R={radius:.0f} mm', max_joint_delta_deg=24.0):
                return True
        self._trigger_safe_recovery(f'No se encontró arco seguro para {label}; inicio={start.round(1).tolist()}, destino={target.round(1).tolist()}')
        return False

    def _build_scan_plan(self, radius, scan_z, scan_pitch):
        points, servo_targets = ([], [])
        reference = np.array(self.robot.startup_angles, dtype=float)
        for angle_deg in np.linspace(25.0, 155.0, 9):
            angle = math.radians(float(angle_deg))
            requested = np.array([float(radius) * math.cos(angle), float(radius) * math.sin(angle), float(scan_z)])
            if not self._workspace_point_is_valid(requested, include_table=True):
                continue
            if not servo_targets:
                ok, servo, _reason = self._solve_pose(requested, float(scan_pitch), reference_angles=reference, allow_pitch_fallback=False)
                if not ok:
                    continue
                target_servo = np.array(servo, dtype=float)
            else:
                target_servo = reference.copy()
                target_servo[0] = float(angle_deg)
            if not np.all(np.isfinite(target_servo)):
                continue
            final_pos, _ = self.robot.compute_fk(target_servo)
            final_pos = np.array(final_pos, dtype=float)
            if not self._workspace_point_is_valid(final_pos, include_table=True):
                continue
            points.append(final_pos)
            servo_targets.append(target_servo.copy())
            reference = target_servo
        if len(points) < 3:
            raise ValueError('no se pudo construir un arco de paneo cercano')
        return (points, servo_targets)

    def _generate_scan_points(self, allow_fallback=False):
        requested = (self.scan_radius_mm, self.scan_z_mm, self.scan_pitch_deg)
        profiles = [requested]
        if allow_fallback:
            profiles.extend([(120.0, 140.0, 85.0), (125.0, 140.0, 90.0), (115.0, 135.0, 85.0), (130.0, 145.0, 90.0), (140.0, 140.0, 85.0)])
        unique_profiles = []
        for profile in profiles:
            rounded = tuple((round(float(value), 3) for value in profile))
            if rounded not in unique_profiles:
                unique_profiles.append(rounded)
        failures = []
        for radius, scan_z, scan_pitch in unique_profiles:
            try:
                points, servo_targets = self._build_scan_plan(radius, scan_z, scan_pitch)
                self.scan_servo_targets = servo_targets
                if (radius, scan_z, scan_pitch) != tuple((round(v, 3) for v in requested)):
                    self.scan_radius_mm = float(radius)
                    self.scan_z_mm = float(scan_z)
                    self.scan_pitch_deg = float(scan_pitch)
                    self._show_msg(f'Los valores de paneo solicitados no eran utilizables; se aplicó automáticamente R={radius:.0f} mm, Z={scan_z:.0f} mm, pitch={scan_pitch:.0f}°.')
                return points
            except Exception as exc:
                failures.append(str(exc))
        raise ValueError('No se pudo generar un paneo cercano ni con los perfiles automáticos: ' + '; '.join(failures[-3:]))

    def _lock_best_detection(self, detection):
        locked = dict(detection)
        if locked.get('scene_id') not in self.scene_objects:
            locked = self._register_scene_object(locked)
        scene_id = int(locked['scene_id'])
        if self.vision_locked_base_servo_deg is None:
            self.vision_locked_base_servo_deg = float(self.current_physical_angles[0])
        base_deg = float(self.vision_locked_base_servo_deg)
        q0 = math.radians(base_deg)
        radial = np.array([math.cos(q0), math.sin(q0)], dtype=float)
        tangential = np.array([-radial[1], radial[0]], dtype=float)
        camera_center_xyz = np.array(self.current_xyz, dtype=float)
        offset_xy = self._camera_to_gripper_offset_xy()
        radial_offset = float(np.dot(offset_xy, radial))
        final_radial_trim = self._final_radial_pick_trim_mm()
        angle_compensation = self._angle_dependent_radial_compensation_mm(base_deg)
        ignored_tangential = float(np.dot(offset_xy, tangential))
        target_radius = max(55.0, math.hypot(float(camera_center_xyz[0]), float(camera_center_xyz[1])) + radial_offset + final_radial_trim + angle_compensation)
        regional_xy = self._angle_dependent_xy_compensation_mm(base_deg)
        angle_edge_xy = self._angle_edge_xy_compensation_mm(base_deg)
        radial_target_xy = target_radius * radial
        target_xy = radial_target_xy + regional_xy + angle_edge_xy
        target_xyz = np.array([target_xy[0], target_xy[1], self._pickup_z_mm()], dtype=float)
        corrected_base_deg = float(np.clip(np.degrees(math.atan2(target_xy[1], target_xy[0])), 0.0, 180.0))
        locked.update({'camera_center_xyz': camera_center_xyz, 'camera_to_gripper_offset_xy': offset_xy.copy(), 'camera_to_gripper_radial_mm': radial_offset, 'final_radial_pick_trim_mm': final_radial_trim, 'angle_dependent_radial_compensation_mm': angle_compensation, 'regional_xy_compensation_mm': regional_xy.copy(), 'angle_edge_xy_compensation_mm': angle_edge_xy.copy(), 'total_xy_compensation_mm': (regional_xy + angle_edge_xy).copy(), 'total_radial_pick_correction_mm': radial_offset + final_radial_trim + angle_compensation, 'ignored_tangential_offset_mm': ignored_tangential, 'visual_locked_base_servo_deg': base_deg, 'corrected_pick_base_servo_deg': corrected_base_deg, 'locked_base_servo_deg': base_deg, 'workspace_xyz': target_xyz.copy(), 'workspace_valid': True})
        scene = self.scene_objects[scene_id]
        scene.update({'xyz': target_xyz.copy(), 'position_confirmed': True, 'status': 'Bloqueado', 'pickable': True})
        self.locked_detection = locked
        self.target_object_xyz = target_xyz.copy()
        self.active_detection_id = scene_id
        self.vision_motion_queue.clear()
        self.vision_segment_next_state = None
        self.slider_pinza.setMaximum(self.VISION_GRIPPER_OPEN_DEG)
        self.slider_pinza.setValue(self.VISION_GRIPPER_OPEN_DEG)
        self._force_hardware_send = True
        self._scene_table_dirty = True
        if self.vision_tracking_pitch_deg is None:
            self.vision_tracking_pitch_deg = self._current_tool_pitch_deg()
        self.vision_last_center_candidate = dict(locked)
        self.vision_last_center_pose = np.array(self.current_xyz, dtype=float)
        self.vision_detection_armed = False
        lock_detail = 'por oclusión esperada' if locked.get('occlusion_lock') else 'por centro estable'
        self._set_yolo_paused(True, f"objetivo {locked['class']} #{scene_id} fijado {lock_detail}; descenso final sin visión")
        total_correction = float(locked['total_radial_pick_correction_mm'])
        total_xy = np.array(locked['total_xy_compensation_mm'], dtype=float)
        self._set_vision_state('TARGET_LOCKED', f"{locked['class']} #{scene_id} · base={base_deg:.1f}° · corrección radial={total_correction:+.1f} mm · comp. XY=({total_xy[0]:+.1f}, {total_xy[1]:+.1f}) mm")
        self._update_scene_object_table()
        self._redraw_robot()

    def _mark_locked_object(self, status):
        if self.active_detection_id not in self.scene_objects:
            return
        item = self.scene_objects[self.active_detection_id]
        item['status'] = str(status)
        if status == 'Entregado':
            item['xyz'] = np.array(self.drop_zone_xyz, dtype=float)
        self._scene_table_dirty = True
        self._update_scene_object_table()

    def _detection_center_error_xy(self, detection):
        frame_w = max(2.0, float(detection.get('frame_w', 416.0)))
        frame_h = max(2.0, float(detection.get('frame_h', 416.0)))
        center_x = (frame_w - 1.0) / 2.0
        center_y = (frame_h - 1.0) / 2.0
        return np.array([float(detection.get('cx', center_x)) - center_x, float(detection.get('cy', center_y)) - center_y], dtype=float)

    def _begin_centering_on_detection(self, detection):
        scene_id = detection.get('scene_id')
        if scene_id != self.vision_centering_scene_id:
            self.vision_centering_scene_id = scene_id
            self.vision_centering_attempts = 0
            self.vision_centering_stable_hits = 0
            self.vision_centering_phase = 'BASE'
            self.vision_locked_base_servo_deg = None
            self.vision_pending_relock_base_deg = None
            self.vision_tracking_pitch_deg = self._current_tool_pitch_deg()
            self.vision_last_center_candidate = None
            self.vision_last_center_pose = None
            self.vision_center_loss_ticks = 0
        x_error, y_error = map(float, self._detection_center_error_xy(detection))
        deadzone = float(self.vision_centering_pixel_deadzone)
        pitch = float(self.vision_tracking_pitch_deg if self.vision_tracking_pitch_deg is not None else self._current_tool_pitch_deg())
        if self.vision_centering_attempts >= int(self.vision_centering_limit):
            self._show_msg('No se pudo completar el giro de base y avance radial dentro del límite de movimientos. Revise las inversiones de dirección o reduzca el paso.', is_error=True)
            return 'failed'
        if self.vision_centering_phase == 'BASE':
            if abs(x_error) <= deadzone:
                self.vision_centering_stable_hits += 1
                required = int(self.VISION_BASE_STABLE_HITS)
                self.lbl_vision_target.setText(f'Base alineada {self.vision_centering_stable_hits}/{required} · error horizontal={x_error:.0f}px · servo base={self.current_physical_angles[0]:.1f}°')
                if self.vision_centering_stable_hits < required:
                    return 'waiting'
                self.vision_locked_base_servo_deg = float(self.current_physical_angles[0])
                self.vision_centering_phase = 'RADIAL'
                self.vision_centering_stable_hits = 0
                if scene_id in self.scene_objects:
                    scene = self.scene_objects[scene_id]
                    scene['status'] = f'Base bloqueada {self.vision_locked_base_servo_deg:.1f}° · avance radial'
                    scene['position_confirmed'] = False
                    self._scene_table_dirty = True
                    self._update_scene_object_table()
                self._set_vision_state('CENTERING', f'base bloqueada a {self.vision_locked_base_servo_deg:.1f}°; iniciar avance radial')
            else:
                self.vision_centering_stable_hits = 0
                direction = -1.0 if x_error > 0.0 else 1.0
                if self.chk_camera_axis_x_invert.isChecked():
                    direction *= -1.0
                requested = float(self.VISION_BASE_STEP_DEG)
                candidate_steps = []
                for value in (requested, max(self.VISION_BASE_MIN_STEP_DEG, requested * 0.5), self.VISION_BASE_MIN_STEP_DEG):
                    if all((abs(value - existing) > 0.01 for existing in candidate_steps)):
                        candidate_steps.append(value)
                used_step = None
                for step_deg in candidate_steps:
                    label = f'giro visual de base {self.vision_centering_attempts + 1}/{self.vision_centering_limit}'
                    if self._begin_visual_base_segment(direction, step_deg, label):
                        used_step = step_deg
                        break
                if used_step is None:
                    self._show_msg('No existe un pequeño giro seguro del servo base en la dirección solicitada. El brazo quedó inmóvil y no se ejecutó HOME.', is_error=True)
                    return 'failed'
                self.vision_centering_attempts += 1
                if scene_id in self.scene_objects:
                    self.scene_objects[scene_id]['status'] = 'Centrando con servo base'
                self._reset_detection_runtime()
                self._set_vision_state('CENTERING', f'solo base · error horizontal={x_error:.0f}px · paso={direction * used_step:+.2f}° · pitch={pitch:.1f}°')
                return 'moving'
        if self.vision_locked_base_servo_deg is None:
            self.vision_locked_base_servo_deg = float(self.current_physical_angles[0])
        near_center_limit = deadzone * 1.6
        lateral_lock_limit = deadzone * 2.5
        radial_centered = abs(y_error) <= deadzone
        lateral_still_valid = abs(x_error) <= lateral_lock_limit
        if radial_centered and lateral_still_valid:
            self.vision_last_center_candidate = dict(detection)
            self.vision_last_center_pose = np.array(self.current_xyz, dtype=float)
            self.vision_center_loss_ticks = 0
            self.vision_centering_stable_hits += 1
            required = int(self.vision_centering_required_stable_hits)
            self.lbl_vision_target.setText(f'Avance radial centrado {self.vision_centering_stable_hits}/{required} · error px=({x_error:.0f}, {y_error:.0f}) · base fija={self.vision_locked_base_servo_deg:.1f}°')
            return 'centered' if self.vision_centering_stable_hits >= required else 'waiting'
        if abs(y_error) <= near_center_limit and abs(x_error) <= lateral_lock_limit:
            self.vision_last_center_candidate = dict(detection)
            self.vision_last_center_pose = np.array(self.current_xyz, dtype=float)
            self.vision_center_loss_ticks = 0
        self.vision_centering_stable_hits = 0
        if radial_centered and (not lateral_still_valid):
            self._show_msg('El avance radial llegó a la altura visual del bloque, pero el objeto se alejó demasiado del centro horizontal con la base bloqueada. Reinicie el paneo para volver a alinear la base.', is_error=True)
            return 'failed'
        direction = -1.0 if y_error > 0.0 else 1.0
        if self.chk_camera_axis_y_invert.isChecked():
            direction *= -1.0
        requested_step = self.VISION_STEP_MM
        candidate_steps = []
        for value in (requested_step, max(1.0, requested_step * 0.5), 1.0):
            if all((abs(value - existing) > 0.05 for existing in candidate_steps)):
                candidate_steps.append(value)
        current_radius = math.hypot(float(self.current_xyz[0]), float(self.current_xyz[1]))
        used_step = None
        for step_mm in candidate_steps:
            target_radius = current_radius + direction * step_mm
            label = f'avance radial visual {self.vision_centering_attempts + 1}/{self.vision_centering_limit}'
            if self._begin_visual_radial_segment(target_radius, pitch, label):
                used_step = step_mm
                break
        if used_step is None:
            self._show_msg(f'No existe un paso radial físicamente válido con la base bloqueada en {self.vision_locked_base_servo_deg:.1f}°, incluso usando pasos pequeños y permitiendo que hombro, codo y muñeca adapten el pitch. Revise la dirección de avance radial. El brazo permaneció inmóvil y no se ejecutó HOME.', is_error=True)
            return 'failed'
        self.vision_centering_attempts += 1
        if scene_id in self.scene_objects:
            scene = self.scene_objects[scene_id]
            scene['pickable'] = None
            scene['status'] = f'Avance radial · base {self.vision_locked_base_servo_deg:.1f}°'
            scene.pop('feasibility_scan_index', None)
            self._scene_table_dirty = True
        self._reset_detection_runtime()
        self._set_vision_state('CENTERING', f'avance radial · error vertical={y_error:.0f}px · paso={direction * used_step:+.1f}mm · base fija={self.vision_locked_base_servo_deg:.1f}°')
        return 'moving'

    def _reset_detection_runtime(self):
        self._stable_detection = None
        self._stable_detection_hits = 0

    def _track_stable_detection(self, candidate):
        if candidate is None:
            self._stable_detection = None
            self._stable_detection_hits = 0
            return None
        current = dict(candidate)
        previous = self._stable_detection
        same = False
        if previous is not None:
            same = current.get('scene_id') == previous.get('scene_id')
            if not same and current.get('class') == previous.get('class'):
                dx = float(current.get('cx', 0.0)) - float(previous.get('cx', 0.0))
                dy = float(current.get('cy', 0.0)) - float(previous.get('cy', 0.0))
                frame_w = max(2.0, float(current.get('frame_w', 416.0)))
                frame_h = max(2.0, float(current.get('frame_h', 416.0)))
                same = math.hypot(dx, dy) <= max(35.0, 0.12 * min(frame_w, frame_h))
        self._stable_detection_hits = self._stable_detection_hits + 1 if same else 1
        current['hits'] = self._stable_detection_hits
        self._stable_detection = current
        return dict(current)

    def _update_detections(self, objects_list):
        if self.vision_state != 'IDLE' and (not self.vision_detection_armed):
            return
        allowed = self._allowed_tetris_classes()
        mapping_allowed = bool(self.observe_mode_active or self.vision_state in ('IDLE', 'SCAN_DWELL', 'CENTERING'))
        enriched = []
        for raw in list(objects_list)[:max(1, self.slider_camera_objects.value())]:
            item = dict(raw)
            item['class'] = self._normalize_tetris_class(item.get('class', 'objeto'))
            if float(item.get('confidence', 0.0)) < self.YOLO_CONFIDENCE:
                continue
            if allowed is not None and item['class'] not in allowed:
                continue
            if 'cx' not in item or 'cy' not in item:
                item['cx'] = int((float(item.get('x1', 0.0)) + float(item.get('x2', 0.0))) / 2.0)
                item['cy'] = int((float(item.get('y1', 0.0)) + float(item.get('y2', 0.0))) / 2.0)
            frame_w = max(2.0, float(item.get('frame_w', 416.0)))
            frame_h = max(2.0, float(item.get('frame_h', 416.0)))
            item['pixel_error_x'] = float(item['cx']) - (frame_w - 1.0) / 2.0
            item['pixel_error_y'] = float(item['cy']) - (frame_h - 1.0) / 2.0
            anchor = np.array(self.current_xyz, dtype=float)
            anchor[2] = self._pickup_z_mm()
            item['workspace_xyz'] = anchor
            item['workspace_valid'] = True
            if mapping_allowed:
                item = self._register_scene_object(item)
            enriched.append(item)
        best = max(enriched, key=lambda item: float(item.get('confidence', 0.0)), default=None)
        if not mapping_allowed:
            if best is None:
                self.lbl_detected_objects.setText('YOLO: 0 objetos >= 70%')
                self.lbl_vision_target.setText('Objetivo: buscando centro de pieza')
            else:
                self.lbl_detected_objects.setText(f'YOLO: {len(enriched)} objeto(s) >= 70%')
                self.lbl_vision_target.setText(f"Viendo {best.get('class', 'objeto')} ({best.get('confidence', 0.0):.2f}) · error px=({best['pixel_error_x']:.0f}, {best['pixel_error_y']:.0f})")
            return
        stable = self._track_stable_detection(best)
        if stable is not None:
            for index, item in enumerate(enriched):
                if item.get('scene_id') == stable.get('scene_id'):
                    enriched[index] = stable
                    break
        if stable is None:
            self.lbl_detected_objects.setText('Detecciones válidas: 0')
            self.lbl_vision_target.setText('Objetivo: sin pieza estable')
            if self.vision_state == 'IDLE':
                self.active_detection_id = None
            return
        hits = int(stable.get('hits', 0))
        required_hits = int(self.required_live_detection_hits)
        px = float(stable.get('pixel_error_x', 0.0))
        py = float(stable.get('pixel_error_y', 0.0))
        self.lbl_detected_objects.setText(f'Detecciones válidas: {len(enriched)} · estabilidad {hits}/{required_hits}')
        self.lbl_vision_target.setText(f"Centro visual: {stable.get('class', 'objeto')} #{stable.get('scene_id', '?')} ({stable.get('confidence', 0.0):.2f}) · error px=({px:.0f}, {py:.0f}) · sin conversión a mm")
        if self.observe_mode_active or self.vision_state in ('IDLE', 'SCANNING', 'SCAN_DWELL', 'CENTERING'):
            anchor = np.array(self.current_xyz, dtype=float)
            anchor[2] = self._pickup_z_mm()
            self.target_object_xyz = anchor
            self.active_detection_id = stable.get('scene_id')

    def _best_live_detection(self):
        candidate = self._stable_detection
        required_hits = int(getattr(self, 'required_live_detection_hits', 1))
        if candidate is None or self._stable_detection_hits < required_hits:
            return None
        if not candidate.get('workspace_valid'):
            return None
        allowed = self._allowed_tetris_classes()
        if allowed is not None and candidate.get('class') not in allowed:
            return None
        return dict(candidate)

    def _try_lock_live_detection(self):
        candidate = self._best_live_detection()
        if candidate is None:
            if self.vision_centering_scene_id is not None and self.vision_last_center_candidate is not None and (self.vision_last_center_pose is not None):
                self.vision_center_loss_ticks += 1
                grace = int(self.vision_center_occlusion_grace_ticks)
                delay = int(self.vision_center_occlusion_lock_delay)
                if self.vision_center_loss_ticks <= grace:
                    self.lbl_vision_target.setText(f'Objetivo oculto bajo la cámara/garra cerca del centro · confirmación ciega {self.vision_center_loss_ticks}/{delay}')
                    if self.vision_center_loss_ticks >= delay:
                        candidate = dict(self.vision_last_center_candidate)
                        candidate['occlusion_lock'] = True
                        candidate['workspace_xyz'] = np.array(self.vision_last_center_pose, dtype=float)
                        candidate['workspace_valid'] = True
                        self._lock_best_detection(candidate)
                    return True
            return False
        self.vision_center_loss_ticks = 0
        scene_id = candidate.get('scene_id')
        scene = self.scene_objects.get(scene_id)
        if scene is None:
            return False
        result = self._begin_centering_on_detection(candidate)
        if result in ('moving', 'waiting'):
            return True
        if result != 'centered':
            scene['pickable'] = False
            scene['status'] = 'No centrado con estrategia base/radial'
            self._scene_table_dirty = True
            self._update_scene_object_table()
            return False
        centered_xyz = np.array(self.current_xyz, dtype=float)
        centered_xyz[2] = self._pickup_z_mm()
        candidate['workspace_xyz'] = centered_xyz.copy()
        candidate['workspace_valid'] = True
        scene['xyz'] = centered_xyz.copy()
        scene['pickable'] = True
        scene['feasibility_scan_index'] = int(self.scan_point_index)
        scene['status'] = 'Posición radial confirmada · listo para descenso Z'
        self._scene_table_dirty = True
        self._update_scene_object_table()
        self._lock_best_detection(candidate)
        return True

    def _advance_scan_index(self):
        count = len(self.scan_servo_targets)
        if count < 2:
            return None
        next_index = self.scan_point_index + self.scan_direction
        if next_index < 0 or next_index >= count:
            self.scan_direction *= -1
            next_index = self.scan_point_index + self.scan_direction
        if next_index < 0 or next_index >= count:
            return None
        self.scan_point_index = next_index
        return np.array(self.scan_servo_targets[next_index], dtype=float)

    def _toggle_vision(self):
        active = self.vision_state not in ('IDLE', 'RECOVERY') and (not self.recovery_active)
        if active:
            self.vision_motion_queue.clear()
            self.vision_actions.clear()
            self.recovery_active = False
            self.recovery_stage = 'IDLE'
            self.locked_detection = None
            self.active_detection_id = None
            self.vision_centering_scene_id = None
            self.vision_centering_attempts = 0
            self.vision_centering_stable_hits = 0
            self.vision_centering_phase = 'BASE'
            self.vision_locked_base_servo_deg = None
            self.vision_pending_relock_base_deg = None
            self.vision_tracking_pitch_deg = None
            self.vision_last_center_candidate = None
            self.vision_last_center_pose = None
            self.vision_center_loss_ticks = 0
            self.vision_detection_armed = True
            self._set_yolo_paused(False, 'búsqueda detenida')
            self._set_vision_state('IDLE', 'detenido por operador')
            self.btn_vision.setText('Iniciar búsqueda continua')
            self.btn_vision.setStyleSheet('background-color: #8B5CF6; color: #FFF; border-radius: 6px; font-weight: bold; font-size: 14px;')
            self.slider_pinza.setMaximum(self.MANUAL_GRIPPER_MAX_DEG)
            self.slider_pinza.setValue(self.MANUAL_GRIPPER_MAX_DEG)
            self._force_hardware_send = True
            self._show_msg(f'Proceso de visión detenido con {len(self.scene_objects)} objetos detectados.')
            return
        self._reset_path(redraw=False)
        self.recovery_active = False
        self.recovery_stage = 'IDLE'
        self.recovery_joint_target = None
        self.vision_motion_queue.clear()
        self.vision_actions.clear()
        self.vision_segment_next_state = None
        self.vision_segment_label = ''
        self.vision_state = 'IDLE'
        if self.vision_thread is None or not self.vision_thread.isRunning():
            self._show_msg('Conecta la cámara antes de iniciar la visión.', is_error=True)
            return
        if not self.chk_camera_yolo.isChecked():
            self._show_msg('Activa YOLO y vuelve a conectar la cámara.', is_error=True)
            return
        if self.chk_vision_physical.isChecked() and self.arduino_serial is None:
            self._show_msg('El movimiento físico de visión está activado, pero el robot no está conectado.', is_error=True)
            return
        try:
            self.scan_points = self._generate_scan_points(allow_fallback=True)
            self._save_vision_config(silent=True)
        except Exception as exc:
            self._show_msg(f'No se pudo crear el paneo automático: {exc}', is_error=True)
            return
        if self.observe_mode_active:
            self.btn_observe_mode.setChecked(False)
        self.mode_selector.setCurrentIndex(1)
        self.slider_pinza.setMaximum(self.VISION_GRIPPER_OPEN_DEG)
        self.slider_pinza.setValue(self.VISION_GRIPPER_OPEN_DEG)
        self._force_hardware_send = True
        self.active_detection_id = None
        self.locked_detection = None
        self.scan_point_index = 0
        self.vision_centering_scene_id = None
        self.vision_centering_attempts = 0
        self.vision_centering_stable_hits = 0
        self.vision_centering_phase = 'BASE'
        self.vision_locked_base_servo_deg = None
        self.vision_pending_relock_base_deg = None
        self.vision_tracking_pitch_deg = None
        self.vision_last_center_candidate = None
        self.vision_last_center_pose = None
        self.vision_center_loss_ticks = 0
        self.scan_direction = 1
        self._reset_detection_runtime()
        self.completed_pick_count = 0
        self.vision_detection_armed = False
        self._set_yolo_paused(True, 'esperando primer punto de paneo')
        self._set_vision_state('MOVE_TO_SCAN_START', 'entrada desde la pose actual')
        self._process_vision_state_machine()
        self.btn_vision.setText('Detener búsqueda')
        self.btn_vision.setStyleSheet(f'background-color: {self.STATUS_ERR}; color: #FFF; border-radius: 6px; font-weight: bold; font-size: 14px;')
        active_radius = self.scan_radius_mm
        self._show_msg(f'Visión real iniciada; paneo R={active_radius:.0f} mm. El sistema centrará con base, avance radial y descenso Z.')

    def _build_pick_place_actions(self):
        obj = np.array(self.target_object_xyz, dtype=float)
        obj[2] = self._pickup_z_mm()
        drop = np.array(self.drop_zone_xyz, dtype=float)
        first_scan = np.array(self.scan_points[0], dtype=float)
        scan_pitch = self.scan_pitch_deg
        pick_pitch = self.PICK_PITCH_DEG
        carry_pitch = self.CARRY_PITCH_DEG
        scan_z = self.scan_z_mm
        pre = self.PREGRASP_CLEARANCE_MM
        lift = self.LIFT_DISTANCE_MM
        tracking_pitch = float(self.vision_tracking_pitch_deg if self.vision_tracking_pitch_deg is not None else scan_pitch)
        actions = []

        def add_action(kind, label, first, second, third=(), fourth=None, fifth=42.0):
            if kind in ('move', 'locked', 'move_relock'):
                data = {'target': first, 'pitch': second, 'axes': tuple(third), 'status': fourth, 'max_delta': fifth}
            elif kind == 'polar':
                data = {'target': first, 'pitch': second}
            else:
                data = {'value': first, 'ticks': second, 'status': third or None}
            actions.append({'type': kind, 'label': label, **data})
        first_lift = obj.copy()
        first_lift[2] = max(self.BLOCK_HEIGHT_MM + 55.0, self._pickup_z_mm() + min(70.0, lift))
        carry_lift = obj.copy()
        carry_lift[2] = max(scan_z, self._pickup_z_mm() + lift)
        drop_high = drop.copy()
        drop_high[2] += max(pre + 35.0, 65.0)
        drop_point = drop.copy()
        drop_point[2] += 8.0
        drop_lift = drop.copy()
        drop_lift[2] += max(pre + 35.0, 65.0)
        drop_carry = drop.copy()
        drop_carry[2] = max(scan_z, drop[2] + lift)
        add_action('grip', 'abrir garra antes de descender', self.VISION_GRIPPER_OPEN_DEG, 14)
        add_action('move_relock', 'corrección XY y rebloqueo de base', lambda: [obj[0], obj[1], self.current_xyz[2]], tracking_pitch, ('x', 'y'), None, 45.0)
        add_action('locked', f'descenso vertical a Z={self._pickup_z_mm():.0f} mm', obj, pick_pitch, ('z',), None, 60.0)
        add_action('grip', 'cerrar garra sobre el bloque', 0, 18, 'En agarre')
        add_action('locked', 'elevación vertical inicial', first_lift, pick_pitch, ('z',), 'En traslado', 60.0)
        add_action('move', 'orientar para traslado', lambda: self.current_xyz, carry_pitch)
        add_action('move', 'subir a altura de transporte', carry_lift, carry_pitch, ('z',))
        add_action('polar', 'traslado polar al dropzone', lambda: [drop[0], drop[1], self.current_xyz[2]], carry_pitch)
        add_action('move', 'descenso previo al dropzone', drop_high, carry_pitch, ('z',))
        add_action('move', 'orientar al dropzone', lambda: self.current_xyz, pick_pitch)
        add_action('move', 'descenso en dropzone', drop_point, pick_pitch, ('z',))
        add_action('grip', 'liberar pieza', self.VISION_GRIPPER_OPEN_DEG, 16, 'Entregado')
        add_action('move', 'retirada vertical del dropzone', drop_lift, pick_pitch, ('z',))
        add_action('move', 'recuperar pitch de traslado', lambda: self.current_xyz, carry_pitch)
        add_action('move', 'subir después de liberar', drop_carry, carry_pitch, ('z',))
        add_action('polar', 'retorno polar al paneo', lambda: [first_scan[0], first_scan[1], self.current_xyz[2]], scan_pitch)
        add_action('move', 'descenso a altura de paneo', first_scan, scan_pitch, ('z',))
        actions.append({'type': 'finish', 'label': 'ciclo completado'})
        self.vision_actions = actions

    def _run_next_vision_action(self):
        while self.vision_actions:
            action = self.vision_actions.pop(0)
            label = action.get('label', 'acción de visión')
            target_spec = action.get('target')
            target = target_spec() if callable(target_spec) else target_spec
            if action.get('status'):
                self._mark_locked_object(action['status'])
            if action['type'] == 'move':
                return self._begin_vision_segment(np.array(target, dtype=float), action['pitch'], 'PICK_PLACE', action['axes'], label, max_joint_delta_deg=action['max_delta'])
            if action['type'] == 'move_relock':
                target_array = np.array(target, dtype=float)
                self.vision_pending_relock_base_deg = float(np.degrees(math.atan2(target_array[1], target_array[0])))
                started = self._begin_vision_segment(target_array, action['pitch'], 'PICK_PLACE', action['axes'], label, max_joint_delta_deg=action['max_delta'])
                if not started:
                    self.vision_pending_relock_base_deg = None
                return started
            if action['type'] == 'locked':
                return self._begin_locked_base_segment(np.array(target, dtype=float), action['pitch'], 'PICK_PLACE', label, max_joint_delta_deg=action['max_delta'])
            if action['type'] == 'polar':
                return self._begin_safe_polar_transfer(np.array(target, dtype=float), action['pitch'], 'PICK_PLACE', label)
            if action['type'] == 'grip':
                self.slider_pinza.setValue(int(action['value']))
                self._force_hardware_send = True
                self.vision_timer = int(action['ticks'])
                self._set_vision_state('ACTION_WAIT', label)
                return True
            if action['type'] == 'finish':
                self._finish_pick_place_cycle()
                return True
        self._finish_pick_place_cycle()
        return True

    def _finish_pick_place_cycle(self):
        self.completed_pick_count += 1
        self.locked_detection = None
        self.active_detection_id = None
        self._reset_detection_runtime()
        self.vision_centering_scene_id = None
        self.vision_centering_attempts = 0
        self.vision_centering_stable_hits = 0
        self.vision_centering_phase = 'BASE'
        self.vision_locked_base_servo_deg = None
        self.vision_pending_relock_base_deg = None
        self.vision_tracking_pitch_deg = None
        self.vision_last_center_candidate = None
        self.vision_last_center_pose = None
        self.vision_center_loss_ticks = 0
        self.vision_detection_armed = True
        self.scan_point_index = 0
        self.scan_direction = 1
        self._set_yolo_paused(False, 'búsqueda en vivo reanudada')
        self._set_vision_state('SCAN_DWELL', f'ciclo {self.completed_pick_count} completado · búsqueda continua')
        self._update_scene_object_table()
        self._show_msg(f'Pick & Place #{self.completed_pick_count} completado. La visión seguirá buscando hasta que presiones Detener búsqueda.')

    def _trigger_safe_recovery(self, reason):
        if self.recovery_active:
            return
        self._reset_path(redraw=False)
        self.vision_motion_queue.clear()
        self.vision_actions.clear()
        self.vision_segment_next_state = None
        self.recovery_active = True
        self.recovery_stage = 'LIFT'
        self.recovery_joint_target = None
        self.recovery_reason = str(reason)
        self.slider_pinza.setMaximum(self.VISION_GRIPPER_OPEN_DEG)
        self.slider_pinza.setValue(self.VISION_GRIPPER_OPEN_DEG)
        self._force_hardware_send = True
        self._set_yolo_paused(False, 'recuperación activa; sin selección de nuevos objetivos')
        self._set_vision_state('RECOVERY', str(reason))
        self._show_msg(f'Recuperación segura: {reason}', is_error=True)

    def _run_safe_recovery(self):
        self.recovery_speed_dps = self.RECOVERY_SPEED_DPS
        self.recovery_safe_z = self.RECOVERY_Z_MM
        if self.recovery_stage == 'LIFT':
            current_pitch = self._current_tool_pitch_deg()
            lift_target = np.array(self.current_xyz, dtype=float)
            lift_target[2] = max(float(lift_target[2]), self.recovery_safe_z)
            ok = False
            servo = None
            start_z = float(lift_target[2])
            minimum_lift = float(self.current_xyz[2]) + 20.0
            for candidate_z in np.arange(start_z, minimum_lift - 1.0, -10.0):
                candidate = lift_target.copy()
                candidate[2] = candidate_z
                ok, servo, _reason = self._solve_pose(candidate, current_pitch)
                if ok:
                    break
            if ok:
                self.recovery_joint_target = servo
                self.recovery_stage = 'MOVE_LIFT'
            else:
                self.recovery_active = False
                self.recovery_stage = 'IDLE'
                self.recovery_joint_target = None
                self.vision_motion_queue.clear()
                self.vision_actions.clear()
                self._set_yolo_paused(False)
                self._set_vision_state('IDLE', 'pausado: no se encontro elevacion segura')
                self.btn_vision.setText('Iniciar busqueda continua')
                reason = self.recovery_reason or 'no se encontró una pose válida'
                self._show_msg(f'Movimiento pausado durante: {reason}. El brazo se mantuvo en su posición actual y no se envió HOME automático.', is_error=True)
                return False
        if self.recovery_joint_target is None:
            self.recovery_active = False
            self.recovery_stage = 'IDLE'
            self._set_vision_state('IDLE', 'pausado sin HOME')
            return False
        target = np.array(self.recovery_joint_target, dtype=float)
        diff = target - self.current_physical_angles
        max_abs = float(np.max(np.abs(diff)))
        max_step = max(0.1, self.recovery_speed_dps * 0.05)
        if max_abs <= 0.25:
            self.current_physical_angles = target.copy()
        else:
            self.current_physical_angles = self.current_physical_angles + diff * min(1.0, max_step / max_abs)
        final_pos, matrices = self.robot.compute_fk(self.current_physical_angles)
        self.current_xyz = np.array(final_pos, dtype=float)
        self._draw_robot(matrices)
        self._update_text_results(self.current_physical_angles, self.current_xyz)
        if max_abs <= 0.25:
            self.recovery_active = False
            self.recovery_stage = 'IDLE'
            self.recovery_joint_target = None
            self.locked_detection = None
            self.active_detection_id = None
            self.vision_motion_queue.clear()
            self.vision_actions.clear()
            self._set_yolo_paused(False)
            self._set_vision_state('IDLE', 'pausado despues de elevacion segura')
            self.btn_vision.setText('Iniciar busqueda continua')
            self._show_msg('Movimiento pausado despues de elevar. No se envio HOME automatico.', is_error=True)
        return True

    def _process_vision_state_machine(self):
        if self.vision_state == 'IDLE' or self.recovery_active or self.vision_motion_queue:
            return
        if self.vision_state == 'MOVE_TO_SCAN_START':
            if not self.scan_servo_targets:
                self._trigger_safe_recovery('La primera pose de paneo no está disponible')
                return
            self._begin_servo_segment(np.array(self.scan_servo_targets[0], dtype=float), 'SCAN_DWELL', 'entrada al primer punto de paneo', max_joint_delta_deg=180.0, allow_initial_escape=True)
            return
        if self.vision_state == 'SCANNING':
            if self.locked_detection is not None:
                self._set_vision_state('TARGET_LOCKED')
                return
            self._reset_detection_runtime()
            self._set_yolo_paused(False, 'paneo en movimiento')
            target = self._advance_scan_index()
            if target is None:
                self._trigger_safe_recovery('La secuencia de paneo no está lista')
                return
            direction = 'ida' if self.scan_direction > 0 else 'regreso'
            self._begin_servo_segment(target, 'SCAN_DWELL', f'paneo {direction} · punto {self.scan_point_index + 1}/{len(self.scan_points)}', max_joint_delta_deg=180.0, allow_initial_escape=True)
            return
        if self.vision_state in ('SCAN_DWELL', 'CENTERING'):
            self._set_yolo_paused(False)
            if self.locked_detection is None and self._try_lock_live_detection():
                return
            if self.locked_detection is not None:
                self._set_vision_state('TARGET_LOCKED')
                return
            if self.vision_state == 'SCAN_DWELL':
                self.vision_timer -= 1
                if self.vision_timer <= 0:
                    self._set_vision_state('SCANNING', 'sin objetivo; continuar paneo')
            return
        if self.vision_state == 'TARGET_LOCKED':
            if self.locked_detection is None:
                self._set_yolo_paused(False)
                self._set_vision_state('SCANNING', 'objetivo perdido antes del bloqueo')
                return
            self._build_pick_place_actions()
            self._set_vision_state('PICK_PLACE', 'secuencia preparada')
            return
        if self.vision_state == 'PICK_PLACE':
            self._run_next_vision_action()
            return
        if self.vision_state == 'ACTION_WAIT':
            self.vision_timer -= 1
            if self.vision_timer <= 0:
                self._set_vision_state('PICK_PLACE')

    def _control_loop_tick(self):
        if self._syncing:
            return
        try:
            self._control_loop_tick_guarded()
        except Exception as exc:
            self._trigger_safe_recovery(f'Excepción de control {type(exc).__name__}: {exc}')

    def _control_loop_tick_guarded(self):
        pendant_control_active = self.chk_pendant.isChecked() and self.vision_state == 'IDLE' and (not self.path_active) and (not self.recovery_active)
        manual_override = False
        if self.pendant_serial:
            manual_override = self._read_pendant(allow_motion=pendant_control_active)
        self._maintain_pendant_connection(time.time())
        moved = False
        if self.recovery_active:
            moved = self._run_safe_recovery()
        else:
            if self.vision_state != 'IDLE':
                self._process_vision_state_machine()
            elif self.path_active:
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
            if self.vision_state != 'IDLE':
                moved = self._run_vision_motion_queue() or moved
            elif pendant_control_active:
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
        vision_physical = (self.vision_state != 'IDLE' or self.recovery_active) and self.chk_vision_physical.isChecked()
        send_requested = moved or self._force_hardware_send
        send_enabled = self.chk_live.isChecked() or self.chk_pendant.isChecked() or path_physical or vision_physical
        if send_requested and send_enabled and (current_time - self.last_robot_send_time > 0.1):
            self._send_to_arduino(live_mode=True)
            self.last_robot_send_time = current_time
            self._force_hardware_send = False
        self._update_overhead_alignment_label()
        now_mono = time.monotonic()
        if self._scene_table_dirty and now_mono - self._last_scene_table_update >= 0.25:
            self._update_scene_object_table()
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
            self._update_overhead_alignment_label()
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
        show_drop_zone = hasattr(self, 'tabs') and hasattr(self, 'vision_tab_index') and (self.tabs.currentIndex() == self.vision_tab_index)
        if show_drop_zone:
            dx, dy, dz = self.drop_zone_xyz
            self.ax.scatter([dx], [dy], [dz], color='#8B5CF6', s=500, marker='X', edgecolor='w')
            self.ax.text(dx, dy, dz + 25, 'Drop Zone', color='#8B5CF6', fontweight='bold')
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