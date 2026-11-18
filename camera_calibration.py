"""Interactive intrinsic calibration using the unrotated camera stream."""
import base64
import json
import os
import queue
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from camera_connection import prepare_arm_stream


class CalibrationSession:
    def __init__(self, pattern=(9, 7), square_mm=25.0):
        self.pattern = pattern
        self.square_mm = square_mm
        self.points = []
        self.size = None

    def capture(self, frame):
        h, w = frame.shape[:2]
        if self.size is not None and self.size != (w, h):
            return 'Cambió la resolución; reinicia la calibración.'
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, self.pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            return 'No se detectó el tablero completo; mejora la luz o aléjalo.'
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .01))
        if any(np.mean(np.linalg.norm(corners.reshape(-1, 2)-old.reshape(-1, 2), axis=1)) < min(w, h)*.025 for old in self.points):
            return 'Vista muy parecida a otra; cambia posición o inclinación.'
        self.size = (w, h)
        self.points.append(corners)
        return f'Vista {len(self.points)} guardada. Cambia posición e inclinación.'

    def save(self, path):
        if len(self.points) < 12:
            raise ValueError('Se necesitan al menos 12 vistas distintas.')
        obj = np.zeros((self.pattern[0]*self.pattern[1], 3), np.float32)
        obj[:, :2] = np.mgrid[0:self.pattern[0], 0:self.pattern[1]].T.reshape(-1, 2)*self.square_mm
        rms, matrix, distortion, _, _ = cv2.calibrateCamera([obj.copy() for _ in self.points], self.points, self.size, None, None)
        if not np.isfinite(rms) or not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion)) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
            raise ValueError('Resultado inválido; repite las capturas.')
        target = Path(path)
        temporary = target.with_name(target.name + '.tmp')
        try:
            with temporary.open('wb') as output:
                np.savez(output, camera_matrix=matrix, dist_coeffs=distortion, image_size=np.array(self.size),
                         rms=np.array(rms), board_inner_corners=np.array(self.pattern), square_mm=np.array(self.square_mm))
            os.replace(temporary, target)
        finally:
            if temporary.exists(): temporary.unlink()
        return float(rms)


def emit(packet):
    print(json.dumps(packet, separators=(',', ':')), flush=True)


def run(config):
    commands = queue.Queue()
    def read_commands():
        for line in sys.stdin:
            commands.put(line.strip())
    threading.Thread(target=read_commands, daemon=True).start()
    source = config['url'].strip()
    if config.get('arm_camera_api'):
        source = prepare_arm_stream(source, status=lambda message: emit({'status': message}))
    if not source.startswith(('http://', 'https://', 'rtsp://')):
        raise ValueError('Introduce una URL http(s) o rtsp de la cámara.')
    emit({'status': 'Abriendo cámara para calibración…'})
    cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG, [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000])
    if not cap.isOpened():
        cap.release()
        raise RuntimeError('No se pudo abrir el stream de la cámara.')
    session = CalibrationSession()
    try:
        while True:
            started = time.monotonic()
            ok, frame = cap.read()
            if not ok: raise RuntimeError('Se perdió el stream de cámara.')
            while not commands.empty():
                command = commands.get_nowait()
                if command == 'capture':
                    emit({'status': session.capture(frame), 'count': len(session.points)})
                elif command == 'finish':
                    try:
                        rms = session.save(config['output'])
                        emit({'completed': config['output'], 'rms': rms, 'count': len(session.points)})
                        return
                    except (ValueError, OSError, cv2.error) as exc:
                        emit({'status': str(exc), 'count': len(session.points)})
                elif command == 'cancel': return
            from camera_worker import rotate_preview
            preview = rotate_preview(frame, int(config.get('rotation', 0)))
            h, w = preview.shape[:2]
            scale = min(1., 640/max(w, h))
            if scale < 1: preview = cv2.resize(preview, (round(w*scale), round(h*scale)))
            ok, jpeg = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok: emit({'jpeg': base64.b64encode(jpeg).decode()})
            time.sleep(max(0, .1-(time.monotonic()-started)))
    finally:
        cap.release()


if __name__ == '__main__':
    try: run(json.loads(sys.argv[1]))
    except Exception as exc:
        emit({'error': str(exc)})
        sys.exit(1)
