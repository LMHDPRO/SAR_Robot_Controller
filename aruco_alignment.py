"""Conservative, pulse-based approach to a taught marker pose."""
import math
import cv2
import numpy as np

# The stream has no camera exposure timestamp. Allow the reported ~1 s
# pipeline delay to clear after motion before acting on another observation.
ASSUMED_CAMERA_LATENCY_S = 1.0
TARGET_DISTANCE_MM = 180.0
DISTANCE_TOLERANCE_MM = 5.0


class ArucoAlignment:
    def __init__(self, reference, now, pwm, require_orientation=True, require_new_sample=False):
        self.ref = reference
        self.require_orientation = require_orientation
        self.minimum_sample_time=now if require_new_sample else now-.5
        self.started = now
        self.pwm = min(90, max(1, int(pwm)))
        self.phase = 'observe'
        self.until = now
        self.stable = 0
        self.last_sample = 0.
        self.pulses = 0
        self.done = False
        self.message = ''
        self.last_valid_sample = None
        self.wait_since = now

    def output(self, sample, now, odom=None):
        if now-self.started > 180: raise RuntimeError('Tiempo de alineación agotado')
        if (sample and sample.get('captured_at', now+1) <= now+.1
                and (self.last_valid_sample is not None or sample['captured_at']>=self.minimum_sample_time)):
            if self.last_valid_sample is None or sample['captured_at'] > self.last_valid_sample['captured_at']:
                self.last_valid_sample = sample
        sample = self.last_valid_sample
        if not sample:
            if now-self.wait_since>30:raise RuntimeError('Marcador perdido o imagen antigua durante 30 s')
            return (0,0)
        age=now-sample['captured_at']
        if age>30: raise RuntimeError('Marcador perdido o imagen antigua durante 30 s')
        if age>.5:
            # If motion was interrupted, still wait for the stream to clear.
            if self.phase in ('pulse','forward'):
                self.phase='settle';self.until=now+.25+ASSUMED_CAMERA_LATENCY_S
            self.stable=0
            return (0,0)
        if sample['id'] != 12: raise RuntimeError('Marcador 12 no disponible')
        if 'image_size' in self.ref and sample.get('image_size') != self.ref['image_size']:
            raise RuntimeError('Cambió la resolución; guarda otra referencia')
        xyz = np.asarray(sample['xyz_camera_mm'], dtype=float)
        rvec = np.asarray(sample['rvec'], dtype=float)
        if xyz.shape != (3,) or rvec.shape != (3,) or not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(rvec)) or xyz[2] <= 0:
            raise RuntimeError('Pose del marcador inválida')
        distance = float(np.linalg.norm(xyz))
        target = np.asarray(self.ref['xyz_camera_mm'], dtype=float)
        target_distance = TARGET_DISTANCE_MM
        if distance < target_distance-DISTANCE_TOLERANCE_MM: raise RuntimeError('Demasiado cerca del marcador; mueve el AGV manualmente')
        if distance > 600: raise RuntimeError('Marcador demasiado lejos para esta aproximación')
        if self.phase == 'forward':
            valid=(odom is not None and odom.imu_valid and odom.imu_segment==self.forward_segment
                   and odom.revision==self.forward_revision)
            travel=abs(((odom.left-self.forward_left)+(odom.right-self.forward_right))/2) if valid else 0
            yaw=abs(odom.imu_yaw-self.forward_yaw) if valid else 0
            if valid and now<self.until and travel<20 and yaw<4 and distance>target_distance+60:
                return self.wheels
            self.phase='settle';self.until=now+.25+ASSUMED_CAMERA_LATENCY_S
            return (0,0)
        if self.phase == 'pulse':
            if now < self.until: return self.wheels
            self.phase = 'settle'; self.until = now+.25+ASSUMED_CAMERA_LATENCY_S
            return (0, 0)
        if self.phase == 'settle':
            if now < self.until: return (0, 0)
            if sample['captured_at'] < self.until: return (0, 0)
            self.phase = 'observe'
        if sample['captured_at'] <= self.last_sample: return (0, 0)
        self.last_sample = sample['captured_at']
        bearing = math.atan2(xyz[1], xyz[2])-math.atan2(target[1], target[2])
        if abs(bearing) > math.radians(3):
            self.stable = 0
            pulse_duration = .12
            # Empirically verified: positive camera Y error requires a right turn.
            self.wheels = (self.pwm, -self.pwm) if bearing > 0 else (-self.pwm, self.pwm)
        elif distance > target_distance+DISTANCE_TOLERANCE_MM:
            self.stable = 0
            error=distance-target_distance
            speed=90 if error>60 else self.pwm
            self.wheels = (speed, speed)
            pulse_duration=.30 if error>60 else .20 if error>50 else .10
            if error>60 and odom is not None and odom.imu_valid:
                self.forward_left=odom.left;self.forward_right=odom.right
                self.forward_yaw=odom.imu_yaw;self.forward_segment=odom.imu_segment
                self.forward_revision=odom.revision
                self.phase='forward';self.until=now+.9
                return self.wheels
        else:
            self.stable += 1
            if self.stable >= 4:
                current_r, _ = cv2.Rodrigues(rvec)
                target_r, _ = cv2.Rodrigues(np.asarray(self.ref['rvec'], dtype=float))
                angle = math.degrees(math.acos(float(np.clip((np.trace(target_r.T @ current_r)-1)/2, -1, 1))))
                if self.require_orientation and angle > 12: raise RuntimeError(f'Posición alcanzada, pero orientación difiere {angle:.1f}°; ajusta manualmente')
                normal_angle = math.degrees(math.acos(float(np.clip(abs(current_r[2,2]),0,1))))
                if normal_angle > 10: raise RuntimeError(f'Marcador inclinado {normal_angle:.1f}° respecto a la cámara; ajusta la posición manualmente')
                self.done = True
                self.message = f'LISTO PARA EXTRACCIÓN: alineación visual estimada; confirma distancia real ({distance:.0f} mm en cámara) y posición antes de continuar'
            return (0, 0)
        self.phase = 'pulse'; self.until = now+pulse_duration
        self.pulses += 1
        if self.pulses > 120: raise RuntimeError('Límite de correcciones alcanzado')
        return self.wheels
