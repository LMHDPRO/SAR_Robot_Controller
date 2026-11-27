"""Conservative, pulse-based approach to a taught marker pose."""
import math
import cv2
import numpy as np

# The stream has no camera exposure timestamp. Allow the reported ~1 s
# pipeline delay to clear after motion before acting on another observation.
ASSUMED_CAMERA_LATENCY_S = 1.0
# Local estimate for 180 mm physical clearance: camera read 217.2 mm at 200 mm.
TARGET_DISTANCE_MM = 197.2
DISTANCE_TOLERANCE_MM = 5.0
APPROACH_BEARING_TOLERANCE_DEG = 3.0
FINAL_BEARING_TOLERANCE_DEG = 1.0
FINE_TURN_PULSE_S = 0.075


class ArucoAlignment:
    def __init__(self, reference, now, pwm, require_orientation=True, require_new_sample=False, *, allow_marker_quarter_turns=False, accept_closer_distance=False):
        self.ref = reference
        self.target_distance_mm = float(reference.get('target_distance_mm', TARGET_DISTANCE_MM))
        if not 0 < self.target_distance_mm <= 600:
            raise ValueError('Distancia objetivo ArUco fuera de rango')
        self.require_orientation = require_orientation
        self.allow_marker_quarter_turns = allow_marker_quarter_turns
        self.accept_closer_distance = accept_closer_distance
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
        target_distance = self.target_distance_mm
        too_close = distance < target_distance-DISTANCE_TOLERANCE_MM
        if too_close and not self.accept_closer_distance:
            raise RuntimeError(f'Demasiado cerca del marcador ({distance:.1f} mm de cámara, objetivo {target_distance:.1f} ± {DISTANCE_TOLERANCE_MM:g} mm); mueve el AGV manualmente')
        if distance > 600: raise RuntimeError('Marcador demasiado lejos para esta aproximación')
        if self.phase == 'forward':
            valid=(odom is not None and odom.imu_valid and odom.imu_segment==self.forward_segment
                   and odom.revision==self.forward_revision)
            travel=abs(((odom.left-self.forward_left)+(odom.right-self.forward_right))/2) if valid else 0
            yaw=abs(odom.imu_yaw-self.forward_yaw) if valid else 0
            if (valid and now<self.until and travel<self.forward_travel_limit
                    and yaw<4 and distance>self.forward_stop_distance):
                return self.wheels
            self.phase='settle';self.until=now+.25+ASSUMED_CAMERA_LATENCY_S
            return (0,0)
        if self.phase == 'pulse':
            if now < self.until and not (too_close and self.wheels[0]==self.wheels[1]): return self.wheels
            self.phase = 'settle'; self.until = now+.25+ASSUMED_CAMERA_LATENCY_S
            return (0, 0)
        if self.phase == 'settle':
            if now < self.until: return (0, 0)
            if sample['captured_at'] < self.until: return (0, 0)
            self.phase = 'observe'
        if sample['captured_at'] <= self.last_sample: return (0, 0)
        self.last_sample = sample['captured_at']
        bearing = math.degrees(math.atan2(xyz[1], xyz[2])-math.atan2(target[1], target[2]))
        at_target_distance = distance <= target_distance+DISTANCE_TOLERANCE_MM
        bearing_tolerance = FINAL_BEARING_TOLERANCE_DEG if at_target_distance else APPROACH_BEARING_TOLERANCE_DEG
        if abs(bearing) > bearing_tolerance:
            self.stable = 0
            pulse_duration = FINE_TURN_PULSE_S if at_target_distance and abs(bearing)<=2.5 else .12
            # Empirically verified: positive camera Y error requires a right turn.
            self.wheels = (self.pwm, -self.pwm) if bearing > 0 else (-self.pwm, self.pwm)
        elif not at_target_distance:
            self.stable = 0
            error=distance-target_distance
            speed=90 if error>60 else self.pwm
            self.wheels = (speed, speed)
            pulse_duration=.30 if error>60 else .20 if error>50 else .10
            if error>30 and odom is not None and odom.imu_valid:
                self.forward_left=odom.left;self.forward_right=odom.right
                self.forward_yaw=odom.imu_yaw;self.forward_segment=odom.imu_segment
                self.forward_revision=odom.revision
                self.forward_travel_limit=20 if error>60 else 15
                self.forward_stop_distance=target_distance+(60 if error>60 else 20)
                self.phase='forward';self.until=now+(.9 if error>60 else .35)
                return self.wheels
        else:
            self.stable += 1
            if self.stable >= 4:
                current_r, _ = cv2.Rodrigues(rvec)
                target_r, _ = cv2.Rodrigues(np.asarray(self.ref['rvec'], dtype=float))
                relative = target_r.T @ current_r
                trace = np.trace(relative)
                if self.allow_marker_quarter_turns:
                    # Maximize the trace over 0°, 90°, 180° and 270° about the marker normal.
                    in_plane = relative[0,0]+relative[1,1]
                    cross = relative[1,0]-relative[0,1]
                    trace = relative[2,2]+max(abs(in_plane),abs(cross))
                angle = math.degrees(math.acos(float(np.clip((trace-1)/2, -1, 1))))
                if self.require_orientation and angle > 12:
                    raise RuntimeError(f'Centrado horizontal {bearing:+.1f}°; orientación del marcador difiere {angle:.1f}° de la referencia. '
                                       'Verifica la pose de recogida y vuelve a guardar la referencia ArUco 12.')
                normal_angle = math.degrees(math.acos(float(np.clip(abs(current_r[2,2]),0,1))))
                if normal_angle > 10: raise RuntimeError(f'Marcador inclinado {normal_angle:.1f}° respecto a la cámara; ajusta la posición manualmente')
                self.done = True
                self.message = (f'LISTO PARA EXTRACCIÓN: {distance:.0f} mm en cámara, error horizontal {bearing:+.1f}°. '
                                'Confirma la distancia real y la posición antes de continuar.')
            return (0, 0)
        self.phase = 'pulse'; self.until = now+pulse_duration
        self.pulses += 1
        if self.pulses > 120: raise RuntimeError('Límite de correcciones alcanzado')
        return self.wheels
