"""Bounded routine blocks; physical arm and camera poses are estimates."""
import math
from aruco_alignment import ArucoAlignment


class ArmRoutineBlock:
    kind='arm_block'
    def __init__(self,engine,kind,value,now):
        self.engine=engine;self.started=now;self.done=False;self.seen_busy=False
        self.duration=value['duration_ms']/1000
        angles=value.get('joints')
        if kind=='arm_xyz':
            ok,angles,detail=engine.arm_model.compute_ik(value['xyz'])
            if not ok:raise ValueError(f'Bloque XYZ sin solución: {detail}')
        self.target=[round(float(angles[0])),round(float(angles[1])),round(float(angles[2])),
                     round(value['rotation']),round(float(angles[3])),round(value['gripper'])]
        engine.send_arm_pose(angles,value['gripper'],value['duration_ms'],'PC',value['rotation'],routine_owned=True)

    def output(self,odom,now):
        if now-self.started>self.duration+3:raise RuntimeError('Bloque brazo sin confirmación de telemetría')
        channels=self.engine.telemetry.get('arm_angles')
        if not isinstance(channels,list) or len(channels)!=6 or any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in channels):
            raise RuntimeError('Bloque brazo: telemetría inválida')
        busy=bool(self.engine.telemetry.get('arm_busy',0))
        if busy:self.seen_busy=True
        if self.seen_busy and not busy and now-self.started>=self.duration and all(abs(a-b)<=1 for a,b in zip(channels,self.target)):
            self.done=True
        return 0,0


class ArucoMarkerUnavailable(RuntimeError):
    """A routine can retry this visual step from the current position."""


class ArucoRoutineBlock:
    kind='aruco_block'
    def __init__(self,engine,reference,now):
        self.engine=engine;self.done=False
        self.controller=ArucoAlignment(reference,now,engine.min_pwm,require_new_sample=True,
                                       allow_marker_quarter_turns=True,accept_closer_distance=True)
        self.engine.aruco_sample=None
    def output(self,odom,now):
        if not odom.imu_valid:raise RuntimeError('Bloque ArUco detenido: IMU no válida')
        sample=self.engine.aruco_sample
        if sample is not None and sample.get('identity')!=self.controller.ref.get('identity'):
            raise RuntimeError('Bloque ArUco detenido: cambió la cámara o su configuración')
        output=self.controller.output(sample,now,odom)
        last_sample=self.controller.last_valid_sample
        last_seen=last_sample['captured_at'] if last_sample else self.controller.started
        if now-last_seen>=2.0:
            raise ArucoMarkerUnavailable('ArUco 12 sin lectura válida.')
        self.done=self.controller.done
        return output
