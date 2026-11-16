import csv, json, math, socket, threading, time, queue
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from collections import deque
from dataclasses import asdict
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
from lidar_workspace import LidarWorkspace
from arm import ArmKinematics
from defaults import RUNTIME_DEFAULTS, GEOMETRY_DEFAULTS
from core import Wait, distance_scale, Geometry, Odometry, Primitive, SingleEncoderMove, ImuTurn, WheelCapture, wheel_correction, validate_routine

class Link:
    def __init__(self,host,port):
        self.socket=socket.create_connection((host,port),timeout=.35)
        self.socket.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_KEEPALIVE,1)
        self.socket.setblocking(False);self.buffer=b'';self.out=b'';self.pending_at=0
    def flush(self):
        if not self.out:return
        if time.monotonic()-self.pending_at>.3:raise ConnectionError('TX stalled; stale commands discarded')
        try:n=self.socket.send(self.out)
        except BlockingIOError:return
        if not n:raise ConnectionError('Socket send closed')
        self.out=self.out[n:]
    def send(self,data):
        if not self.out:self.pending_at=time.monotonic()
        self.out+=data.encode()
        if len(self.out)>16384:raise ConnectionError('TX backlog exceeded')
        self.flush()
    def urgent(self,data):
        self.out=b'';self.pending_at=0
        self.send("\n"+data)
    def receive(self):
        self.flush()
        try:data=self.socket.recv(8192)
        except BlockingIOError:return []
        if not data:raise ConnectionError('Remote closed connection')
        self.buffer+=data
        if len(self.buffer)>65536:raise ConnectionError('RX buffer exceeded')
        lines=self.buffer.split(b'\n');self.buffer=lines.pop();result=[]
        for line in lines:
            try:
                item=json.loads(line)
                if isinstance(item,dict):result.append(item)
            except (ValueError,UnicodeError):continue
        return result
    def close(self):
        try:self.socket.close()
        except OSError:pass

class Signals(QtCore.QObject):
    cancelled=QtCore.Signal()
    state=QtCore.Signal(object)
    message=QtCore.Signal(str)

class Engine(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True);self.signals=Signals();self.commands=queue.Queue();self.quit=False
        self.robot=self.pad=None;self.g=Geometry();self.odom=Odometry();self.telemetry={}
        self.arm_model=ArmKinematics();self.arm_angles=self.arm_model.startup_angles.copy();self.arm_gripper=75.0;self.arm_rotation=90.0
        self.arm_target_angles=self.arm_angles.copy();self.arm_target_gripper=self.arm_gripper
        self.arm_target_rotation=self.arm_rotation
        self.arm_busy=False;self.arm_error='';self.coco_arm_at=None;self.coco_arm_z=None;self.coco_arm_grip=None
        self.last_robot=self.last_pad=0;self.pad_data={};self.pad_base=None;self.pad_released=False;self.pad_authorized=False
        self.live=None;self.primitive=None;self.source='PC';self.last_ui=time.monotonic();self.path=deque(maxlen=10000)
        self.sent_pwm=(0,0)
        self.estop_latched=False;self.reset_pending=False;self.epoch=0;self.urgent_commands=queue.Queue()
        self.pool=ThreadPoolExecutor(max_workers=2);self.pending_links={};self.link_generation={'robot':0,'pad':0}
        self.runtime=dict(RUNTIME_DEFAULTS)
        try:self.runtime.update(json.loads(Path(__file__).with_name('runtime.json').read_text()))
        except (OSError,ValueError):pass
        self.distance_reference=None;self.distance_result=None;self.sequence_status='Sin ruta'
        try:self.distance_result=json.loads(Path(__file__).with_name('settings.json').read_text()).get('_distance_calibration')
        except (OSError,ValueError,AttributeError):pass

        self.rows=deque(maxlen=60000);self.test_result='No test yet';self.stopped=True;self.last_boot=None;self.pad_ready=False;self.robot_host=self.pad_host=None;self.robot_connected_at=0;self.retry_at=0;self.wheel_mark=None;self.turn_mark=None;self.turn_delta=0.0;self.turn_origin=0.0;self.was_turning=False;self.turn_settle_until=0;self.routine=deque();self.routine_total=0;self.routine_step=0;self.next_step_at=0;self.routine_cap=85;self.last_wheel_test=None;self.capture=None;self.min_pwm=int(self.runtime["min_pwm"])
    def post(self,kind,value=None):
        if kind in ('stop','estop'):
            self.epoch+=1
            self.urgent_commands.put((kind,value,self.epoch))
        else:self.commands.put((kind,value,self.epoch))
    def save_runtime(self):
        path=Path(__file__).with_name('runtime.json');temp=path.with_suffix('.tmp')
        temp.write_text(json.dumps(self.runtime,indent=2));temp.replace(path)
    def request_link(self,which,ip=None):
        if ip is not None:
            self.link_generation[which]+=1
            self.halt()
            old=getattr(self,which)
            if old:old.close()
            setattr(self,which,None);setattr(self,which+'_host',ip)
            self.runtime[which+'_ip']=ip;self.save_runtime()
            if which=='robot':self.telemetry={};self.last_robot=0;self.distance_reference=None
            else:self.pad_data={};self.last_pad=0
        if which in self.pending_links:return
        host=getattr(self,which+'_host')
        if host:self.pending_links[which]=(self.link_generation[which],self.pool.submit(Link,host,4211 if which=='robot' else 4210))
    def poll_links(self):
        for which,(generation,future) in list(self.pending_links.items()):
            if not future.done():continue
            del self.pending_links[which]
            try:link=future.result()
            except OSError as e:
                self.emit(f'{which}: reconexión pendiente ({type(e).__name__})');continue
            if generation!=self.link_generation[which]:link.close();continue
            setattr(self,which,link)
            if which=='robot':
                self.robot_connected_at=time.monotonic();self.last_robot=0;self.telemetry={}
                link.send(('ESTOP' if self.estop_latched else 'STOP')+'\n'+f"POLICY:{int(self.runtime['cut_relays_on_loss'])}\nSTATUS\n")
            else:
                self.last_pad=time.monotonic();self.pad_data={};self.power_seen=None
                self.pad_authorized=False;self.pad_released=False
                self.coco_arm_at=None;self.coco_arm_z=None;self.coco_arm_grip=None
            self.emit(f'{which}: conectado; movimiento requiere una orden nueva')

    def emit(self,text):self.signals.message.emit(text)
    def motion_block(self):
        if self.estop_latched or self.telemetry.get('estop')==1:return 'E-STOP enclavado: desbloquear primero'
        if self.source!='PC':return 'Select PC to use screen controls'
        if not self.robot:return 'AGV disconnected'
        if time.monotonic()-self.last_robot>=.5:return 'AGV telemetry stale; reconnect AGV'
        if self.telemetry.get('estop')==1:return 'Robot emergency stop is active'
        if self.telemetry.get('pwm_ok')==0:return 'Robot reports PWM initialization failure'
        if self.telemetry.get('protocol')!=1:return 'Cargar firmware SAR v1.0 en el AGV'
        if not self.telemetry.get('armed'):return 'Relays OFF: press POWER ON'
        if self.arm_busy:return 'STOP el brazo antes de mover el AGV'
        if self.capture:return 'Wheel capture still active: press STOP and wait for its result'
        return ''
    def halt(self,disarm=False):
        self.signals.cancelled.emit()
        if self.capture:self.capture.stop('Stopped: partial capture retained',time.monotonic())
        self.routine.clear();self.live=None;self.primitive=None;self.pad_authorized=False;self.pad_released=False;self.stopped=True
        self.arm_target_angles=self.arm_angles.copy();self.arm_target_gripper=self.arm_gripper
        self.arm_target_rotation=self.arm_rotation;self.arm_busy=False
        if self.robot:
            try:
                send=getattr(self.robot,'urgent',self.robot.send)
                send('ESTOP\n' if self.estop_latched and not self.reset_pending else 'POWER:0\n' if disarm else 'STOP\n');self.sent_pwm=(0,0)
            except OSError:pass
    def arm_motion_block(self,owner):
        if self.estop_latched or self.telemetry.get('estop')==1:return 'E-STOP enclavado: desbloquear primero'
        if self.source!=owner:return f'Seleccionar {owner} como propietario del control'
        if not self.robot:return 'AGV disconnected'
        if time.monotonic()-self.last_robot>=.5:return 'AGV telemetry stale; reconnect AGV'
        if self.telemetry.get('protocol')!=1:return 'Cargar firmware SAR v1.0 en el AGV'
        if not self.telemetry.get('armed'):return 'Relays OFF: press POWER ON'
        if self.telemetry.get('pwm_ok')==0:return 'Robot reports PWM initialization failure'
        if self.telemetry.get('pca_ready')!=1:return 'PCA9685 not ready on AGV'
        if self.telemetry.get('arm_protocol')!=1:return 'Cargar firmware SAR con soporte del brazo'
        return ''
    def send_arm_pose(self,angles,gripper,duration_ms,owner,rotation=None):
        reason=self.arm_motion_block(owner)
        if reason:raise ValueError(reason)
        if owner=='PC' and (self.live or self.primitive or self.routine or self.capture):
            raise ValueError('STOP drivetrain motion before moving the arm from PC')
        target=np.asarray(angles,dtype=float)
        if target.shape!=(4,) or not np.all(np.isfinite(target)) or np.any(target<0) or np.any(target>180):
            raise ValueError('Arm joint angles must be finite and between 0 and 180 degrees')
        grip=float(gripper)
        wrist_rotation=self.arm_target_rotation if rotation is None else float(rotation)
        duration=int(duration_ms)
        if not math.isfinite(grip) or not 10<=grip<=75:raise ValueError('Gripper angle must be between 10 and 75 degrees')
        if not math.isfinite(wrist_rotation) or not 0<=wrist_rotation<=180:raise ValueError('Wrist rotation angle must be between 0 and 180 degrees')
        if not 100<=duration<=10000:raise ValueError('Arm move duration must be between 100 and 10000 ms')
        channels=[int(round(target[0])),int(round(target[1])),int(round(target[2])),int(round(wrist_rotation)),int(round(target[3])),int(round(grip))]
        self.robot.send(f"ARM:{','.join(map(str,channels))},{duration}\n")
        self.arm_target_angles=target.copy();self.arm_target_gripper=grip
        self.arm_target_rotation=wrist_rotation
        self.arm_model.last_ik_theta=self.arm_model.convert_angles(target)
        self.arm_busy=True;self.arm_error=''
        if owner=='PC':self.emit('Pose arm transmitida al AGV por la conexión existente')
    def update_arm_joints_from_coco(self,p,now):
        reason=self.arm_motion_block('COCO')
        if reason:
            self.arm_error=reason
            return
        joint_index=p.get('arm_joint_index')
        axis=p.get('arm_filtered_y',0)
        if type(joint_index) is not int or not 0<=joint_index<6:
            self.arm_error='Invalid COCO arm joint selection'
            return
        if not isinstance(axis,(int,float)) or not math.isfinite(axis):
            self.arm_error='Invalid COCO arm joystick data'
            return
        elapsed=0.0 if self.coco_arm_at is None else min(.1,max(0.0,now-self.coco_arm_at))
        self.coco_arm_at=now
        delta=max(-100.0,min(100.0,float(axis)))/100.0*30.0*elapsed
        if abs(delta)<.001:return
        angles=self.arm_target_angles.copy()
        rotation=self.arm_target_rotation
        gripper=self.arm_target_gripper
        if joint_index<3:
            angles[joint_index]=max(0.0,min(180.0,angles[joint_index]+delta))
        elif joint_index==3:
            rotation=max(0.0,min(180.0,rotation+delta))
        elif joint_index==4:
            angles[3]=max(0.0,min(180.0,angles[3]+delta))
        else:
            gripper=max(10.0,min(75.0,gripper+delta))
        if (np.max(np.abs(angles-self.arm_target_angles))<.5
                and abs(rotation-self.arm_target_rotation)<.5
                and abs(gripper-self.arm_target_gripper)<.5):
            return
        try:
            self.send_arm_pose(angles,gripper,150,'COCO',rotation)
        except ValueError as exc:
            self.arm_error=str(exc)
    def update_arm_from_coco(self,p,now):
        if p.get('mode')=='A':
            self.update_arm_joints_from_coco(p,now)
            return
        reason=self.arm_motion_block('COCO')
        if reason:
            self.arm_error=reason
            return
        self.arm_error=''
        axes=[]
        for key in ('arm_filtered_x','arm_filtered_y'):
            value=p.get(key,0)
            if not isinstance(value,(int,float)) or not math.isfinite(value):
                self.arm_error='Invalid COCO arm joystick data'
                return
            axes.append(max(-100.0,min(100.0,float(value)))/100.0)
        try:
            z_request=float(p.get('z_change_mm',0.0))
            grip_request=float(p.get('grip_change_pct',0.0))
        except (TypeError,ValueError):
            self.arm_error='Invalid COCO arm encoder data'
            return
        if not math.isfinite(z_request) or not math.isfinite(grip_request):
            self.arm_error='Invalid COCO arm encoder data'
            return
        z_delta=0.0 if self.coco_arm_z is None else z_request-self.coco_arm_z
        grip_delta=0.0 if self.coco_arm_grip is None else grip_request-self.coco_arm_grip
        self.coco_arm_z=z_request;self.coco_arm_grip=grip_request
        elapsed=0.0 if self.coco_arm_at is None else min(.1,max(0.0,now-self.coco_arm_at))
        self.coco_arm_at=now
        if max(abs(axes[0]),abs(axes[1]))<.01 and abs(z_delta)<.001 and abs(grip_delta)<.001:
            return
        target_angles=self.arm_target_angles.copy()
        x,y,z=self.arm_model.compute_fk(target_angles)[0]
        gripper=self.arm_target_gripper
        arm_mode=p.get('arm_mode','XY')
        if arm_mode=='XY':
            target_xyz=np.array([x+axes[0]*25.0*elapsed,y+axes[1]*25.0*elapsed,z])
        elif arm_mode=='RX_RY':
            target_angles[3]=max(0.0,min(180.0,target_angles[3]-axes[1]*30.0*elapsed))
            gripper=max(10.0,min(75.0,gripper-axes[0]*30.0*elapsed))
            target_xyz=self.arm_model.compute_fk(target_angles)[0]
        else:
            self.arm_error='Unknown COCO arm mode'
            return
        encoder_mode=p.get('encoder_mode','Z')
        if encoder_mode=='Z':
            target_xyz[2]=max(0.0,min(self.arm_model.max_reach,target_xyz[2]+z_delta))
        elif encoder_mode=='GRIP':
            gripper=max(10.0,min(75.0,gripper+grip_delta*.65))
        else:
            self.arm_error='Unknown COCO arm encoder mode'
            return
        target_xyz=self.arm_model.enforce_safety_zone(target_xyz)
        needs_ik=(arm_mode=='XY' and (max(abs(axes[0]),abs(axes[1]))>=.01 or abs(z_delta)>=.001)
                  or arm_mode=='RX_RY' and abs(z_delta)>=.001)
        if needs_ik:
            if arm_mode=='RX_RY':self.arm_model.last_ik_theta=self.arm_model.convert_angles(target_angles)
            success,angles,detail=self.arm_model.compute_ik(target_xyz)
            if not success:
                self.arm_error=f'COCO arm target rejected: {detail}'
                return
        else:
            angles=target_angles
        if (np.max(np.abs(angles-self.arm_target_angles))<.5
                and abs(gripper-self.arm_target_gripper)<.5):
            return
        try:
            self.send_arm_pose(angles,gripper,150,'COCO')
        except ValueError as exc:
            self.arm_error=str(exc)
    def handle_arm_pose(self,value):
        angles,gripper,duration=value
        self.send_arm_pose(angles,gripper,duration,'PC')
    def halt_loss(self):
        self.distance_reference=None
        self.halt(bool(self.runtime['cut_relays_on_loss']))
    def handle(self,k,v):
        if k=='heartbeat':self.last_ui=time.monotonic();return
        if k=='arm_camera_url':self.runtime['arm_camera_url']=str(v);self.save_runtime()
        elif k=='connect_robot':self.request_link('robot',v)
        elif k=='connect_pad':self.request_link('pad',v)
        elif k=='estop':
            self.estop_latched=True;self.reset_pending=False;self.halt(True);self.emit('E-STOP: PWM cero, relays OFF, bloqueo activo')
        elif k=='estop_reset':
            if not self.robot or time.monotonic()-self.last_robot>.5:raise ValueError('Conectar AGV para desbloquear')
            if self.telemetry.get('physical_estop')==1:raise ValueError('Liberar el paro físico primero')
            self.halt(True);self.robot.send('RESET_ESTOP\n');self.reset_pending=True
            self.emit('Desbloqueo solicitado; POWER ON sigue siendo manual')
        elif k=='policy':
            self.halt();self.runtime['cut_relays_on_loss']=bool(v);self.save_runtime()
            if self.robot:self.robot.send(f'POLICY:{int(bool(v))}\n')
            self.emit('Política guardada; verificar confirmación en telemetría')
        elif k=='distance_begin':
            if self.live or self.primitive or self.capture or self.routine:raise ValueError('STOP before marking distance')
            if not self.robot or time.monotonic()-self.last_robot>.5:raise ValueError('Connect AGV first')
            side=v or 'both'
            if side not in ('left','right','both'):raise ValueError('Select Left or Right encoder')
            if side!='both' and not self.odom.imu_valid:raise ValueError('Fresh IMU required for distance calibration')
            self.distance_reference=dict(left=self.odom.left,right=self.odom.right,boot=self.telemetry['boot_id'],
                                         side=side,yaw=self.odom.imu_yaw,segment=self.odom.imu_segment,valid=True)
            self.emit('Start marked: travel 50 cm indicated, STOP, then measure actual distance')
        elif k=='distance_test':
            side,cap=v
            self.handle('distance_begin',side)
            try:self.start_test('distance_imu',50,cap,side)
            except Exception:
                self.distance_reference=None
                raise
        elif k=='distance_apply':
            if self.live or self.primitive or self.capture or self.routine:raise ValueError('STOP before applying calibration')
            if not self.distance_reference:raise ValueError('Mark the start first')
            if not self.robot or time.monotonic()-self.last_robot>.5:raise ValueError('Fresh telemetry required')
            ref=self.distance_reference;side=ref['side']
            if ref['boot']!=self.telemetry['boot_id']:raise ValueError('AGV rebooted: repeat calibration')
            dl=self.odom.left-ref['left'];dr=self.odom.right-ref['right']
            if side=='both':
                if dl*dr<=0 or abs(dl-dr)>0.15*max(abs(dl),abs(dr)):raise ValueError('Repeat a straight run without turning')
                indicated=abs((dl+dr)/20)
            else:
                if not ref['valid'] or not self.odom.imu_valid or self.odom.imu_segment!=ref['segment'] or abs(self.odom.imu_yaw-ref['yaw'])>5:
                    raise ValueError('IMU lost or heading changed: repeat the straight calibration run')
                indicated=abs(dl if side=='left' else dr)/10
            if indicated<49:raise ValueError('Complete the 50 cm indicated run first (1 cm stopping tolerance)')
            measured=float(v);ratio=distance_scale(measured,indicated)
            new=Geometry(**asdict(self.g))
            if side in ('left','both'):new.scale_l*=ratio
            if side in ('right','both'):new.scale_r*=ratio
            new.validate()
            old_g,old_report=self.g,self.distance_result
            self.g=new;self.distance_result=dict(side=side,requested_cm=50,indicated_cm=indicated,measured_cm=measured,
                                               ratio=ratio,scale_l=new.scale_l,scale_r=new.scale_r,saved_unix_s=time.time())
            try:self.save_settings()
            except OSError:
                self.g,self.distance_result=old_g,old_report
                raise
            self.distance_reference=None
            choice=self.odom.prefer_imu;self.odom.reset();self.odom.prefer_imu=choice;self.path.clear()
            self.emit(f'Saved {side} distance scale x{ratio:.4f} to settings.json; repeat 50 cm to verify')
        elif k=='stop':self.halt(bool(v))
        elif k=='source':
            self.halt();self.source=v;self.pad_base=None
            self.coco_arm_at=None;self.coco_arm_z=None;self.coco_arm_grip=None
        elif k=='arm_pose':self.handle_arm_pose(v)
        elif k=='power':
            if v and (self.estop_latched or self.telemetry.get('estop')==1):raise ValueError('Desbloquear E-STOP primero')
            if v and self.telemetry.get('protocol')!=1:raise ValueError('Cargar firmware SAR v1.0')
            self.halt(not v)
            if v and self.robot and time.monotonic()-self.last_robot<0.5:
                self.robot.send('POWER:1\n');self.stopped=False
        elif k=='manual':
            if v is None:self.halt()
            else:
                reason=self.motion_block()
                if reason:raise ValueError('Movement blocked: '+reason)
                self.routine.clear();self.primitive=None;self.live=v;self.stopped=False
                self.test_result=f'Manual PWM L={v[0]}, R={v[1]} (hold button to move)';self.emit(self.test_result)
        elif k=='prefer_imu':
            if self.live or self.primitive or self.routine or self.capture:raise ValueError('STOP before switching heading source')
            self.odom.prefer_imu=bool(v);self.emit('Heading source changed; reset pose while stopped if needed')
        elif k=='wheel_mark':
            if self.live or self.primitive or self.telemetry.get('armed'):raise ValueError('DISARM, keep telemetry connected, then mark wheel counts')
            if not self.robot or time.monotonic()-self.last_robot>.5:raise ValueError('Fresh robot telemetry required')
            self.wheel_mark=(self.telemetry['tl'],self.telemetry['tr']);self.emit('Wheel mark captured. Manually rotate selected wheel exactly N revolutions.')
        elif k=='wheel_finish':
            if self.wheel_mark is None or self.telemetry.get('armed') or not self.robot or time.monotonic()-self.last_robot>.5:raise ValueError('Fresh telemetry, DISARM and a wheel mark required')
            count=float(v)
            if count<=0:raise ValueError('Revolutions must be positive')
            dl=self.odom.tick_delta(self.telemetry['tl'],self.wheel_mark[0]);dr=self.odom.tick_delta(self.telemetry['tr'],self.wheel_mark[1]);dl,dr=self.g.wheel_ticks(dl,dr)
            self.test_result=f'Wheel measurement ({count:g} physical revolutions): delta L={dl}, R={dr}; candidate TPR L={abs(dl)/count:.3f}, R={abs(dr)/count:.3f}. Apply ONLY the wheel actually rotated. No automatic changes.'
            self.emit(self.test_result)
        elif k=='wheel_test':
            reason=self.motion_block()
            if reason:raise ValueError(reason)
            side,revs,pwm=v
            if self.source!='PC' or not self.robot or time.monotonic()-self.last_robot>.6 or not self.telemetry.get('armed'):
                raise ValueError('Connect AGV, choose PC and switch POWER ON')
            if self.live or self.primitive or self.routine or self.capture:raise ValueError('Stop the current action first')
            self.last_wheel_test=None
            self.capture=WheelCapture(side,revs,self.telemetry,Geometry(**asdict(self.g)),time.monotonic(),pwm)
            self.test_result='Wheel capture running. STOP keeps partial ticks. Map paused for this bench test.'
        elif k=='wheel_apply':
            observed,forward=v
            if self.capture or self.live or self.primitive or self.routine:raise ValueError('Stop and wait for capture to finish')
            tpr,sign,motor=wheel_correction(self.last_wheel_test,observed,forward)
            side='l' if self.last_wheel_test['side']=='left' else 'r'
            setattr(self.g,'tpr_'+side,tpr);setattr(self.g,'sign_'+side,sign);setattr(self.g,'motor_'+side,motor)
            self.g.validate();self.save_settings();self.odom.reset();self.path.clear()
            self.test_result=f"APPLIED & SAVED {side.upper()}: {tpr:.3f} ticks/rev, encoder sign {sign:+d}, motor sign {motor:+d}. Map reset."
            self.emit(self.test_result)
        elif k=='min_pwm':
            self.min_pwm=max(1,min(int(v),180));self.runtime['min_pwm']=self.min_pwm;self.save_runtime()
        elif k=='imu_flip':
            self.halt();self.g.imu_sign*=-1;self.odom.reset();self.path.clear();self.save_settings()
            self.emit(f'IMU direction reversed: {self.g.imu_sign:+d}; map reset and setting saved')
        elif k=='routine':
            steps,cap=v
            steps=validate_routine(steps)
            if self.source!='PC':raise ValueError('Select PC ownership for routines')
            if self.live or self.primitive or self.routine or self.capture:raise ValueError('STOP before starting a routine')
            self.start_test(steps[0][0],steps[0][1],cap)
            self.routine=deque(steps[1:]);self.routine_total=len(steps);self.routine_step=1;self.routine_cap=cap
            self.emit(f'Routine started: {len(steps)} steps; no automatic resume after interruption')
        elif k=='primitive':
            if self.routine:raise ValueError('STOP the running routine first')
            if self.source=='PC':self.start_test(*v)
        elif k=='geometry':
            self.distance_reference=None
            if self.live or self.primitive or self.routine or self.capture:raise ValueError('STOP before changing geometry')
            v.validate();self.g=v;self.save_settings();choice=self.odom.prefer_imu;self.odom.reset();self.odom.prefer_imu=choice;self.path.clear();self.emit('Settings applied and saved; map reset')
        elif k=='reset':
            self.distance_reference=None
            if self.live or self.primitive or self.routine or self.capture:raise ValueError('STOP before resetting pose')
            choice=self.odom.prefer_imu;self.odom.reset();self.odom.prefer_imu=choice;self.path.clear();self.emit('Pose zeroed; raw encoders unchanged')
        elif k=='export':
            with open(v,'w',newline='',encoding='utf8') as f:
                writer=csv.writer(f);writer.writerow(['host_monotonic_s','robot_ms','ticks_l','ticks_r','x_mm','y_mm','yaw_left_deg','imu_relative_deg','speed_mm_s','pwm_l','pwm_r','encoder_yaw_deg','yaw_source','imu_valid','left_increment_mm','right_increment_mm']);writer.writerows(list(self.rows))
            self.emit('CSV exported')
    def save_settings(self):
        path=Path(__file__).with_name('settings.json');temporary=path.with_suffix('.tmp')
        data=asdict(self.g)
        if self.distance_result:data['_distance_calibration']=self.distance_result
        temporary.write_text(json.dumps(data,indent=2));temporary.replace(path)
    def start_test(self,kind,value,cap,encoder_side=None):
        reason=self.motion_block()
        if reason:raise ValueError(reason)
        if not self.robot or time.monotonic()-self.last_robot>0.5 or not self.telemetry.get('armed'):
            raise ValueError('Fresh robot telemetry and power ON required')
        if self.primitive or self.live or self.capture:raise ValueError('STOP / complete the current motion first')
        if kind=='distance_imu':
            self.primitive=SingleEncoderMove(value,encoder_side,self.odom,time.monotonic(),cap,self.min_pwm)
        else:
            self.primitive=(Wait(value,self.odom,time.monotonic()) if kind=='wait' else ImuTurn(value,self.odom,self.g,time.monotonic(),cap,self.min_pwm,precise=kind=='imu_precise') if kind in ('turn','imu_turn','imu_precise') else Primitive(kind,value,self.odom,self.g,time.monotonic(),cap,self.min_pwm))
        self.stopped=False
        if kind in ('turn','imu_turn','imu_precise'):self.turn_mark=(self.odom.left,self.odom.right,self.odom.imu_yaw)
        if kind=='distance_imu':
            self.test_result=f'Running {value:g} cm with {encoder_side} encoder and IMU heading hold'
        else:self.test_result=f'Running {kind}: {value}'
    def process_pad(self, p, now):
        if p.get('type') != 'coco':
            return

        if self.estop_latched or self.telemetry.get('estop')==1:
            self.halt(True);return
        if now-self.last_ui>.6:
            self.halt_loss();return
        previous = self.pad_data
        if (previous and p.get('boot_id') == previous.get('boot_id')
                and p.get('seq') == previous.get('seq')):
            return

        restarted = bool(previous and p.get('boot_id') != previous.get('boot_id'))
        self.pad_data = dict(p)
        self.last_pad = now
        pair = (p.get('boot_id'), p.get('power_request_id', 0))
        if restarted:
            if self.source == 'COCO':
                self.halt(False)
            self.power_seen = pair
            return

        if getattr(self, 'power_seen', None) is None:
            self.power_seen = pair
        if pair != self.power_seen:
            self.power_seen = pair
            if p.get('power_pending') and self.source == 'COCO':
                self.handle('power', bool(p.get('power_enable')))

        active = (self.source == 'COCO' and p.get('mode') in ('A','B','C')
                  and not p.get('menu_open') and not p.get('debug_open'))
        centered = all(abs(p.get(k, 0)) <= 10 for k in (
            'agv_filtered_x', 'agv_filtered_y',
            'arm_filtered_x', 'arm_filtered_y'))

        if active and not p.get('deadman'):
            self.pad_released = True

        if not active or not p.get('motion_enabled') or not p.get('deadman'):
            if self.source == 'COCO':
                released = self.pad_released
                self.halt()
                self.pad_released = bool(active and not p.get('deadman') and released)
            return

        # The controller performs the centered DM edge handshake. Python still
        # requires a DM release in this session, but does not demand a second
        # centered frame after the user has already moved the joystick.
        if not self.pad_authorized:
            if not self.pad_released:return
            self.pad_authorized=True;self.pad_released=False

        # This receiver intentionally accepts only the new manual protocol.
        if p.get('control_style') != 'manual90':
            self.halt()
            return

        drive = p.get('drive', 0)
        steer = p.get('steer', 0)
        if drive not in (-1, 0, 1) or steer not in (-1, 0, 1):
            self.halt()
            return
        self.update_arm_from_coco(p,now)

        cap = (85, 110, 135)[int(p.get('speed_level', 0)) % 3]
        self.primitive = None
        if steer:
            self.live = (-steer * cap, steer * cap)
        elif drive:
            self.live = (drive * cap, drive * cap)
        else:
            self.live = None
        self.stopped = self.live is None
    def run(self):
        last_output=last_state=0
        while not self.quit:
            now=time.monotonic()
            try:
                for _ in range(30):
                    try:k,v,epoch=self.urgent_commands.get_nowait()
                    except queue.Empty:break
                    self.handle(k,v)
                self.poll_links()
                if now-self.retry_at>2:
                    self.retry_at=now
                    for which in ('robot','pad'):
                        if getattr(self,which) is None:self.request_link(which)
                for _ in range(30):
                    try:k,v,epoch=self.commands.get_nowait()
                    except queue.Empty:break
                    if epoch!=self.epoch:continue
                    try:self.handle(k,v)
                    except Exception as e:
                        self.halt();self.test_result='Stopped: '+str(e);self.emit(self.test_result)
                if now-self.last_ui>0.6:self.halt_loss()
                if self.robot:
                    try:
                        for t in self.robot.receive():
                            if not all(k in t for k in ('tl','tr','ms','seq','boot_id','armed','pl','pr','heading','imu')):continue
                            if self.telemetry and t['boot_id']==self.telemetry.get('boot_id') and t['seq']==self.telemetry.get('seq'):continue
                            if not all(type(t[k]) is int for k in ('tl','tr','ms','seq','boot_id')):continue
                            if not all(isinstance(t[k],(int,float)) and math.isfinite(t[k]) for k in ('tl','tr','ms','seq','boot_id','pl','pr','heading')):continue
                            if self.last_boot is not None and t['boot_id']!=self.last_boot:
                                self.halt(False);choice=self.odom.prefer_imu;self.odom.reset();self.odom.prefer_imu=choice;self.path.clear();self.emit('Robot rebooted; pose reset, reading boot relay state')
                            if self.reset_pending and t.get('estop')==0:
                                self.reset_pending=False;self.estop_latched=False;self.emit('E-STOP desbloqueado; alimentación OFF')
                            if t.get('estop')==1 and not self.estop_latched:
                                self.estop_latched=True;self.halt(True)
                            self.last_boot=t['boot_id'];self.last_robot=time.monotonic();self.telemetry=t
                            arm_channels=t.get('arm_angles')
                            if (isinstance(arm_channels,list) and len(arm_channels)==6
                                    and all(isinstance(value,(int,float)) and math.isfinite(value) for value in arm_channels)):
                                self.arm_angles=np.array([arm_channels[0],arm_channels[1],arm_channels[2],arm_channels[4]],dtype=float)
                                self.arm_rotation=float(arm_channels[3])
                                self.arm_gripper=float(arm_channels[5])
                                self.arm_busy=bool(t.get('arm_busy',0))
                                if not self.arm_busy:
                                    self.arm_target_angles=self.arm_angles.copy()
                                    self.arm_target_gripper=self.arm_gripper
                                    self.arm_target_rotation=self.arm_rotation
                                    self.arm_model.last_ik_theta=self.arm_model.convert_angles(self.arm_angles)
                            if not t['armed'] and (self.live or self.primitive):self.halt()
                            if self.capture:
                                self.capture.observe(t)
                                self.odom.previous=None
                            if self.odom.update(t,self.g):
                                ref=self.distance_reference
                                if ref and ref['side']!='both' and (not self.odom.imu_valid or self.odom.imu_segment!=ref['segment'] or abs(self.odom.imu_yaw-ref['yaw'])>5):ref['valid']=False
                                point=(self.odom.x/10,self.odom.y/10)
                                if not self.path:self.path.append((0.0,0.0))
                                if math.hypot(point[0]-self.path[-1][0],point[1]-self.path[-1][1])>=0.01:self.path.append(point)
                                self.rows.append((now,t['ms'],t['tl'],t['tr'],self.odom.x,self.odom.y,math.degrees(self.odom.yaw),self.odom.imu_yaw,self.odom.v,t['pl'],t['pr'],math.degrees(self.odom.enc_yaw),self.odom.source,int(self.odom.imu_valid),self.odom.dl,self.odom.dr))
                    except (OSError,ConnectionError) as e:
                        self.halt_loss();self.robot.close();self.robot=None;self.telemetry={};self.emit('Robot link lost: '+str(e))
                if self.pad:
                    try:
                        for p in self.pad.receive():self.process_pad(p,time.monotonic())
                    except (OSError,ConnectionError,KeyError,ValueError) as e:
                        if self.source=='COCO':self.halt_loss()
                        self.pad.close();self.pad=None;self.last_pad=0;self.emit('COCO link lost: '+str(e))
                now=time.monotonic()
                if self.source=='COCO' and now-self.last_pad>0.5 and (self.live or self.primitive or self.pad_authorized):self.halt_loss()
                if self.robot and now-self.last_robot>0.5 and (self.live or self.primitive or self.telemetry.get('armed')):self.halt_loss()
                # Rebuild genuinely stalled sessions; do not resume old commands.
                if self.robot and now-max(self.last_robot,self.robot_connected_at)>3:
                    self.halt(False);self.robot.close();self.robot=None;self.telemetry={}
                    self.emit('Robot RX silent >3 s: reconnecting with motion stopped')
                if self.pad and self.last_pad and now-self.last_pad>3:
                    if self.source=='COCO':self.halt_loss()
                    self.pad.close();self.pad=None;self.pad_data={};self.last_pad=0
                    self.emit('COCO RX silent >3 s: reconnecting, release DM')
                l=r=0
                if self.robot and self.telemetry.get('armed') and now-self.last_robot<=0.5 and now-self.last_ui<=.6 and not self.estop_latched and self.telemetry.get('protocol')==1:
                    if self.routine and not self.primitive and not self.live and now>=self.next_step_at:
                        kind,value=self.routine.popleft();self.start_test(kind,value,self.routine_cap);self.routine_step+=1
                        self.emit(f'Routine step {self.routine_step}/{self.routine_total}: {kind} {value:g}')
                    # Advance timed pulses only when their PWM will be sent.
                    if self.primitive and now-last_output>=0.05:
                        try:
                            l,r=self.primitive.output(self.odom,now)
                            if self.primitive.done:
                                p=self.primitive
                                if p.kind=='distance_imu':actual=p.distance_cm(self.odom)
                                elif p.kind=='distance':actual=((self.odom.left+self.odom.right)/2-p.origin[4])/10
                                else:actual=self.odom.imu_yaw-p.origin[3]
                                if p.kind=='wait':actual=now-p.start
                                imu=self.odom.imu_yaw-p.origin[3]
                                self.test_result=f'{p.kind}: target {p.value:.2f}, measured {actual:.2f}, error {actual-p.value:+.2f}; IMU delta {imu:.2f} deg'
                                if p.kind=='distance_imu':
                                    self.test_result+=f'; encoder {p.side}; heading error {p.heading_target-self.odom.imu_yaw:+.2f} deg'
                                elif p.kind in ('imu_turn','imu_precise'):
                                    dl=self.odom.left-p.wheels[0];dr=self.odom.right-p.wheels[1]
                                    candidate=(dr-dl)/math.radians(imu) if abs(imu)>5 else 0
                                    self.test_result+=f'; wheel L={dl:.1f} mm R={dr:.1f} mm; candidate track={candidate:.1f} mm (requires calibrated TPR and correct signs). No values auto-applied.'
                                self.primitive=None;self.stopped=True;self.next_step_at=now+0.5
                        except RuntimeError as e:
                            self.halt();self.test_result='Stopped: '+str(e);self.emit(self.test_result);l=r=0
                    elif self.live:l,r=self.live
                if self.capture:
                    if self.robot and now-self.last_robot<=.6:
                        if not self.telemetry.get('armed'):self.capture.stop('Power OFF: partial capture kept',now)
                        l,r=self.capture.output(self.telemetry,now)
                    else:
                        self.capture.valid=False;self.capture.stop('Telemetry lost: capture cannot include coasting; repeat test',now)
                        if now-self.capture.stopped_at>=.45:self.capture.done=True
                        l=r=0
                    self.last_wheel_test=self.capture.report()
                    if self.capture.done:
                        self.test_result=f"{self.capture.side.upper()}: {self.capture.signed_ticks:+d} raw ticks. {self.capture.reason}. Enter actual turns and Apply."
                        self.emit(self.test_result);self.capture=None;l=r=0
                if self.robot and now-last_output>=0.05:
                    turning=(l*r<0 and abs(l)>0 and abs(r)>0)
                    if turning and not self.was_turning:self.turn_origin=self.odom.imu_yaw
                    if self.was_turning and not turning:self.turn_settle_until=now+.5
                    if turning or now<self.turn_settle_until:self.turn_delta=self.odom.imu_yaw-self.turn_origin
                    self.was_turning=turning
                    last_output=now
                    try:
                        out_l,out_r=self.g.motor_output(l,r)
                        self.robot.send(f'PWM:{out_l},{out_r}\n');self.sent_pwm=(out_l,out_r)
                    except OSError:self.halt(False);self.robot.close();self.robot=None
                if self.pad and now-last_state>=0.1:
                    fresh=bool(self.robot and now-self.last_robot<=0.5);last_state=now
                    try:self.pad.send(f"CTRL:{int(self.source=='COCO')}\nSTATE:0,0,0,0,0,0,{self.odom.v:.2f},{int(fresh)},{int(fresh and self.telemetry.get('armed',0))},0\nAGV:{(self.odom.left+self.odom.right)/20:.2f},{math.degrees(self.odom.yaw):.2f},{self.odom.v:.2f},{self.telemetry.get('tl',0)},{self.telemetry.get('tr',0)},{int(self.telemetry.get('imu',0))}\nYAW:{self.odom.imu_yaw:.2f},{self.turn_delta:.2f},{self.odom.imu_yaw/360:.4f},{int(fresh and self.odom.imu_valid)}\n")
                    except OSError:
                        if self.source=='COCO':self.halt_loss()
                        self.pad.close();self.pad=None
                if now-getattr(self,'last_render',0)>0.1:
                    self.last_render=now;self.signals.state.emit({'distance_calibration':self.distance_result,'distance_progress':({'side':self.distance_reference['side'],'left_cm':(self.odom.left-self.distance_reference['left'])/10,'right_cm':(self.odom.right-self.distance_reference['right'])/10} if self.distance_reference else None),'pose_revision':self.odom.revision,'imu_segment':self.odom.imu_segment,'single_left':self.odom.single_left,'single_right':self.odom.single_right,'host_monotonic':now,'sent_pwm':self.sent_pwm,'estop_latched':self.estop_latched,'age':now-self.last_robot if self.last_robot else 0,'vl':self.odom.vl,'vr':self.odom.vr,'routine_step':self.routine_step,'routine_total':self.routine_total,'telemetry':dict(self.telemetry),'fresh':bool(self.robot and now-self.last_robot<0.5),'pad_fresh':bool(self.pad and now-self.last_pad<0.5),'path':list(self.path),'x':self.odom.x,'y':self.odom.y,'yaw':math.degrees(self.odom.yaw),'imu_yaw':self.odom.imu_yaw,'speed':self.odom.v,'result':self.test_result,'pad':dict(self.pad_data),'enc_yaw':math.degrees(self.odom.enc_yaw),'imu_valid':self.odom.imu_valid,'imu_reason':self.odom.imu_reason,'yaw_source':self.odom.source,'robot_retry':bool(self.robot_host and self.robot is None),'pad_retry':bool(self.pad_host and self.pad is None),'geometry':asdict(self.g),'capture_active':self.capture is not None,'last_wheel_test':self.last_wheel_test,'turn_delta':self.turn_delta,'dl':self.odom.dl,'dr':self.odom.dr,'arm_angles':self.arm_angles.tolist(),'arm_gripper':self.arm_gripper,'arm_target_angles':self.arm_target_angles.tolist(),'arm_target_gripper':self.arm_target_gripper,'arm_busy':self.arm_busy,'arm_error':self.arm_error})
            except Exception as e:self.halt(False);self.emit('Stopped: '+str(e))
            time.sleep(0.005)
        self.halt(True)
        for _,future in self.pending_links.values():
            future.add_done_callback(lambda f: f.result().close() if not f.cancelled() and f.exception() is None else None)
        self.pool.shutdown(wait=False,cancel_futures=True)
        for link in (self.robot,self.pad):
            if link:link.close()

class ArmPreview(QtWidgets.QWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setMinimumSize(300,260);self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,QtWidgets.QSizePolicy.Policy.Expanding)
        self.points=[];self.position=np.zeros(3);self.gripper=75.0
        model=ArmKinematics()
        self.set_pose(model,model.startup_angles,self.gripper)
    def set_pose(self,model,angles,gripper):
        self.position,self.transforms=model.compute_fk(angles)
        self.points=[matrix[:3,3].copy() for matrix in self.transforms]
        self.gripper=float(gripper)
        self.update()
    def paintEvent(self,_event):
        painter=QtGui.QPainter(self);painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(),QtGui.QColor('#0a1420'))
        if not self.points:return
        projected=[((point[0]-point[1])*.7071,point[2]-(point[0]+point[1])*.3536) for point in self.points]
        xs=[point[0] for point in projected];ys=[point[1] for point in projected]
        width=max(max(xs)-min(xs),1.0);height=max(max(ys)-min(ys),1.0)
        margin=30.0;scale=min((self.width()-2*margin)/width,(self.height()-2*margin)/height)
        center_x=self.width()/2;center_y=self.height()/2
        offset_x=(min(xs)+max(xs))/2;offset_y=(min(ys)+max(ys))/2
        screen=[QtCore.QPointF(center_x+(x-offset_x)*scale,center_y-(y-offset_y)*scale) for x,y in projected]
        painter.setPen(QtGui.QPen(QtGui.QColor('#7ce1d8'),5,QtCore.Qt.PenStyle.SolidLine,QtCore.Qt.PenCapStyle.RoundCap))
        for start,end in zip(screen,screen[1:]):painter.drawLine(start,end)
        painter.setPen(QtGui.QPen(QtGui.QColor('#ffc760'),2));painter.setBrush(QtGui.QColor('#ffc760'))
        painter.drawEllipse(screen[0],6,6)
        painter.setPen(QtGui.QPen(QtGui.QColor('#f8fafc'),2));painter.setBrush(QtGui.QColor('#f8fafc'))
        painter.drawEllipse(screen[-1],5,5)
        painter.setPen(QtGui.QColor('#94a3b8'))
        painter.drawText(10,20,f"Simulated XYZ: {self.position[0]:.1f}, {self.position[1]:.1f}, {self.position[2]:.1f} mm")
        _event.accept()

class Window(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__();self.setWindowTitle('SAR Robot Controller 1.6 — AGV / Brazo V7 / Cámaras');self.resize(1320,860)
        self.setStyleSheet('''QWidget{background:#101b27;color:#eaf2fa;font-size:13px;}
        QPushButton{background:#29465c;border:1px solid #50738b;border-radius:5px;padding:8px;}
        QPushButton:hover{background:#3c617b;} QPushButton:pressed{background:#078d8b;}
        QLineEdit,QSpinBox,QDoubleSpinBox,QComboBox,QListWidget,QPlainTextEdit{background:#1a2c3d;color:#fff;border:1px solid #456179;padding:4px;}
        QGroupBox{border:1px solid #3b5369;border-radius:6px;margin-top:14px;padding:10px;}
        QGroupBox::title{color:#80ded8;subcontrol-origin:margin;left:8px;}
        QTabBar::tab{padding:9px;background:#24394b;} QTabBar::tab:selected{background:#3d6077;}
        QScrollArea{border:0;} QLabel{background:transparent;}''')
        self.engine=Engine();self.engine.signals.state.connect(self.render);self.engine.signals.message.connect(self.message)
        self.last_report=None;self.last_geometry=None;self.holding=False;self.routine_steps=[]
        self.arm_model=ArmKinematics();self.arm_preview_angles=self.arm_model.startup_angles.copy();self.arm_preview_gripper=75.0;self.arm_inputs_syncing=False;self.arm_preview_valid=True
        central=QtWidgets.QWidget();self.setCentralWidget(central);outer=QtWidgets.QVBoxLayout(central)
        top=QtWidgets.QHBoxLayout();outer.addLayout(top)
        brand=QtWidgets.QLabel('SAR 1.6');brand.setStyleSheet('font-size:21px;color:#7ce1d8;font-weight:bold');top.addWidget(brand)
        self.ip=QtWidgets.QLineEdit(self.engine.runtime['robot_ip']);self.padip=QtWidgets.QLineEdit(self.engine.runtime['pad_ip'])
        for label,field in [('AGV',self.ip),('COCO',self.padip)]:top.addWidget(QtWidgets.QLabel(label));top.addWidget(field)
        self.button(top,'Connect AGV',lambda:self.engine.post('connect_robot',self.ip.text().strip()))
        self.button(top,'Connect COCO',lambda:self.engine.post('connect_pad',self.padip.text().strip()))
        self.source=QtWidgets.QComboBox();self.source.addItems(['PC','COCO']);top.addWidget(self.source);self.source.currentTextChanged.connect(lambda v:self.global_stop('source',v))
        self.button(top,'POWER ON',lambda:self.engine.post('power',True))
        b=self.button(top,'POWER OFF',lambda:self.global_stop('stop',True));b.setStyleSheet('background:#713542')
        b=self.button(top,'STOP',self.stop);b.setStyleSheet('background:#b95823;font-weight:bold;padding:12px')
        safety=QtWidgets.QHBoxLayout();outer.addLayout(safety)
        emergency=self.button(safety,'E-STOP',lambda:self.global_stop('estop'))
        emergency.setStyleSheet('background:#c6283c;color:white;font-size:18px;font-weight:bold;padding:12px;')
        self.button(safety,'Desbloquear E-STOP',lambda:self.global_stop('estop_reset'))
        self.button(safety,'Dar control a COCO',lambda:self.source.setCurrentText('COCO'))
        self.arm_send_button=self.button(safety,'Enviar pose del brazo',self.send_arm_pose)
        self.arm_send_button.setToolTip('Envía al AGV la pose válida del panel Brazo · V7.')
        self.button(safety,'Recuperar control PC',lambda:self.source.setCurrentText('PC'))
        self.cut_loss=QtWidgets.QCheckBox('Apagar relays al perder comunicación')
        self.cut_loss.setChecked(bool(self.engine.runtime['cut_relays_on_loss']))
        self.cut_loss.toggled.connect(lambda v:self.engine.post('policy',v));safety.addWidget(self.cut_loss)
        self.status=QtWidgets.QLabel('Connect AGV to receive telemetry');self.status.setWordWrap(True);outer.addWidget(self.status)
        self.motor_status=QtWidgets.QLabel('Motor PWM: waiting for telemetry');self.motor_status.setWordWrap(True);outer.addWidget(self.motor_status)
        self.workspaces=QtWidgets.QTabWidget();outer.addWidget(self.workspaces,1)
        agv_page=QtWidgets.QWidget();row=QtWidgets.QHBoxLayout(agv_page);self.workspaces.addTab(agv_page,'AGV')
        self.tabs=QtWidgets.QTabWidget();self.tabs.setMinimumWidth(440);self.tabs.setMaximumWidth(540);row.addWidget(self.tabs)
        def tab(name):
            page=QtWidgets.QWidget();box=QtWidgets.QVBoxLayout(page);sc=QtWidgets.QScrollArea();sc.setWidgetResizable(True);sc.setWidget(page);self.tabs.addTab(sc,name);return box
        drive=tab('Drive');cal=tab('Calibrate');routine=tab('Routine');arm=tab('Arm')
        f=QtWidgets.QFormLayout();drive.addLayout(f)
        self.cap=QtWidgets.QSpinBox();self.cap.setRange(1,180);self.cap.setValue(110);f.addRow('Drive PWM',self.cap)
        self.minimum=QtWidgets.QSpinBox();self.minimum.setRange(1,180);self.minimum.setValue(self.engine.min_pwm);f.addRow('Minimum moving PWM',self.minimum)
        self.minimum.valueChanged.connect(lambda v:self.engine.post('min_pwm',v))
        grid=QtWidgets.QGridLayout();drive.addLayout(grid)
        for label,pos,d in [('FORWARD',(0,1),(1,1)),('LEFT',(1,0),(-1,1)),('RIGHT',(1,2),(1,-1)),('REVERSE',(2,1),(-1,-1))]:
            b=QtWidgets.QPushButton(label+'\nHold');grid.addWidget(b,*pos)
            b.pressed.connect(lambda d=d:self.drive(d));b.released.connect(self.stop)
        f=QtWidgets.QFormLayout();drive.addLayout(f)
        self.dist=self.number(-200,200,20,' cm');self.turn=self.number(-360,360,90,' deg')
        f.addRow('Move distance',self.dist);f.addRow('Turn (+ left / - right)',self.turn)
        self.move_encoder=QtWidgets.QComboBox();self.move_encoder.addItems(['Left','Right'])
        f.addRow('Encoder for single-encoder MOVE',self.move_encoder)
        r=QtWidgets.QHBoxLayout();drive.addLayout(r);self.button(r,'MOVE',lambda:self.test('distance',self.dist.value()));self.button(r,'ROTATE · IMU',lambda:self.test('turn',self.turn.value()))
        self.button(drive,'MOVE · 1 ENCODER + IMU',lambda:self.test('distance_imu',self.dist.value(),self.move_encoder.currentText().lower()))
        self.button(drive,'ROTATE · PRECISE IMU',lambda:self.test('imu_precise',self.turn.value()))
        hint=QtWidgets.QLabel('Precise IMU: short pulses near target; corrects overshoot.\nFinishes within 2 deg after settling. Stops if IMU is lost.');hint.setWordWrap(True);drive.addWidget(hint)
        self.button(drive,'New map / zero relative heading',lambda:self.engine.post('reset'))
        self.button(drive,'Reverse IMU direction',lambda:self.engine.post('imu_flip'))
        self.imu_info=QtWidgets.QLabel('BNO055: waiting');self.imu_info.setWordWrap(True);drive.addWidget(self.imu_info)
        drive.addWidget(QtWidgets.QLabel('Map axes remain fixed in the room.\n0° points along +X; +90° along +Y.\nWith one wheel on the ground stationary, an arc is normal.\nUse Calibrate with both wheels lifted: map is paused.'))
        drive.addStretch()
        self.cal_status=QtWidgets.QLabel('1. Lift both wheels. Choose a wheel and start.');self.cal_status.setWordWrap(True);cal.addWidget(self.cal_status)
        f=QtWidgets.QFormLayout();cal.addLayout(f)
        self.cal_side=QtWidgets.QComboBox();self.cal_side.addItems(['Left','Right']);f.addRow('Wheel',self.cal_side)
        self.cal_turns=self.number(.05,10,1,' rev');f.addRow('Requested turns',self.cal_turns)
        self.cal_pwm=QtWidgets.QSpinBox();self.cal_pwm.setRange(1,180);self.cal_pwm.setValue(110);f.addRow('Constant test PWM',self.cal_pwm)
        r=QtWidgets.QHBoxLayout();cal.addLayout(r)
        self.button(r,'START WHEEL TEST',self.start_wheel);self.button(r,'STOP / KEEP TICKS',self.stop)
        self.capture_label=QtWidgets.QLabel('Recorded ticks: —');self.capture_label.setStyleSheet('font-size:17px;color:#7ae4d6;padding:8px');self.capture_label.setWordWrap(True);self.capture_label.setMinimumHeight(58);cal.addWidget(self.capture_label)
        f=QtWidgets.QFormLayout();cal.addLayout(f)
        self.observed=self.number(.001,30,.75,' rev');self.observed.setDecimals(3);f.addRow('Actually turned (75% = 0.750)',self.observed)
        self.forward=QtWidgets.QComboBox();self.forward.addItems(['Toward robot FRONT','Toward robot REAR']);f.addRow('Physical wheel direction',self.forward)
        self.preview=QtWidgets.QLabel('The recorded result will appear here, even after STOP.');self.preview.setWordWrap(True);self.preview.setMinimumHeight(58);cal.addWidget(self.preview)
        self.observed.valueChanged.connect(self.preview_correction);self.forward.currentIndexChanged.connect(self.preview_correction)
        self.button(cal,'APPLY + SAVE RESULT',self.apply_capture)
        self.applied=QtWidgets.QLabel('Current wheel calibration: —');self.applied.setWordWrap(True);cal.addWidget(self.applied)
        cal.addWidget(QtWidgets.QLabel('This updates ticks/revolution AND direction signs.\nNo copying values between panels. Retest after applying.\nCount full turns plus the remaining fraction.'))
        advanced=QtWidgets.QGroupBox('Advanced settings / wiring checks');advanced.setCheckable(True);advanced.setChecked(False);cal.addWidget(advanced)
        av=QtWidgets.QVBoxLayout(advanced);inner=QtWidgets.QWidget();av.addWidget(inner);af=QtWidgets.QFormLayout(inner);inner.setVisible(False);advanced.toggled.connect(inner.setVisible);self.fields={}
        defaults=dict(GEOMETRY_DEFAULTS);path=Path(__file__).with_name('settings.json')
        if path.exists():
            try:defaults.update({k:v for k,v in json.loads(path.read_text()).items() if k in defaults})
            except (ValueError,OSError):pass
        labels={'scale_l':'Left distance scale','scale_r':'Right distance scale','diameter':'Wheel diameter mm','track':'Wheel-center separation mm','tpr_l':'Left ticks/rev','tpr_r':'Right ticks/rev','sign_l':'Left encoder sign','sign_r':'Right encoder sign','motor_l':'Left motor sign','motor_r':'Right motor sign','imu_sign':'IMU sign (+left)','swap_lr':'Swap motor channels 0/1','swap_encoders':'Swap encoder channels 0/1'}
        for k,v in defaults.items():
            field=QtWidgets.QLineEdit(str(v));self.fields[k]=field;af.addRow(labels[k],field)
        b=QtWidgets.QPushButton('Apply & save advanced settings');af.addRow(b);b.clicked.connect(self.apply_geometry)
        distance_box=QtWidgets.QGroupBox('Ground distance calibration · 50 cm');df=QtWidgets.QVBoxLayout(distance_box)
        self.distance_encoder=QtWidgets.QComboBox();self.distance_encoder.addItems(['Left','Right']);df.addWidget(self.distance_encoder)
        if self.engine.distance_result and self.engine.distance_result.get('side') in ('left','right'):self.distance_encoder.setCurrentText(self.engine.distance_result['side'].title())
        self.button(df,'1. START 50 cm · encoder + IMU',lambda:self.engine.post('distance_test',(self.distance_encoder.currentText().lower(),self.cap.value())))
        self.button(df,'STOP / keep distance',self.stop)
        help_distance=QtWidgets.QLabel('Choose the reliable encoder. Mark the floor, run 50 cm indicated, then measure the actual travel. Keep a straight heading. Only the selected encoder scale is updated.');help_distance.setWordWrap(True);df.addWidget(help_distance)
        self.distance_feedback=QtWidgets.QLabel('No distance calibration yet');self.distance_feedback.setWordWrap(True);df.addWidget(self.distance_feedback)
        self.real_distance=self.number(1,500,50,' cm actual');df.addWidget(self.real_distance)
        self.button(df,'2. APPLY + SAVE distance scale',lambda:self.engine.post('distance_apply',self.real_distance.value()))
        cal.addWidget(distance_box);cal.addStretch()
        routine.addWidget(QtWidgets.QLabel('Add distance / angle steps, then RUN.\nStops between steps. STOP cancels the sequence.'))
        self.routine_kind=QtWidgets.QComboBox();self.routine_kind.addItems(['Move (cm)','Rotate (degrees)','Rotate precise IMU (degrees)','Wait (seconds)'])
        self.routine_value=self.number(-360,360,20,'');f=QtWidgets.QFormLayout();f.addRow('Action',self.routine_kind);f.addRow('Value',self.routine_value);routine.addLayout(f)
        self.button(routine,'Add step',self.add_step);self.routine_list=QtWidgets.QListWidget();routine.addWidget(self.routine_list)
        r=QtWidgets.QHBoxLayout();routine.addLayout(r);self.button(r,'Remove',self.remove_step);self.button(r,'Clear',self.clear_steps)
        r=QtWidgets.QHBoxLayout();routine.addLayout(r);self.button(r,'Load',self.load_routine);self.button(r,'Save',self.save_routine)
        self.button(routine,'RUN ROUTINE',lambda:self.engine.post('routine',(list(self.routine_steps),self.cap.value())))
        self.build_arm_tab(arm)
        right=QtWidgets.QVBoxLayout();row.addLayout(right,1)
        self.notice=QtWidgets.QLabel('Ready for connection. Calibration results and errors appear here.');self.notice.setWordWrap(True);self.notice.setStyleSheet('background:#24384b;color:#ffd88c;padding:10px');right.addWidget(self.notice)
        self.plot=pg.PlotWidget(title='SAR · odometría estimada / chasis 23 × 23 cm');self.plot.setBackground('#0a1420');self.plot.setLabel('bottom','World X','cm');self.plot.setLabel('left','World Y','cm')
        for axis in ('bottom','left'):self.plot.getAxis(axis).enableAutoSIPrefix(False)
        self.plot.setAspectLocked(True);self.plot.setLimits(minXRange=60,minYRange=60);self.plot.showGrid(x=True,y=True,alpha=.25);right.addWidget(self.plot,1)
        self.curve=self.plot.plot(pen=pg.mkPen('#43d9c5',width=2));self.body=self.plot.plot(pen=pg.mkPen('#cad9e5',width=2));self.arrow=self.plot.plot(pen=pg.mkPen('#ffc760',width=3))
        self.wheels=[self.plot.plot(pen=pg.mkPen('#4ad8c1',width=5)) for _ in range(2)]
        self.plot.setRange(xRange=(-40,40),yRange=(-40,40))
        r=QtWidgets.QHBoxLayout();right.addLayout(r);self.button(r,'Fit map',lambda:self.plot.enableAutoRange());self.button(r,'Export CSV',self.export)
        self.log=QtWidgets.QPlainTextEdit();self.log.setReadOnly(True);self.log.setMaximumBlockCount(100);self.log.setMaximumHeight(110);right.addWidget(self.log)
        # V7 keeps its original planning / vision methods; all physical commands go through Engine.
        from arm_workspace import ArmWorkspace
        from cameras import CamerasPage
        self.tabs.removeTab(3)
        self.arm_workspace=ArmWorkspace(self.engine)
        self.workspaces.addTab(self.arm_workspace.takeCentralWidget(),'Brazo · V7')
        self.cameras=CamerasPage()
        camera_scroll=QtWidgets.QScrollArea();camera_scroll.setWidgetResizable(True);camera_scroll.setWidget(self.cameras)
        self.workspaces.addTab(camera_scroll,'Cámaras IP · ArUco')
        self.lidar_workspace=LidarWorkspace()
        self.workspaces.addTab(self.lidar_workspace,'LiDAR / SLAM')
        right.removeWidget(self.notice);outer.addWidget(self.notice)
        right.removeWidget(self.log);outer.addWidget(self.log);self.log.setMaximumHeight(65)
        self.workspaces.currentChanged.connect(self.workspace_changed)
        self.button(safety,'Reset odometría',lambda:self.global_stop('reset'))
        self.arm_workspace.timer.stop()
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(lambda:self.engine.post('heartbeat'));self.timer.start(100)
        self.engine.start();self.apply_geometry()
    @staticmethod
    def number(lo,hi,value,suffix):
        n=QtWidgets.QDoubleSpinBox();n.setRange(lo,hi);n.setValue(value);n.setSuffix(suffix);return n
    @staticmethod
    def button(layout,text,fn):
        b=QtWidgets.QPushButton(text);layout.addWidget(b);b.clicked.connect(lambda checked=False:fn());return b
    def message(self,text):self.notice.setText(text);self.log.appendPlainText(text)
    def global_stop(self,kind='stop',value=None):
        self.holding=False
        if hasattr(self,'arm_workspace'):self.arm_workspace.cancel_motion()
        self.engine.post('stop',False)
        if kind!='stop' or value:self.engine.post(kind,value)
    def stop(self):self.global_stop()
    def workspace_changed(self,index):
        self.global_stop()
        if index==1:self.arm_workspace.timer.start(50)
        else:self.arm_workspace.timer.stop()

    def build_arm_tab(self,layout):
        layout.addWidget(QtWidgets.QLabel('FK / IK del brazo · Vista simulada; solo se envía al AGV con el botón explícito.'))
        self.arm_mode=QtWidgets.QComboBox()
        self.arm_mode.addItems(['FK · Ángulos de articulación','IK · Posición XYZ'])
        self.arm_mode.currentIndexChanged.connect(self.toggle_arm_mode)
        layout.addWidget(self.arm_mode)
        self.arm_input_stack=QtWidgets.QStackedWidget()
        fk_page=QtWidgets.QWidget();fk_layout=QtWidgets.QVBoxLayout(fk_page)
        self.arm_joint_sliders=[];self.arm_joint_labels=[]
        for index,name in enumerate(('Base CH0','Hombro CH1','Codo CH2','Muñeca elevación CH4')):
            label=QtWidgets.QLabel()
            slider=QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal);slider.setRange(0,1800)
            slider.setValue(int(round(self.arm_preview_angles[index]*10)))
            slider.valueChanged.connect(self.update_arm_preview)
            self.arm_joint_labels.append(label);self.arm_joint_sliders.append(slider)
            fk_layout.addWidget(label);fk_layout.addWidget(slider)
        self.arm_input_stack.addWidget(fk_page)
        ik_page=QtWidgets.QWidget();ik_form=QtWidgets.QFormLayout(ik_page)
        reach=self.arm_model.max_reach
        initial_xyz=self.arm_model.compute_fk(self.arm_preview_angles)[0]
        self.arm_xyz_inputs=[
            self.number(-reach,reach,float(initial_xyz[0]),' mm'),
            self.number(0,reach,float(max(0,initial_xyz[1])),' mm'),
            self.number(0,reach,float(max(0,initial_xyz[2])),' mm'),
        ]
        for field,name in zip(self.arm_xyz_inputs,('X','Y','Z')):
            field.valueChanged.connect(self.update_arm_preview)
            ik_form.addRow(f'{name} objetivo',field)
        self.arm_input_stack.addWidget(ik_page)
        layout.addWidget(self.arm_input_stack)
        grip_group=QtWidgets.QGroupBox('Pinza · CH5')
        grip_layout=QtWidgets.QVBoxLayout(grip_group)
        self.arm_gripper_label=QtWidgets.QLabel()
        self.arm_gripper_slider=QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.arm_gripper_slider.setRange(10,75);self.arm_gripper_slider.setValue(75)
        self.arm_gripper_slider.valueChanged.connect(self.update_arm_preview)
        grip_layout.addWidget(self.arm_gripper_label);grip_layout.addWidget(self.arm_gripper_slider)
        layout.addWidget(grip_group)
        self.arm_preview=ArmPreview()
        layout.addWidget(self.arm_preview,1)
        self.arm_owner_status=QtWidgets.QLabel('Control del brazo: simulación local')
        self.arm_owner_status.setWordWrap(True);layout.addWidget(self.arm_owner_status)
        self.arm_status=QtWidgets.QLabel('Pose inicial simulada. Sin sensores de posición articular.')
        self.arm_status.setWordWrap(True);layout.addWidget(self.arm_status)
        duration_form=QtWidgets.QFormLayout()
        self.arm_duration=QtWidgets.QSpinBox();self.arm_duration.setRange(100,10000);self.arm_duration.setSingleStep(100);self.arm_duration.setValue(2000);self.arm_duration.setSuffix(' ms')
        duration_form.addRow('Duración de movimiento',self.arm_duration);layout.addLayout(duration_form)
        layout.addWidget(QtWidgets.QLabel('El preview PC no modela CH3 (inicia en 90°); COCO modo Joint controla ese servo. COCO usa deadman; STOP/E-STOP cancela la interpolación.'))
        layout.addStretch()
        self.update_arm_preview()
    def toggle_arm_mode(self,*_):
        self.arm_inputs_syncing=True
        mode=self.arm_mode.currentIndex()
        self.arm_input_stack.setCurrentIndex(mode)
        if mode==0:
            for slider,value in zip(self.arm_joint_sliders,self.arm_preview_angles):
                slider.setValue(int(round(float(value)*10)))
        else:
            xyz=self.arm_model.compute_fk(self.arm_preview_angles)[0]
            for field,value in zip(self.arm_xyz_inputs,xyz):
                field.setValue(float(value))
        self.arm_inputs_syncing=False
        self.update_arm_preview()
    def update_arm_preview(self,*_):
        if self.arm_inputs_syncing:return
        mode=self.arm_mode.currentIndex()
        if mode==0:
            angles=np.array([slider.value()/10.0 for slider in self.arm_joint_sliders],dtype=float)
            xyz,_=self.arm_model.compute_fk(angles)
            self.arm_preview_angles=angles
            self.arm_inputs_syncing=True
            for field,value in zip(self.arm_xyz_inputs,xyz):
                field.setValue(float(value))
            self.arm_inputs_syncing=False
            self.arm_preview_valid=True;self.arm_status.setText('FK válida · posición simulada, no medida')
        else:
            target=np.array([field.value() for field in self.arm_xyz_inputs],dtype=float)
            target=self.arm_model.enforce_safety_zone(target)
            self.arm_inputs_syncing=True
            for field,value in zip(self.arm_xyz_inputs,target):
                field.setValue(float(value))
            self.arm_inputs_syncing=False
            success,angles,detail=self.arm_model.compute_ik(target)
            self.arm_preview_valid=success
            if success:
                self.arm_preview_angles=angles
                self.arm_status.setText(f'IK válida · error {detail:.2f} mm · pose simulada')
                self.arm_inputs_syncing=True
                for slider,value in zip(self.arm_joint_sliders,angles):
                    slider.setValue(int(round(float(value)*10)))
                self.arm_inputs_syncing=False
            else:
                self.arm_status.setText(f'IK no disponible: {detail}')
        self.arm_preview_gripper=float(self.arm_gripper_slider.value())
        self.arm_gripper_label.setText(f'Ángulo de referencia: {self.arm_preview_gripper:.0f}°')
        self.arm_preview.set_pose(self.arm_model,self.arm_preview_angles,self.arm_preview_gripper)
        self.arm_send_button.setEnabled(self.arm_preview_valid and self.source.currentText()=='PC')
    def send_arm_pose(self):
        if self.source.currentText()!='PC':
            self.message('Selecciona PC como propietario para enviar la pose desde la app.')
            return
        self.engine.post('arm_pose',(self.arm_preview_angles.copy(),self.arm_preview_gripper,self.arm_duration.value()))
        self.message('Solicitud de envío en curso; la vista es una estimación sin realimentación de servos.')
    def drive(self,d):
        if self.source.currentText()!='PC':self.message('Select PC to use screen controls');return
        self.holding=True;p=self.cap.value();self.engine.post('manual',(d[0]*p,d[1]*p))
    def test(self,k,v,encoder_side=None):
        if self.source.currentText()!='PC':self.message('Select PC to use screen controls');return
        command=(k,v,self.cap.value())
        if encoder_side is not None:command+=(encoder_side,)
        self.engine.post('primitive',command)
    def start_wheel(self):
        self.engine.post('wheel_test',(self.cal_side.currentText().lower(),self.cal_turns.value(),self.cal_pwm.value()))
    def preview_correction(self,*args):
        try:
            tpr,sign,motor=wheel_correction(self.last_report,self.observed.value(),1 if self.forward.currentIndex()==0 else -1)
            self.preview.setText(f"{self.last_report['side'].upper()} result → {tpr:.3f} ticks/rev\nEncoder sign {sign:+d}; motor sign {motor:+d}")
        except ValueError as e:self.preview.setText(str(e))
    def apply_capture(self):
        if not self.last_report:self.message('Run a wheel test first');return
        self.engine.post('wheel_apply',(self.observed.value(),1 if self.forward.currentIndex()==0 else -1))
    def apply_geometry(self):
        try:
            ints={'sign_l','sign_r','motor_l','motor_r','imu_sign','swap_lr','swap_encoders'}
            g=Geometry(**{k:(int(f.text()) if k in ints else float(f.text())) for k,f in self.fields.items()});g.validate();self.engine.post('geometry',g)
        except ValueError as e:self.message(str(e))
    def refresh_steps(self):
        self.routine_list.clear()
        for n,(kind,v) in enumerate(self.routine_steps,1):self.routine_list.addItem(f'{n}. {kind} {v:+g} {"cm" if kind=="distance" else "s" if kind=="wait" else "deg"}')
    def add_step(self):
        try:self.routine_steps=validate_routine(self.routine_steps+[(('distance','turn','imu_precise','wait')[self.routine_kind.currentIndex()],self.routine_value.value())]);self.refresh_steps()
        except ValueError as e:self.message(str(e))
    def remove_step(self):
        i=self.routine_list.currentRow()
        if i>=0:self.routine_steps.pop(i);self.refresh_steps()
    def clear_steps(self):self.routine_steps=[];self.refresh_steps()
    def load_routine(self):
        p,_=QtWidgets.QFileDialog.getOpenFileName(self,'Load routine','','JSON (*.json)')
        if p:
            try:self.routine_steps=validate_routine(json.loads(Path(p).read_text())['steps']);self.refresh_steps()
            except (ValueError,OSError,KeyError,TypeError) as e:self.message(str(e))
    def save_routine(self):
        p,_=QtWidgets.QFileDialog.getSaveFileName(self,'Save routine','routine.json','JSON (*.json)')
        if p:
            try:Path(p).write_text(json.dumps({'version':1,'steps':validate_routine(self.routine_steps)},indent=2))
            except (OSError,ValueError) as e:self.message(str(e))
    def export(self):
        p,_=QtWidgets.QFileDialog.getSaveFileName(self,'Export telemetry','telemetry.csv','CSV (*.csv)')
        if p:self.engine.post('export',p)
    def render(self,s):
        self.lidar_workspace.receive_state(s)
        progress=s.get('distance_progress');calibration=s.get('distance_calibration')
        if progress:
            side=progress['side'];value=progress.get(side+'_cm',0)
            self.distance_feedback.setText(f"{side.title()} encoder: {value:.2f} / 50 cm indicated. Enter actual travel after STOP.")
        elif calibration:
            self.distance_feedback.setText(f"Saved {calibration['side']}: indicated {calibration['indicated_cm']:.2f} cm / actual {calibration['measured_cm']:.2f} cm · factor {calibration['ratio']:.4f}")
        if hasattr(self,'arm_workspace'):self.arm_workspace.receive_state(s)
        t=s['telemetry'];g=s['geometry'];fresh=s['fresh'];heading=t.get('heading',0)
        sent=s.get('sent_pwm',(0,0))
        actual=f"{t.get('pl',0)}, {t.get('pr',0)}" if fresh else 'unknown (offline)'
        component=lambda key:'?' if t.get(key) is None else 'OK' if t.get(key) else 'ERR'
        bno_state='?' if 'imu_read_ok' not in t else 'OK' if t.get('imu_read_ok') and t.get('imu_age_ms',99999)<=250 else 'ERR'
        self.motor_status.setText(f"Control: {self.source.currentText()} | PWM PC L,R: {sent[0]}, {sent[1]} | PWM AGV L,R: {actual} | PWM ready: {t.get('pwm_ok','?')} | E-stop: {t.get('estop','?')}")
        self.status.setText(f"AGV: {'ONLINE' if fresh else 'RECONNECTING' if s['robot_retry'] else 'OFFLINE'}   |   Relays: {'ON' if fresh and t.get('armed') else 'OFF' if fresh else '?'}   |   COCO: {'ONLINE' if s['pad_fresh'] else 'OFFLINE'}   |   X {s['x']/10:.1f} cm   Y {s['y']/10:.1f} cm   Yaw {s['yaw']:+.1f}° ({s['yaw_source']})")
        self.motor_status.setText(self.motor_status.text()+f" | v L/R: {s.get('vl',0)/10:.1f}/{s.get('vr',0)/10:.1f} cm/s\nE-STOP: {'ACTIVO' if s.get('estop_latched') else 'libre'} | Política confirmada: {t.get('cut_relays_on_loss','?')} | Datos: {s.get('age',0):.2f}s | Paso {s.get('routine_step',0)}/{s.get('routine_total',0)}\nI2C BNO:{bno_state} OLED:{component('oled_ready')} PCA9685:{component('pca_ready')} | TCP RX:{t.get('tcp_rx_frames','?')} TX:{t.get('tcp_tx_frames','?')}")
        imu=f"BNO055: {'DATA OK' if fresh and s['imu_valid'] else s['imu_reason']}\nHeading {heading:.1f}° | relative {s['imu_yaw']:+.1f}° | roll {t.get('roll',0):.1f}° pitch {t.get('pitch',0):.1f}°\nCalibration (info): {t.get('cs','?')}/{t.get('cg','?')}/{t.get('ca','?')}/{t.get('cm','?')} | address {hex(t.get('imu_addr',0))}\nTicks raw L/R: {t.get('tl','?')} / {t.get('tr','?')} | Vueltas IMU: {s['imu_yaw']/360:+.3f}"
        self.imu_info.setText(imu)
        pca_status='OK' if fresh and t.get('pca_ready')==1 else 'ERR' if fresh and t.get('pca_ready')==0 else '?'
        arm_state='moviendo' if s.get('arm_busy') else 'en espera'
        arm_note=s.get('arm_error') or f'PCA9685 {pca_status} · pose de servos estimada · {arm_state}'
        self.arm_owner_status.setText(f"Propietario: {self.source.currentText()} · AGV {'ONLINE' if fresh else 'OFFLINE'} · {arm_note}")
        if self.source.currentText()=='COCO' and fresh:
            angles=s.get('arm_angles')
            if isinstance(angles,list) and len(angles)==4:
                self.arm_preview_angles=np.array(angles,dtype=float)
                self.arm_preview_gripper=float(s.get('arm_gripper',self.arm_preview_gripper))
                self.arm_inputs_syncing=True
                for slider,value in zip(self.arm_joint_sliders,self.arm_preview_angles):
                    slider.setValue(int(round(float(value)*10)))
                xyz=self.arm_model.compute_fk(self.arm_preview_angles)[0]
                for field,value in zip(self.arm_xyz_inputs,xyz):
                    field.setValue(float(value))
                self.arm_gripper_slider.setValue(int(round(self.arm_preview_gripper)))
                self.arm_inputs_syncing=False
                self.update_arm_preview()
        self.arm_send_button.setEnabled(self.arm_preview_valid and self.source.currentText()=='PC')
        if self.last_geometry!=g:
            self.last_geometry=dict(g)
            for k,v in g.items():self.fields[k].setText(str(v))
        self.applied.setText(f"SAVED: Left {g['tpr_l']:.3f} ticks/rev | Right {g['tpr_r']:.3f}\nEncoder signs L {g['sign_l']:+d}, R {g['sign_r']:+d} | Motor signs L {g['motor_l']:+d}, R {g['motor_r']:+d}")
        report=s['last_wheel_test']
        if report!=self.last_report:
            self.last_report=dict(report) if report else None
            if report:
                self.capture_label.setText(f"{report['side'].upper()} ticks: {report['ticks']:+d}\nOther wheel: {report['other_ticks']:+d} | {report['reason']}")
            else:self.capture_label.setText('Recorded ticks: —')
            self.preview_correction()
        self.cal_status.setText('BENCH TEST: map paused; both wheels must be lifted.' if s['capture_active'] else 'Choose wheel → test → enter actual turns → Apply & Save.')
        if s['result']!=getattr(self,'previous_result',None):self.previous_result=s['result'];self.notice.setText(s['result'])
        path=s['path'];self.curve.setData([p[0] for p in path],[p[1] for p in path])
        cx=s['x']/10;cy=s['y']/10;a=math.radians(s['yaw']);c=math.cos(a);sn=math.sin(a)
        corners=[(-11.5,-11.5),(11.5,-11.5),(11.5,11.5),(-11.5,11.5),(-11.5,-11.5)]
        ring=[(cx+x*c-y*sn,cy+x*sn+y*c) for x,y in corners]
        for wheel,side in zip(self.wheels,(-1,1)):
            points=[(-4,side*12),(4,side*12)]
            wheel.setData([cx+x*c-y*sn for x,y in points],[cy+x*sn+y*c for x,y in points])
        self.body.setData([v[0] for v in ring],[v[1] for v in ring])
        local=[(0,0),(8,0),(4.5,2.5),(8,0),(4.5,-2.5)]
        self.arrow.setData([cx+x*c-y*sn for x,y in local],[cy+x*sn+y*c for x,y in local])
    def changeEvent(self,event):
        if event.type()==QtCore.QEvent.Type.ActivationChange and not self.isActiveWindow() and self.holding:self.stop()
        super().changeEvent(event)
    def closeEvent(self,event):
        self.global_stop()
        if hasattr(self,'arm_workspace'):
            self.arm_workspace.timer.stop()
            self.arm_workspace._save_vision_config(silent=True)
            worker=self.arm_workspace.vision_thread
            if worker is not None and worker.isRunning():
                self.arm_workspace._stop_camera_stream()
                QtCore.QTimer.singleShot(200,self.close)
                event.ignore();return
        self.cameras.shutdown()
        self.lidar_workspace.shutdown()
        self.engine.quit=True;self.engine.join(1.5);event.accept()

if __name__=='__main__':
    app=QtWidgets.QApplication([]);app.setStyle('Fusion');w=Window();w.show();app.exec()
