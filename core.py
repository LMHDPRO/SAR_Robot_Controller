"""Units: mm, radians, encoder A-channel CHANGE ticks; yaw positive LEFT."""
import math
from dataclasses import dataclass

@dataclass
class Geometry:
    diameter: float = 120.0
    track: float = 200.0
    tpr_l: float = 341.2
    tpr_r: float = 341.2
    scale_l: float = 1.0
    scale_r: float = 1.0
    sign_l: int = 1
    sign_r: int = 1
    motor_l: int = 1
    motor_r: int = 1
    imu_sign: int = -1
    swap_lr: int = 0
    swap_encoders: int = 0
    def validate(self):
        if not (10 <= self.diameter <= 500 and 30 <= self.track <= 1000 and
                1 <= self.tpr_l <= 100000 and 1 <= self.tpr_r <= 100000):
            raise ValueError('Geometry outside allowed range')
        if not (0.25 <= self.scale_l <= 4 and 0.25 <= self.scale_r <= 4):
            raise ValueError('Distance scale must be between 0.25 and 4')
        if any(v not in (-1, 1) for v in (self.sign_l,self.sign_r,self.motor_l,self.motor_r,self.imu_sign)):
            raise ValueError('Signs must be -1 or +1')
        if self.swap_lr not in (0,1) or self.swap_encoders not in (0,1):raise ValueError('Swap motor/encoder channels must be 0 or 1')
    def wheel_ticks(self, tl, tr):
        return (tr,tl) if self.swap_encoders else (tl,tr)
    def motor_output(self,left,right):
        l=int(left)*self.motor_l;r=int(right)*self.motor_r
        return (r,l) if self.swap_lr else (l,r)
    def distances(self, tl, tr):
        tl,tr=self.wheel_ticks(tl,tr)
        return (tl*self.sign_l*math.pi*self.diameter/self.tpr_l*self.scale_l,
                tr*self.sign_r*math.pi*self.diameter/self.tpr_r*self.scale_r)

class Odometry:
    def __init__(self): self.reset()
    def reset(self):
        self.x=self.y=self.yaw=self.enc_yaw=self.left=self.right=self.v=self.imu_yaw=0.0
        self.previous=None;self.last_heading=None;self.imu_valid=False;self.imu_segment=0
        self.source='ENC';self.imu_reason='Waiting for telemetry';self.prefer_imu=True
        self.imu_delta=0.0;self.imu_rate=0.0;self.dl=self.dr=0.0;self.vl=self.vr=0.0
    @staticmethod
    def tick_delta(a,b):return ((a-b+2**31)%2**32)-2**31
    @staticmethod
    def quality(t):
        if not t.get('imu'):return False,'IMU absent'
        if not math.isfinite(float(t.get('heading',float('nan')))):return False,'Invalid heading'
        if not t.get('imu_read_ok',False) or t.get('imu_age_ms',99999)>250:return False,'IMU sample stale'
        return True,'BNO055 relative heading; calibration status is informational'
    def update(self,t,g):
        ok,reason=self.quality(t);self.imu_reason=reason
        heading=float(t.get('heading',0))
        key=t['boot_id'];ms=t['ms'];l=t['tl'];r=t['tr']
        if self.previous is None or self.previous[0]!=key:
            self.previous=(key,ms,l,r);self.last_heading=heading if ok else None
            self.imu_valid=False;return False
        _,oldms,oldl,oldr=self.previous;elapsed=(ms-oldms)&0xffffffff
        if not elapsed:return False
        self.previous=(key,ms,l,r);dt=elapsed/1000
        # Never bridge a long telemetry gap with an invented orientation.
        if elapsed>1000:
            self.last_heading=None;self.imu_valid=False;self.imu_segment+=1;self.v=0
            self.imu_reason='Telemetry gap: pose gap omitted';return False
        dl,dr=g.distances(self.tick_delta(l,oldl),self.tick_delta(r,oldr))
        self.dl,self.dr=dl,dr;self.vl=dl/dt;self.vr=dr/dt;self.left+=dl;self.right+=dr
        ds=(dl+dr)/2;da_enc=(dr-dl)/g.track;self.enc_yaw+=da_enc
        self.imu_valid=False;self.imu_delta=0;self.imu_rate=0
        if ok and self.last_heading is not None:
            di=g.imu_sign*((heading-self.last_heading+180)%360-180)
            if abs(di)<=360*dt+3:
                self.imu_valid=True;self.imu_delta=di;self.imu_rate=di/dt;self.imu_yaw+=di
            else:
                self.imu_reason='Heading jump rejected';self.imu_segment+=1
        elif self.last_heading is not None or not ok:
            self.imu_segment+=1
        self.last_heading=heading if ok else None
        da=math.radians(self.imu_delta) if self.prefer_imu and self.imu_valid else da_enc
        self.source=('IMU' if self.imu_valid else 'ENC fallback') if self.prefer_imu else 'ENC'
        # Exact constant-curvature integration of the differential center.
        scale=1.0 if abs(da)<1e-9 else math.sin(da/2)/(da/2)
        self.x+=ds*scale*math.cos(self.yaw+da/2)
        self.y+=ds*scale*math.sin(self.yaw+da/2)
        self.yaw+=da;self.v=ds/dt
        return True

class Primitive:
    def __init__(self,kind,value,odom,g,now,cap=100,min_pwm=80):
        if kind=='distance':
            if not 0<abs(value)<=200:raise ValueError('Distance must be nonzero, within +/-200 cm')
            dl=dr=value*10
        else:
            if not 0<abs(value)<=360:raise ValueError('Turn must be nonzero, within +/-360 degrees')
            arc=g.track*math.radians(value)/2;dl=-arc;dr=arc
        self.targets=(odom.left+dl,odom.right+dr);self.kind=kind;self.value=value
        self.start=now;self.progress_at=now;self.progress=(odom.left,odom.right)
        self.wheel_progress=[odom.left,odom.right];self.wheel_progress_at=[now,now]
        self.origin=(odom.x,odom.y,odom.yaw,odom.imu_yaw,(odom.left+odom.right)/2)
        self.cap=cap;self.min_pwm=min(cap,min_pwm);self.done=False;self.wheel_origins=(odom.left,odom.right);self.reached=[False,False];self.settle_at=None
    def output(self,odom,now):
        if now-self.start>25:raise RuntimeError('Primitive timed out')
        if max(abs(odom.left-self.progress[0]),abs(odom.right-self.progress[1]))>1:
            self.progress=(odom.left,odom.right);self.progress_at=now
        if now-self.progress_at>1.5:raise RuntimeError('No encoder progress: STOP; check polarity, PWM and wiring')
        positions=(odom.left,odom.right)
        errors=(self.targets[0]-odom.left,self.targets[1]-odom.right)
        for i in (0,1):
            travel=positions[i]-self.wheel_origins[i]
            direction=self.targets[i]-self.wheel_origins[i]
            if travel*direction<0 and abs(travel)>5:
                raise RuntimeError('Wheel encoder / motor sign inconsistent: motion cancelled')
            if abs(errors[i])<=3 or errors[i]*direction<=0:self.reached[i]=True
            if abs(positions[i]-self.wheel_progress[i])>1:
                self.wheel_progress[i]=positions[i];self.wheel_progress_at[i]=now
            if not self.reached[i] and now-self.wheel_progress_at[i]>1.5:
                raise RuntimeError(f'Wheel {i+1}: no encoder progress; STOP')
        if all(self.reached):
            if self.settle_at is None:self.settle_at=now
            self.done=now-self.settle_at>=.35
            return 0,0
        def output(i):
            if self.reached[i]:return 0
            return math.copysign(min(self.cap,max(self.min_pwm,abs(errors[i])*1.2)),errors[i])
        return output(0),output(1)

class SingleEncoderMove:
    # Conservative starting gains; tune on the chassis with the wheels lifted.
    kind='distance_imu'
    kp=2.0
    ki=0.05
    kd=0.5
    def __init__(self,value,side,odom,now,cap=100,min_pwm=80):
        if side not in ('left','right'):raise ValueError('Choose the left or right encoder')
        if not 0<abs(value)<=200:raise ValueError('Distance must be nonzero, within +/-200 cm')
        if not odom.imu_valid:raise ValueError('No fresh BNO055 heading; see IMU status')
        self.side=side;self.index=0 if side=='left' else 1
        self.wheel_origin=(odom.left,odom.right)[self.index]
        self.target=self.wheel_origin+value*10;self.direction=1 if value>0 else -1
        self.heading_target=odom.imu_yaw;self.segment=odom.imu_segment
        self.origin=(odom.x,odom.y,odom.yaw,odom.imu_yaw,(odom.left+odom.right)/2)
        self.start=now;self.progress_at=now;self.progress=self.wheel_origin
        self.last_output_at=now;self.integral=0.0;self.cap=max(1,min(int(cap),180))
        self.min_pwm=min(self.cap,max(1,int(min_pwm)));self.done=False;self.settle_at=None
    def distance_cm(self,odom):
        position=(odom.left,odom.right)[self.index]
        return (position-self.wheel_origin)/10
    def output(self,odom,now):
        if now-self.start>25:raise RuntimeError('Single-encoder MOVE timed out')
        if not odom.imu_valid or odom.imu_segment!=self.segment:
            raise RuntimeError('IMU continuity lost: single-encoder MOVE cancelled')
        position=(odom.left,odom.right)[self.index]
        if abs(position-self.progress)>1:
            self.progress=position;self.progress_at=now
        if now-self.progress_at>1.5:
            raise RuntimeError(f'{self.side.title()} encoder: no progress; STOP')
        travel=position-self.wheel_origin
        if travel*self.direction<0 and abs(travel)>5:
            raise RuntimeError('Selected encoder / motor sign inconsistent: motion cancelled')
        error=self.target-position
        if abs(error)<=3 or error*self.direction<=0:
            if self.settle_at is None:self.settle_at=now
            self.done=now-self.settle_at>=.35
            return 0,0
        self.settle_at=None
        dt=max(0.0,min(now-self.last_output_at,.2))
        self.last_output_at=now
        self.integral=max(-20.0,min(20.0,self.integral+(self.heading_target-odom.imu_yaw)*dt))
        heading_error=self.heading_target-odom.imu_yaw
        correction=self.kp*heading_error+self.ki*self.integral-self.kd*odom.imu_rate
        base=min(self.cap,max(self.min_pwm,abs(error)*1.2))
        correction=max(-min(45.0,base),min(min(45.0,base),correction))
        # Positive correction commands a left turn (right wheel faster).
        left=self.direction*base-correction;right=self.direction*base+correction
        return max(-self.cap,min(self.cap,left)),max(-self.cap,min(self.cap,right))


class ImuTurn:
    def __init__(self,value,odom,g,now,cap=85,min_pwm=80,precise=False):
        if not odom.imu_valid:raise ValueError('No fresh BNO055 heading; see IMU status')
        if not 0<abs(value)<=360:raise ValueError('Turn must be within +/-360 deg')
        self.kind='imu_turn';self.value=value;self.start=now;self.cap=min(cap,110);self.min_pwm=min(self.cap,min_pwm)
        self.origin=(odom.x,odom.y,odom.yaw,odom.imu_yaw,(odom.left+odom.right)/2)
        self.wheels=(odom.left,odom.right);self.segment=odom.imu_segment
        self.target=odom.imu_yaw+value;self.done=False;self.good_at=None
        self.progress=odom.imu_yaw;self.progress_at=now
        self.precise=precise;self.coast_until=now;self.pulse_until=None
        self.settle_min=self.settle_max=None;self.fine_started=None
        if precise:self.kind='imu_precise'
    def output(self,odom,now):
        if not odom.imu_valid or odom.imu_segment!=self.segment:
            raise RuntimeError('IMU continuity lost: turn cancelled; no encoder fallback for this test')
        if now-self.start>40:raise RuntimeError('IMU turn timed out')
        if abs(odom.imu_yaw-self.progress)>0.5:
            self.progress=odom.imu_yaw;self.progress_at=now
        error=self.target-odom.imu_yaw
        if self.precise:return self.precise_output(odom,now,error)
        if self.good_at is not None or abs(error)<=2 or error*self.value<=0:
            if self.good_at is None:self.good_at=now
            self.done=now-self.good_at>=0.35
            return 0,0
        self.good_at=None
        if now-self.progress_at>2:raise RuntimeError('No IMU rotation progress')
        # Stop on an incorrect IMU sign instead of continuing the wrong way.
        change=odom.imu_yaw-self.origin[3]
        if change*self.value<0 and abs(change)>8:raise RuntimeError('Turn / IMU sign inconsistent: verify polarity')
        pwm=math.copysign(min(self.cap,max(self.min_pwm,abs(error)*1.2)),error)
        return -pwm,pwm

    def precise_output(self,odom,now,error):
        # Use a stable heading window: differentiating small sensor jitter can
        # report a large rate while the chassis is already stationary.
        rate=odom.imu_rate
        if abs(error)<=2:
            if self.good_at is None:
                self.good_at=now;self.settle_min=self.settle_max=odom.imu_yaw
            self.settle_min=min(self.settle_min,odom.imu_yaw)
            self.settle_max=max(self.settle_max,odom.imu_yaw)
            if self.settle_max-self.settle_min>1:
                self.good_at=now;self.settle_min=self.settle_max=odom.imu_yaw
            self.done=now-self.good_at>=.35
        else:self.good_at=None
        if self.done:return 0,0
        if abs(error)<=12 and self.fine_started is None:self.fine_started=now
        if self.fine_started is not None and now-self.fine_started>8:
            raise RuntimeError('Precise IMU could not settle: stopped. Check PWM / IMU; retry or use standard ROTATE.')
        change=odom.imu_yaw-self.origin[3]
        if change*self.value<0 and abs(change)>8:
            raise RuntimeError('Turn / IMU sign inconsistent: verify polarity')
        if self.pulse_until is not None:
            if now>=self.pulse_until or abs(error)<=2 or error*self.pulse_direction<=0:
                self.pulse_until=None;self.coast_until=now+.35
        if now<self.coast_until or abs(error)<=2:return 0,0
        if now-self.progress_at>2:raise RuntimeError('No IMU rotation progress')
        # Stop ahead of the target using measured angular speed, then remeasure.
        if error*rate>0 and abs(error)<=abs(rate)*.18:
            self.pulse_until=None;self.coast_until=now+.35
            return 0,0
        if abs(error)<=12 and self.pulse_until is None:
            # Firmware ramps by 6 PWM / 10 ms; allow it to reach breakaway
            # PWM before ending the pulse. A 50 ms pulse can never reach 80.
            ramp_time=math.ceil(self.min_pwm/6)*.01
            self.pulse_until=now+ramp_time+.05;self.pulse_direction=1 if error>0 else -1
        pwm=math.copysign(min(self.cap,max(self.min_pwm,abs(error)*1.2)),error)
        return -pwm,pwm


def validate_routine(steps):
    if not isinstance(steps,(list,tuple)) or not 1<=len(steps)<=100:
        raise ValueError('Routine requires 1 to 100 steps')
    result=[]
    for step in steps:
        if not isinstance(step,(list,tuple)) or len(step)!=2:raise ValueError('Each step must be [action, value]')
        kind,value=step
        if kind not in ('distance','turn','imu_precise','wait'):raise ValueError('Actions: distance (cm), turn / imu_precise (degrees), wait (s)')
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError('Finite numeric value required')
        if kind=='wait' and not 0<value<=60:raise ValueError('Wait must be 0–60 seconds, nonzero')
        if not 0<abs(value)<=(200 if kind=='distance' else 360):raise ValueError('Nonzero distance +/-200 cm or turn +/-360 degrees required')
        result.append((kind,float(value)))
    return result

class WheelCapture:
    """Raw encoder capture. A completed OR interrupted run can be calibrated."""
    kind='wheel'
    def __init__(self,side,revolutions,t,g,now,pwm=85):
        if side not in ('left','right') or not math.isfinite(revolutions) or not 0<revolutions<=10:
            raise ValueError('Choose left/right and 0.01 to 10 requested turns')
        self.side=side;self.index=0 if side=='left' else 1;self.requested=revolutions
        self.boot=t['boot_id'];self.start_ticks=(t['tl'],t['tr']);self.end_ticks=self.start_ticks
        self.g=g;self.target=revolutions*(g.tpr_l if side=='left' else g.tpr_r)
        self.pwm=max(1,min(int(pwm),180));self.start=now;self.progress_at=now;self.progress=0
        self.signed_ticks=0;self.other_ticks=0;self.done=False;self.stopping=False
        self.reason='Running';self.stopped_at=None;self.valid=True
    def observe(self,t):
        if t['boot_id']!=self.boot:self.valid=False;return
        self.end_ticks=(t['tl'],t['tr'])
        raw=[Odometry.tick_delta(self.end_ticks[i],self.start_ticks[i]) for i in (0,1)]
        raw=self.g.wheel_ticks(*raw)
        self.signed_ticks=raw[self.index];self.other_ticks=raw[1-self.index]
    def stop(self,reason,now):
        if not self.stopping:self.stopping=True;self.stopped_at=now;self.reason=reason
    def output(self,t,now):
        self.observe(t)
        ticks=abs(self.signed_ticks)
        if not self.valid:self.stop('Robot restarted; capture invalid',now)
        if ticks!=self.progress:self.progress=ticks;self.progress_at=now
        if ticks>=self.target:self.stop('Target reached',now)
        if now-self.progress_at>2.5:self.stop('No new ticks: partial result kept. Check PWM / encoder.',now)
        if now-self.start>60:self.stop('60-second limit: partial result kept',now)
        if self.stopping:
            self.done=now-self.stopped_at>=.45
            return (0,0)
        return (self.pwm,0) if self.index==0 else (0,self.pwm)
    def report(self):
        return {'side':self.side,'requested':self.requested,'ticks':self.signed_ticks,
                'other_ticks':self.other_ticks,'valid':self.valid,'reason':self.reason,
                'done':self.done,'motor_sign':self.g.motor_l if self.index==0 else self.g.motor_r,'active':not self.stopping}

def wheel_correction(report,observed,forward):
    if not report or not report.get('done') or not report.get('valid'):
        raise ValueError('Stop the wheel test and wait for the recorded result')
    if not math.isfinite(observed) or observed<=0:raise ValueError('Enter observed full turns + fraction, e.g. 0.75')
    ticks=report['ticks']
    if abs(ticks)<4:raise ValueError('Too few encoder ticks; check the encoder connection')
    if abs(report['other_ticks'])>max(8,abs(ticks)*.15):
        raise ValueError('Other encoder also moved. Lift wheels and verify channel mapping first')
    tpr=abs(ticks)/observed
    if not 1<=tpr<=100000:raise ValueError('Result outside TPR range')
    if forward not in (-1,1):raise ValueError('Confirm the physical direction during the test')
    motor_sign=report.get('motor_sign',1)*forward
    encoder_sign=(1 if ticks>0 else -1)*forward
    return tpr,encoder_sign,motor_sign


def distance_scale(measured_cm, indicated_cm):
    if not all(math.isfinite(v) and v > 0 for v in (measured_cm, indicated_cm)):
        raise ValueError('Use positive measured and indicated distances')
    ratio = measured_cm / indicated_cm
    if not 0.5 <= ratio <= 2:
        raise ValueError('Scale differs by more than 2x; verify ticks/rev and diameter first')
    return ratio


class Wait:
    kind='wait'
    def __init__(self,value,odom,now):
        self.value=value;self.start=now;self.done=False
        self.origin=(odom.x,odom.y,odom.yaw,odom.imu_yaw,(odom.left+odom.right)/2)
    def output(self,odom,now):
        self.done=now-self.start>=self.value
        return 0,0
