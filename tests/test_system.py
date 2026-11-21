import os,math,socket,time,threading,json,unittest
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from core import Geometry,Odometry,ImuTurn
from app import Engine,Window
from PySide6 import QtWidgets
_runtime_patch=patch.object(Engine,'save_runtime');_runtime_patch.start()

def frame(ms=0,l=0,r=0,heading=0,**kw):
 t=dict(protocol=1,boot_id=1,seq=ms+1,ms=ms,tl=l,tr=r,pl=0,pr=0,armed=0,imu=1,heading=heading,imu_age_ms=0,imu_read_ok=1,cg=3,cs=3,ca=3,cm=3);t.update(kw);return t
class DifferentialTests(unittest.TestCase):
 def setUp(self):self.g=Geometry(diameter=100/math.pi,track=100,tpr_l=100,tpr_r=100,imu_sign=1);self.o=Odometry();self.o.update(frame(),self.g)
 def test_straight_uses_both_encoders_and_imu_heading(self):
  self.o.update(frame(100,10,14),self.g);self.assertAlmostEqual(self.o.x,12);self.assertAlmostEqual(self.o.y,0);self.assertGreater(self.o.enc_yaw,0);self.assertEqual(self.o.source,'IMU')
 def test_in_place_and_then_forward(self):
  for n in range(1,10):self.o.update(frame(n*100,-n*10,n*10,n*10),self.g)
  self.assertAlmostEqual(self.o.x,0);self.assertAlmostEqual(self.o.y,0);self.assertAlmostEqual(math.degrees(self.o.yaw),90)
  self.o.update(frame(1000,10,190,90),self.g);self.assertAlmostEqual(self.o.x,0,places=6);self.assertAlmostEqual(self.o.y,100)
 def test_one_wheel_motion_is_not_ignored(self):
  self.o.update(frame(100,0,20,10),self.g);self.assertGreater(self.o.x,9);self.assertGreater(self.o.y,0)
 def test_wraps(self):
  o=Odometry();o.update(frame(0xfffffff0,2147483647,2147483647,359),self.g);o.update(frame(84,-2147483648,-2147483648,1),self.g)
  self.assertAlmostEqual(o.imu_yaw,2);self.assertAlmostEqual(o.left,1)
 def test_calibration_status_does_not_block_imu_and_fallback(self):
  self.o.update(frame(100,-10,10,20,cs=0,cg=0,cm=0),self.g)
  self.assertEqual(self.o.source,'IMU');self.assertAlmostEqual(math.degrees(self.o.yaw),20)
  self.o.update(frame(200,-20,20,40,imu_read_ok=0),self.g)
  self.assertEqual(self.o.source,'ENC fallback');self.assertGreater(math.degrees(self.o.yaw),20)
 def test_jump_and_gap(self):
  self.o.update(frame(100,0,0,150),self.g);self.assertFalse(self.o.imu_valid);self.assertEqual(self.o.yaw,0)
  self.assertFalse(self.o.update(frame(2000,200,200,90),self.g));self.assertEqual(self.o.x,0)
 def test_imu_turn_360_and_loss(self):
  self.o.update(frame(100),self.g);p=ImuTurn(360,self.o,self.g,0,85);self.assertEqual(p.output(self.o,0),(-85,85))
  self.o.imu_yaw=361;self.assertEqual(p.output(self.o,1),(0,0));p.output(self.o,1.4);self.assertTrue(p.done)
  p=ImuTurn(90,self.o,self.g,2);self.o.imu_valid=False
  with self.assertRaises(RuntimeError):p.output(self.o,2.1)
 def test_manual90_neutral_and_release(self):
  e=Engine();e.source='COCO';e.odom.imu_valid=True;p=dict(type='coco',control_style='manual90',boot_id=1,seq=0,mode='B',deadman=0,motion_enabled=0,drive=0,steer=0)
  def send(**v):p.update(v);p['seq']+=1;e.process_pad(dict(p),time.monotonic())
  send();send(deadman=1,motion_enabled=1);send(drive=1);self.assertEqual(e.live,(85,85));send(drive=0);self.assertIsNone(e.live);send(steer=-1);self.assertEqual(e.live,(85,-85));send(deadman=0);self.assertIsNone(e.live)
 def test_motor_polarity_and_channel_swap(self):
  g=Geometry();self.assertEqual(g.motor_output(80,80),(80,80))
  g.motor_r=-1;self.assertEqual(g.motor_output(80,80),(80,-80))
  g.swap_lr=1;self.assertEqual(g.motor_output(80,0),(0,80));self.assertEqual(g.wheel_ticks(10,20),(10,20));g.swap_encoders=1;self.assertEqual(g.wheel_ticks(10,20),(20,10))
 def test_wheel_calibration_does_not_apply(self):
  e=Engine();e.robot=object();e.last_robot=time.monotonic();e.telemetry=frame(l=100,r=200);e.handle('wheel_mark',None);e.telemetry=frame(l=3512,r=200);e.handle('wheel_finish',10)
  self.assertIn('341.200',e.test_result);self.assertEqual(e.g.tpr_l,341.2)

class PreciseImuTests(unittest.TestCase):
 def make_turn(self,value=180):
  o=Odometry();o.imu_valid=True
  return o,ImuTurn(value,o,Geometry(),0,precise=True)
 def test_overshoot_is_corrected_and_settles(self):
  o,p=self.make_turn();p.output(o,0)
  o.imu_yaw=195
  l,r=p.output(o,1)
  self.assertGreater(l,0);self.assertLess(r,0);self.assertFalse(p.done)
  o.imu_yaw=180;p.output(o,1.1);self.assertFalse(p.done)
  o.imu_yaw=178.5;o.imu_rate=10;p.output(o,1.4);self.assertFalse(p.done)
  o.imu_yaw=180;o.imu_rate=0;p.output(o,1.6);p.output(o,2)
  self.assertTrue(p.done)
 def test_sensor_jitter_does_not_hold_motion_active(self):
  o,p=self.make_turn()
  for n in range(1,21):
   o.imu_yaw=180+(.2 if n%2 else -.2);o.imu_rate=8 if n%2 else -8
   self.assertEqual(p.output(o,n*.05),(0,0))
   if p.done:break
  self.assertTrue(p.done)
 def test_unstable_heading_stops_with_bounded_wait(self):
  o,p=self.make_turn()
  with self.assertRaisesRegex(RuntimeError,'could not settle'):
   for n in range(1,280):
    o.imu_yaw=180+(1.5 if n%2 else -1.5);o.imu_rate=60 if n%2 else -60
    p.output(o,n*.05)
 def test_drift_during_settling_resumes_correction(self):
  o,p=self.make_turn();o.imu_yaw=180;p.output(o,.5)
  o.imu_yaw=185;l,r=p.output(o,1)
  self.assertFalse(p.done);self.assertGreater(l,0);self.assertLess(r,0)
 def test_stopping_and_settling_do_not_count_as_missing_rotation_progress(self):
  o,p=self.make_turn(10);p.output(o,0)
  o.imu_yaw=9;o.imu_rate=0
  for now in (1.,1.5,2.,2.5,3.):
   p.output(o,now)
  self.assertTrue(p.done)
 def test_brakes_early_for_measured_angular_momentum(self):
  o,p=self.make_turn(90);o.imu_yaw=80;o.imu_rate=45
  self.assertEqual(p.output(o,1),(0,0))
 def test_imu_loss_and_discontinuity_cancel(self):
  for field,value in [('imu_valid',False),('imu_segment',1)]:
   o,p=self.make_turn();setattr(o,field,value)
   with self.assertRaises(RuntimeError):p.output(o,.1)
 def test_inertial_turns_with_stationary_encoders(self):
  # 50 ms telemetry/commands and a motor that coasts after PWM goes to zero.
  for target in (180,-180,90,-90,360,5):
   with self.subTest(target=target):
    o,p=self.make_turn(target);speed=0
    for tick in range(800):
     now=tick*.05;l,r=p.output(o,now)
     speed+=(r*.9-speed)*.35
     o.imu_yaw+=speed*.05;o.imu_rate=speed
     if p.done:break
    self.assertTrue(p.done);self.assertLessEqual(abs(o.imu_yaw-target),2)
    self.assertEqual((o.left,o.right),(0,0))
 def test_routine_mode_round_trip_and_dispatch(self):
  from core import validate_routine
  steps=validate_routine(json.loads(json.dumps([['imu_precise',-180]])))
  e=Engine();e.robot=object();e.telemetry={'armed':1,'protocol':1};e.last_robot=time.monotonic();e.odom.imu_valid=True
  e.handle('routine',(steps,85))
  self.assertTrue(e.primitive.precise);self.assertEqual(e.primitive.kind,'imu_precise')
 def test_short_turn_reaches_breakaway_pwm_with_firmware_ramp(self):
  for target in (5,-5,90,-90,180):
   with self.subTest(target=target):
    o,p=self.make_turn(target);actual=command=peak=0
    for tick in range(4000):
     now=tick*.01
     if tick%5==0:_,command=p.output(o,now)
     # COCO_AGV.ino: 6 PWM / 10 ms, zero stops immediately,
     # and polarity reversal passes through zero.
     demand=0 if actual*command<0 else command
     actual=0 if command==0 else actual+max(-6,min(6,demand-actual))
     peak=max(peak,abs(actual))
     o.imu_rate=actual*.5 if abs(actual)>=80 else 0
     o.imu_yaw+=o.imu_rate*.01
     if p.done:break
    self.assertGreaterEqual(peak,80);self.assertTrue(p.done)
    self.assertLessEqual(abs(o.imu_yaw-target),2)

class ManualDiagnosticsTests(unittest.TestCase):
 def test_rejection_explains_why_even_when_relays_are_on(self):
  for field,value,expected in [('source','COCO','Select PC'),('capture',object(),'capture'),('last_robot',0,'stale')]:
   e=Engine();e.robot=object();e.telemetry={'armed':1,'protocol':1};e.last_robot=time.monotonic();setattr(e,field,value)
   with self.assertRaisesRegex(ValueError,expected):e.handle('manual',(110,110))
  for telemetry,expected in [({'armed':0},'Relays OFF'),({'armed':1,'pwm_ok':0},'PWM initialization'),({'armed':1,'estop':1},'E-STOP')]:
   e=Engine();e.robot=object();e.telemetry=dict(protocol=1,**telemetry);e.last_robot=time.monotonic()
   with self.assertRaisesRegex(ValueError,expected):e.handle('manual',(110,110))

class FakeRobot(threading.Thread):
 def __init__(self):
  super().__init__(daemon=True);self.server=socket.socket();self.server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);self.server.bind(('127.0.0.1',4211));self.server.listen();self.server.settimeout(.1);self.quit=False;self.pause=False;self.armed=0;self.estop=0;self.pwm=(0,0);self.commands=[]
 def run(self):
  c=None;buf=b'';start=time.monotonic();last=0;l=r=0
  try:
   while not self.quit:
    if c is None:
     try:c,_=self.server.accept();c.settimeout(.01)
     except socket.timeout:continue
    try:
     b=c.recv(8192)
     if not b:break
     buf+=b
    except socket.timeout:pass
    while b'\n' in buf:
     b,buf=buf.split(b'\n',1);s=b.decode();self.commands.append(s)
     if s=='POWER:1' and not self.estop:self.armed=1
     elif s=='ESTOP':self.estop=1;self.armed=0;self.pwm=(0,0)
     elif s=='RESET_ESTOP':self.estop=0;self.armed=0;self.pwm=(0,0)
     elif s=='POWER:0':self.armed=0;self.pwm=(0,0)
     elif s=='STOP':self.pwm=(0,0)
     elif s.startswith('PWM:'):self.pwm=tuple(map(int,s[4:].split(','))) if self.armed else (0,0)
    now=time.monotonic()
    if not self.pause and now-last>.05:
     last=now;l+=self.pwm[0]/10;r+=self.pwm[1]/10;t=frame(int((now-start)*1000),int(l),int(r),armed=self.armed,estop=self.estop,pl=self.pwm[0],pr=self.pwm[1]);c.sendall((json.dumps(t)+'\n').encode())
  except OSError:pass
  finally:
   if c:c.close()
   self.server.close()
class NetworkTests(unittest.TestCase):
 def test_tcp_pwm_stale_stops_keeps_session_and_ui_watchdog(self):
  robot=FakeRobot();robot.start();e=Engine();e.runtime['cut_relays_on_loss']=False;e.start()
  def pump(seconds,heartbeat=True):
   end=time.monotonic()+seconds
   while time.monotonic()<end:
    if heartbeat:e.post('heartbeat')
    time.sleep(.02)
  try:
   e.post('connect_robot','127.0.0.1');pump(.2);e.post('power',True);pump(.2);e.post('manual',(85,85));pump(.3)
   self.assertGreater(e.odom.x,0);self.assertIn('PWM:85,85',robot.commands)
   robot.pause=True;pump(.7);self.assertEqual(robot.armed,1);self.assertEqual(robot.pwm,(0,0));self.assertIsNotNone(e.robot)
   robot.pause=False;pump(.2);e.post('power',True);pump(.2);self.assertEqual(robot.armed,1);pump(.8,False);self.assertEqual(robot.armed,1);self.assertEqual(robot.pwm,(0,0))
   robot.pause=True;pump(3.3);robot.quit=True;robot.join(1)
   robot=FakeRobot();robot.armed=1;robot.start();pump(2.5)
   self.assertIsNotNone(e.robot);self.assertLess(time.monotonic()-e.last_robot,.5)
   self.assertEqual(robot.armed,1);self.assertEqual(robot.pwm,(0,0));self.assertIsNone(e.live);self.assertNotIn("POWER:0",robot.commands)
  finally:e.quit=True;e.join(2);robot.quit=True;robot.join(2)
class GuiTests(unittest.TestCase):
 def test_direction_buttons_send_pwm_and_release_stops(self):
  from PySide6 import QtCore,QtTest
  app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
  robot=FakeRobot();robot.start()
  def pump(seconds):
   end=time.monotonic()+seconds
   while time.monotonic()<end:app.processEvents();time.sleep(.01)
  with patch.object(Engine,'save_settings'):
   w=Window()
   try:
    w.show();w.activateWindow();w.engine.post('connect_robot','127.0.0.1');pump(.3)
    w.engine.post('power',True);pump(.2)
    for name,expected in [('FORWARD',(110,110)),('REVERSE',(-110,-110)),('LEFT',(-110,110)),('RIGHT',(110,-110))]:
     button=next(b for b in w.findChildren(QtWidgets.QPushButton) if b.text()==name+'\nHold')
     QtTest.QTest.mousePress(button,QtCore.Qt.MouseButton.LeftButton);pump(.3)
     self.assertEqual(robot.pwm,expected);self.assertEqual(w.engine.sent_pwm,expected)
     self.assertIn(f'PWM AGV L,R: {expected[0]}, {expected[1]}',w.motor_status.text())
     QtTest.QTest.mouseRelease(button,QtCore.Qt.MouseButton.LeftButton);pump(.15)
     self.assertEqual(robot.pwm,(0,0))
   finally:w.close();app.processEvents();robot.quit=True;robot.join(2)
 def test_precise_stop_timeout_and_manual_recovery_without_restart(self):
  app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
  robot=FakeRobot();robot.start()
  def pump(seconds):
   end=time.monotonic()+seconds
   while time.monotonic()<end:app.processEvents();time.sleep(.01)
  with patch.object(Engine,'save_settings'):
   w=Window()
   try:
    w.show();w.engine.post('connect_robot','127.0.0.1');pump(.3)
    w.engine.post('power',True);pump(.2)
    w.test('imu_precise',90);pump(.25)
    self.assertIsNotNone(w.engine.primitive)
    stop=next(b for b in w.findChildren(QtWidgets.QPushButton) if b.text()=='STOP')
    stop.click();pump(.15)
    self.assertIsNone(w.engine.primitive);self.assertEqual(robot.pwm,(0,0))
    w.drive((1,1));pump(.2);self.assertGreater(robot.pwm[0],0)
    w.stop();pump(.1)
    # The fake robot has no heading progress: an error must release the action.
    w.test('imu_precise',90);pump(2.4)
    self.assertIsNone(w.engine.primitive);self.assertTrue(w.engine.is_alive())
    self.assertIn('No IMU rotation progress',w.notice.text())
    self.assertLess(time.monotonic()-w.engine.last_ui,.3)
    w.drive((1,1));pump(.2);self.assertGreater(robot.pwm[0],0)
    w.stop();pump(.1)
    w.test('turn',90);pump(.2)
    self.assertIsNotNone(w.engine.primitive);self.assertTrue(w.engine.primitive.precise)
    self.assertLess(robot.pwm[0],0);self.assertGreater(robot.pwm[1],0)
    stop.click();pump(.15);self.assertEqual(robot.pwm,(0,0))
   finally:w.close();app.processEvents();robot.quit=True;robot.join(2)
 def test_integrated_calibration_ui(self):
  app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([]);w=Window()
  try:
   w.show();w.tabs.setCurrentIndex(1)
   for _ in range(8):app.processEvents();time.sleep(.025)
   self.assertFalse(w.plot.getAxis('left').autoSIPrefix)
   self.assertEqual(len(w.body.getData()[0]),5)
   w.last_report={'side':'left','ticks':256,'other_ticks':0,'valid':True,'done':True,'motor_sign':1}
   w.observed.setValue(.75);w.preview_correction();self.assertIn('341.333',w.preview.text())
   w.grab().save(str(__import__('pathlib').Path(__file__).resolve().parents[2]/'.build/coco05_preview.png'))
  finally:w.close();app.processEvents()

class Version04Tests(unittest.TestCase):
 def test_routine_validation_and_cancellation(self):
  from core import validate_routine
  self.assertEqual(validate_routine([['distance',20],['turn',-90]]),[('distance',20.0),('turn',-90.0)])
  for bad in ([],[['distance',0]],[['distance',201]],[['turn',float('nan')]],[['bad',2]]):
   with self.assertRaises(ValueError):validate_routine(bad)
  class Sink:
   def __init__(self):self.commands=[]
   def send(self,s):self.commands.append(s)
  e=Engine();e.robot=Sink();e.telemetry={'armed':1,'protocol':1};e.last_robot=time.monotonic();e.odom.imu_valid=True
  e.handle('routine',([('distance',20),('turn',90)],85));self.assertEqual(len(e.routine),1)
  e.halt();self.assertFalse(e.routine);self.assertIsNone(e.primitive);self.assertEqual(e.robot.commands[-1],'STOP\n')
  e.handle('stop',True);self.assertEqual(e.robot.commands[-1],'POWER:0\n')
 def test_partial_stalled_capture_is_calibratable(self):
  from core import WheelCapture,wheel_correction
  p=WheelCapture('left',1,frame(),Geometry(),0,110)
  self.assertEqual(p.output(frame(100,l=-256),.1),(110,0))
  self.assertEqual(p.output(frame(3000,l=-256),3),(0,0))
  p.output(frame(3500,l=-256),3.5);report=p.report()
  self.assertTrue(report['done']);tpr,sign,motor=wheel_correction(report,.75,1)
  self.assertAlmostEqual(tpr,256/.75);self.assertEqual((sign,motor),(-1,1))
 def test_manual_stop_capture_and_backward_motor(self):
  from core import WheelCapture,wheel_correction
  g=Geometry(motor_r=-1);p=WheelCapture('right',1,frame(),g,0,110)
  p.output(frame(100,r=100),.1);p.stop('Operator stop',.2)
  p.output(frame(700,r=120),.7)
  self.assertTrue(p.done);self.assertEqual(wheel_correction(p.report(),.5,-1),(240,-1,1))
 def test_bad_capture_rejected(self):
  from core import wheel_correction
  base=dict(side='left',ticks=300,other_ticks=0,valid=True,done=True,motor_sign=1)
  for extra in ({'ticks':0},{'valid':False},{'other_ticks':250},{'done':False}):
   with self.assertRaises(ValueError):wheel_correction(dict(base,**extra),.75,1)
 def test_world_square_returns_to_origin(self):
  g=Geometry(diameter=100/math.pi,track=100,tpr_l=100,tpr_r=100,imu_sign=1);o=Odometry();ms=l=r=h=0;o.update(frame(),g)
  for side in range(4):
   ms+=100;l+=100;r+=100;o.update(frame(ms,l,r,h%360),g)
   for n in range(9):
    ms+=100;l-=10;r+=10;h+=10;o.update(frame(ms,l,r,h%360),g)
  self.assertAlmostEqual(o.x,0,places=6);self.assertAlmostEqual(o.y,0,places=6)
 def test_routine_sequence_over_tcp(self):
  robot=FakeRobot();robot.start();e=Engine();messages=[];e.emit=messages.append;e.runtime['cut_relays_on_loss']=False;e.start()
  def pump(sec):
   end=time.monotonic()+sec
   while time.monotonic()<end:e.post('heartbeat');time.sleep(.02)
  try:
   e.post('connect_robot','127.0.0.1');pump(.3);e.post('power',True);pump(.2)
   e.post('routine',([('distance',2),('distance',-2)],50));pump(6)
   self.assertIsNone(e.primitive,str(messages));self.assertFalse(e.routine,str(messages));self.assertEqual(e.routine_step,2,str(messages))
   self.assertTrue(any(s.startswith('PWM:-') for s in robot.commands));self.assertEqual(robot.armed,1)
  finally:e.quit=True;e.join(2);robot.quit=True;robot.join(2)

class FixRegressionTests(unittest.TestCase):
 def test_learned_encoder_signs_fix_pivot_translation(self):
  from core import wheel_correction
  g=Geometry(imu_sign=-1);base=dict(other_ticks=0,done=True,valid=True,motor_sign=1)
  g.tpr_l,g.sign_l,g.motor_l=wheel_correction(dict(base,side='left',ticks=-300),1,1)
  g.tpr_r,g.sign_r,g.motor_r=wheel_correction(dict(base,side='right',ticks=300),1,1)
  o=Odometry();o.update(frame(),g)
  for n in range(1,10):o.update(frame(n*100,n*10,n*10,(-n*10)%360),g)
  self.assertAlmostEqual(o.x,0);self.assertAlmostEqual(o.y,0);self.assertAlmostEqual(math.degrees(o.yaw),90)
 def test_partial_capture_pipeline_apply_save_and_no_bench_trace(self):
  from pathlib import Path
  import tempfile
  robot=FakeRobot();robot.start();e=Engine();e.runtime['cut_relays_on_loss']=False;e.start()
  def pump(sec):
   end=time.monotonic()+sec
   while time.monotonic()<end:e.post('heartbeat');time.sleep(.02)
  try:
   e.post('connect_robot','127.0.0.1');pump(.25);e.post('power',True);pump(.2)
   e.post('wheel_test',('left',1,110));pump(.5);e.post('stop',False);pump(.7)
   report=e.last_wheel_test;self.assertIsNotNone(report);self.assertTrue(report['done']);self.assertGreater(report['ticks'],0)
   self.assertAlmostEqual(e.odom.x,0);self.assertAlmostEqual(e.odom.y,0)
   expected=report['ticks']/.75
   with patch.object(e,'save_settings') as saved:
    e.post('wheel_apply',(.75,1));pump(.2);saved.assert_called_once()
   self.assertAlmostEqual(e.g.tpr_l,expected);self.assertEqual(robot.armed,1)
  finally:e.quit=True;e.join(2);robot.quit=True;robot.join(2)

if __name__=='__main__':unittest.main()
