import unittest,time,socket
from unittest.mock import patch
from app import Engine,Link
from core import Geometry,distance_scale
class Sink:
 def __init__(self):self.messages=[]
 def send(self,s):self.messages.append(s)
 def urgent(self,s):self.messages.append(s)
 def close(self):pass
class SarTests(unittest.TestCase):
 def ready(self):
  e=Engine();e.robot=Sink();e.last_robot=time.monotonic();e.telemetry={'protocol':1,'armed':1,'boot_id':17,'estop':0};return e
 def test_estop_cancels_every_owner_and_cannot_rearm(self):
  e=self.ready();e.live=(110,110);e.routine.append(('distance',20));e.handle('estop',None)
  self.assertIsNone(e.live);self.assertFalse(e.routine);self.assertTrue(e.estop_latched)
  self.assertEqual(e.robot.messages[-1],'ESTOP\n')
  with self.assertRaises(ValueError):e.handle('power',True)
  with self.assertRaises(ValueError):e.handle('manual',(80,80))
  e.source='COCO';e.process_pad({'type':'coco','drive':1,'deadman':1},time.monotonic());self.assertIsNone(e.live)
 def test_reset_requires_ack_and_does_not_arm(self):
  e=self.ready();e.handle('estop',None);e.handle('estop_reset',None)
  self.assertTrue(e.reset_pending);self.assertTrue(e.estop_latched);self.assertEqual(e.robot.messages[-1],'RESET_ESTOP\n')
  self.assertNotIn('POWER:1\n',e.robot.messages)
 def test_both_loss_policies_always_stop(self):
  for cut in (False,True):
   e=self.ready();e.runtime['cut_relays_on_loss']=cut;e.live=(90,90);e.halt_loss()
   self.assertIsNone(e.live);self.assertEqual(e.robot.messages[-1],'POWER:0\n' if cut else 'STOP\n')
 def test_distance_calibration_preserves_ticks(self):
  e=self.ready();e.g.tpr_l=662.667;e.g.tpr_r=669.333;e.handle('distance_begin',None);e.odom.left=500;e.odom.right=500
  with patch.object(e,'save_settings'):e.handle('distance_apply',40)
  self.assertAlmostEqual(e.g.scale_l,.8);self.assertAlmostEqual(e.g.scale_r,.8)
  self.assertEqual(e.g.tpr_l,662.667);self.assertEqual(e.g.tpr_r,669.333);self.assertIsNone(e.distance_reference)
 def test_reject_turn_as_distance_calibration(self):
  e=self.ready();e.handle('distance_begin',None);e.odom.left=-500;e.odom.right=500
  with self.assertRaises(ValueError):e.handle('distance_apply',40)
 def test_distance_scales_and_invalid_measurement(self):
  self.assertAlmostEqual(distance_scale(80,100),.8)
  for value in (0,-1,float('nan')):
   with self.assertRaises(ValueError):distance_scale(value,100)
  l,r=Geometry(scale_l=.8,scale_r=1.2).distances(100,100);self.assertAlmostEqual(r/l,1.5)
 def test_stop_invalidates_queued_motion(self):
  e=self.ready();e.post('manual',(110,110));old=e.commands.get_nowait();e.post('stop',False)
  self.assertNotEqual(old[2],e.epoch);self.assertEqual(e.urgent_commands.get_nowait()[0],'stop')
 def test_async_connection_does_not_block_engine(self):
  e=Engine()
  def slow(*args):time.sleep(.15);return Sink()
  with patch('app.Link',side_effect=slow),patch.object(e,'save_runtime'):
   start=time.monotonic();e.request_link('pad','127.0.0.1');self.assertLess(time.monotonic()-start,.05)
   e.pending_links['pad'][1].result(timeout=3);e.poll_links();self.assertIsInstance(e.pad,Sink)
  e.pool.shutdown()
 def test_urgent_discards_unsent_motion(self):
  a,b=socket.socketpair();a.setblocking(False);link=Link.__new__(Link);link.socket=a;link.out=b'PWM:200,200\n';link.pending_at=time.monotonic()
  try:link.urgent('ESTOP\n');self.assertEqual(b.recv(100),b'\nESTOP\n')
  finally:a.close();b.close()
 def test_wait_route(self):
  from core import Wait,Odometry,validate_routine
  self.assertEqual(validate_routine([['wait',1]]),[('wait',1.)])
  with self.assertRaises(ValueError):validate_routine([['wait',-1]])
  o=Odometry();w=Wait(1,o,0);self.assertEqual(w.output(o,.5),(0,0));self.assertFalse(w.done)
  w.output(o,1);self.assertTrue(w.done)
 def test_estop_and_acknowledged_reset_over_tcp(self):
  from test_system import FakeRobot
  robot=FakeRobot();robot.start();e=Engine();e.start()
  def pump(seconds):
   end=time.monotonic()+seconds
   while time.monotonic()<end:e.post('heartbeat');time.sleep(.01)
  try:
   e.post('connect_robot','127.0.0.1');pump(.3);e.post('power',True);pump(.2)
   e.post('manual',(90,90));pump(.2);self.assertEqual(robot.pwm,(90,90))
   e.post('estop');pump(.2);self.assertEqual(robot.armed,0);self.assertEqual(robot.pwm,(0,0));self.assertTrue(e.estop_latched)
   e.post('power',True);pump(.1);self.assertEqual(robot.armed,0)
   e.post('estop_reset');pump(.3);self.assertFalse(e.estop_latched);self.assertFalse(e.reset_pending);self.assertEqual(robot.armed,0)
   e.post('power',True);pump(.2);self.assertEqual(robot.armed,1);self.assertEqual(robot.pwm,(0,0))
  finally:e.quit=True;e.join(2);robot.quit=True;robot.join(2)
 def test_one_stalled_wheel_stops_distance_motion(self):
  from core import Primitive,Odometry
  o=Odometry();p=Primitive('distance',50,o,Geometry(),0)
  o.right=50
  with self.assertRaisesRegex(RuntimeError,'Wheel 1'):p.output(o,1.6)
 def test_protocol_mismatch_explained(self):
  e=self.ready();e.telemetry.pop('protocol')
  with self.assertRaisesRegex(ValueError,'firmware SAR'):e.handle('manual',(80,80))
if __name__=='__main__':unittest.main()
