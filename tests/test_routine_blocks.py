import math
import time
import unittest
from unittest.mock import Mock

from app import Engine
from core import validate_routine
from routine_blocks import ArmRoutineBlock, ArucoRoutineBlock


class RoutineBlockTests(unittest.TestCase):
    def setUp(self):
        self.value={'joints':[90,126,74,106],'rotation':90,'gripper':75,'duration_ms':1000}
        self.engine=Engine();self.addCleanup(self.engine.pool.shutdown)
        self.engine.robot=Mock();self.engine.last_robot=time.monotonic();self.engine.odom.imu_valid=True
        self.engine.telemetry={'protocol':1,'armed':1,'pwm_ok':1,'pca_ready':1,'arm_protocol':1,'estop':0,
                               'pl':0,'pr':0,'arm_angles':[90,126,74,90,106,75],'arm_busy':0}

    def test_validation_mixes_blocks_and_rejects_unsafe_values(self):
        blocks=validate_routine([['distance',10],['arm',self.value],['wait',1],['aruco',None]])
        self.assertEqual([item[0] for item in blocks],['distance','arm','wait','aruco'])
        with self.assertRaisesRegex(ValueError,'10–75'):validate_routine([['arm',dict(self.value,gripper=9)]])
        with self.assertRaises(ValueError):validate_routine([['arm_xyz',dict(self.value,xyz=[0,float('nan'),2])]])

    def test_arm_block_waits_for_busy_and_matching_telemetry(self):
        block=ArmRoutineBlock(self.engine,'arm',self.value,0)
        self.assertEqual(block.output(self.engine.odom,1.1),(0,0));self.assertFalse(block.done)
        self.engine.telemetry['arm_busy']=1;block.output(self.engine.odom,1.2)
        self.engine.telemetry['arm_busy']=0;self.engine.telemetry['arm_angles'][5]=60
        block.output(self.engine.odom,1.3);self.assertFalse(block.done)
        self.engine.telemetry['arm_angles'][5]=75;block.output(self.engine.odom,1.4)
        self.assertTrue(block.done)

    def test_routine_blocks_external_motion_and_stop_cancels(self):
        self.engine.handle('routine',([['arm',self.value],['distance',10]],85,'left'))
        self.assertTrue(self.engine.routine_running)
        with self.assertRaisesRegex(ValueError,'STOP'):self.engine.handle('manual',(85,85))
        self.engine.handle('stop',False)
        self.assertFalse(self.engine.routine_running);self.assertIsNone(self.engine.primitive)
        self.assertFalse(self.engine.routine)

    def test_aruco_requires_reference_and_xyz_rejects_unreachable_point(self):
        with self.assertRaisesRegex(ValueError,'referencia'):
            self.engine.handle('routine',([['aruco',None]],85,'left'))
        xyz={'xyz':[10000,0,0],'rotation':90,'gripper':75,'duration_ms':1000}
        with self.assertRaisesRegex(ValueError,'sin solución'):
            self.engine.handle('routine',([['arm_xyz',xyz]],85,'left'))

    def test_aruco_rejects_changed_camera_before_motion(self):
        ref={'identity':{'url':'first'},'xyz_camera_mm':[0,0,180],'rvec':[0,0,0]}
        block=ArucoRoutineBlock(self.engine,ref,0)
        self.engine.aruco_sample={'identity':{'url':'other'}}
        with self.assertRaisesRegex(RuntimeError,'cambió la cámara'):
            block.output(self.engine.odom,.1)

    def test_can_start_from_selected_aruco_step(self):
        ref={'xyz_camera_mm':[0,0,210.1],'rvec':[0,0,0]}
        self.engine.handle('routine',([['distance',100],['aruco',None],['wait',1]],85,'left',ref,1))
        self.assertEqual((self.engine.routine_step,self.engine.routine_total),(2,3))
        self.assertIsInstance(self.engine.primitive,ArucoRoutineBlock)
        self.assertEqual(list(self.engine.routine),[('wait',1)])

    def test_can_start_from_pick_after_an_earlier_aruco_step_without_camera(self):
        self.engine.handle('routine',([['aruco',None],['pickup',None],['wait',1]],85,'left',None,1))
        self.assertEqual((self.engine.routine_step,self.engine.routine_total),(2,3))
        self.assertIsNotNone(self.engine.pickup_sequence)
        self.assertEqual(list(self.engine.routine),[('wait',1)])
        self.engine.robot.send.assert_called_once_with('ARM:112,111,161,90,125,75,3000\n')

    def test_aruco_retry_window_expires_and_stop_clears_the_checkpoint(self):
        e=self.engine
        ref={'xyz_camera_mm':[0,0,210.1],'rvec':[0,0,0]}
        e.handle('routine',([['aruco',None],['wait',1]],85,'left',ref))
        now=time.monotonic()
        e.pause_aruco_routine('ArUco 12 sin lectura válida.',now)
        deadline=e.routine_retry_deadline
        e.aruco_sample=dict(ref,id=12,captured_at=time.monotonic())
        e.auto_retry_aruco_routine(deadline)
        self.assertIsNone(e.routine_retry_at)
        self.assertIsNotNone(e.routine_retry)
        self.assertIsNone(e.primitive)
        e.handle('stop',False)
        self.assertIsNone(e.routine_retry)
        self.assertIsNone(e.routine_retry_deadline)

    def test_aligned_aruco_keeps_pick_queued_until_wheels_confirm_stop(self):
        e=self.engine
        now=time.monotonic()
        ref={'xyz_camera_mm':[0,0,218.4],'rvec':[0,0,0],'target_distance_mm':218.4}
        e.handle('routine',([['aruco',None],['pickup',None],['wait',1]],85,'left',ref))
        for tick in range(1,5):
            timestamp=now+tick*.05
            e.aruco_sample=dict(ref,captured_at=timestamp,id=12)
            e.primitive.output(e.odom,timestamp)
        self.assertTrue(e.primitive.done)
        e.primitive=None;e.next_step_at=now+.7
        e.telemetry['pl']=6
        e.start_next_routine_step(e.next_step_at)
        self.assertEqual(list(e.routine),[('pickup',None),('wait',1)])
        self.assertEqual(e.routine_step,1)
        e.robot.send.assert_not_called()
        e.telemetry['pl']=0
        e.start_next_routine_step(e.next_step_at+.1)
        self.assertEqual(e.routine_step,2)
        self.assertEqual(list(e.routine),[('wait',1)])
        e.robot.send.assert_called_once_with('ARM:112,111,161,90,125,75,3000\n')

    def test_arm_pickup_release_and_aruco_retry_in_running_engine(self):
        class Link:
            def __init__(self):self.seq=0;self.target=[90,126,74,90,106,75];self.until=0;self.commands=[]
            def send(self,msg):
                self.commands.append(msg)
                if msg.startswith('ARM:'):
                    values=[int(x) for x in msg.strip()[4:].split(',')]
                    self.target=values[:6];self.until=time.monotonic()+.15
            def receive(self):
                self.seq+=1
                return [dict(tl=0,tr=0,ms=self.seq*50,seq=self.seq,boot_id=1,armed=1,pl=0,pr=0,
                             heading=0,imu=1,imu_read_ok=1,imu_age_ms=0,protocol=1,pwm_ok=1,
                             pca_ready=1,arm_protocol=1,arm_angles=self.target,arm_busy=int(time.monotonic()<self.until))]
            def close(self):pass
        e=self.engine;e.robot=Link();e.last_robot=time.monotonic();e.start()
        try:
            e.post('routine',([['wait',.1],['arm',dict(self.value,duration_ms=100)],['wait',.1]],85,'left'))
            deadline=time.monotonic()+4
            while time.monotonic()<deadline:
                e.post('heartbeat');time.sleep(.03)
                if e.routine_step==3 and not e.routine_running:break
            self.assertEqual(e.routine_step,3,e.test_result)
            self.assertFalse(e.routine_running,e.test_result)
            self.assertTrue(any(m.startswith('ARM:') for m in e.robot.commands))
            self.assertNotIn('STOP\n',e.robot.commands)
            # Advance only the sequence clock; exercise the real telemetry and queue.
            from pickup_sequence import PickupSequence
            from unittest.mock import patch
            observe = PickupSequence.observe
            origins = {}
            def accelerated(sequence, channels, busy, now):
                origin = origins.setdefault(sequence, sequence.started)
                return observe(sequence, channels, busy, origin + (now-origin)*8)
            with patch.object(PickupSequence, 'observe', accelerated):
                e.post('routine', ([['pickup',None],['release',None],['wait',.1]],85,'left'))
                deadline=time.monotonic()+12
                while time.monotonic()<deadline:
                    e.post('heartbeat');time.sleep(.03)
                    if e.routine_step==3 and not e.routine_running and sum(m.startswith('ARM:') for m in e.robot.commands)==10:break
                handling_commands=[m for m in e.robot.commands if m.startswith('ARM:')][1:]
                self.assertEqual(handling_commands,[
                    'ARM:112,111,161,90,125,75,3000\n',
                    'ARM:115,14,96,90,163,75,3000\n',
                    'ARM:115,14,96,90,163,15,3000\n',
                    'ARM:115,55,82,90,124,15,3000\n',
                    'ARM:115,110,121,90,100,15,3000\n',
                    'ARM:115,55,82,90,124,15,3000\n',
                    'ARM:115,14,96,90,163,15,3000\n',
                    'ARM:115,14,96,90,163,75,3000\n',
                    'ARM:112,111,161,90,125,75,3000\n',
                ])
                self.assertEqual(e.routine_step,3)
                self.assertFalse(e.routine_running)
            ref={'identity':{'url':'test'},'xyz_camera_mm':[0,0,180],'rvec':[0,0,0]}
            e.post('routine',([['wait',.1],['aruco',None],['pickup',None],['wait',.1]],85,'left',ref))
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                e.post('heartbeat');time.sleep(.03)
                if e.routine_step==2 and isinstance(e.primitive,ArucoRoutineBlock):break
            self.assertIsInstance(e.primitive,ArucoRoutineBlock)
            self.assertTrue(e.routine_running)
            self.assertNotIn('STOP\n',e.robot.commands)
            deadline=time.monotonic()+3
            while time.monotonic()<deadline and e.routine_retry is None:
                e.post('heartbeat');time.sleep(.03)
            self.assertIsNotNone(e.routine_retry)
            self.assertEqual((e.routine_step,e.sent_pwm),(2,(0,0)))
            self.assertIn('STOP\n',e.robot.commands)
            e.post('routine_retry')
            time.sleep(.1)
            self.assertIsNotNone(e.routine_retry)
            self.assertFalse(e.routine_running)
            rotated_pose=dict(ref,id=12,xyz_camera_mm=[0,2.6,210.1],rvec=[0,0,math.radians(92.9)])
            with patch.object(PickupSequence, 'observe', accelerated):
                deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    e.post('heartbeat')
                    e.post('aruco_sample',dict(rotated_pose,captured_at=time.monotonic()))
                    time.sleep(.06)
                    if e.routine_step==4 and not e.routine_running:break
            self.assertIsNone(e.routine_retry)
            self.assertEqual(e.routine_step,4,e.test_result)
            self.assertFalse(e.routine_running)
            self.assertEqual(e.robot.target,[115,110,121,90,100,15])
        finally:
            e.quit=True;e.join(2)
