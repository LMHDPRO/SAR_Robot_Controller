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

    def test_arm_block_advances_to_wait_in_running_engine(self):
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
                    if e.robot.target[5]==75 and e.routine_step==3 and not e.routine_running and sum(m.startswith('ARM:') for m in e.robot.commands)==11:break
                self.assertEqual(sum(m.startswith('ARM:') for m in e.robot.commands),11)
                self.assertEqual(e.robot.target,[108,149,180,90,7,75])
                self.assertEqual(e.routine_step,3)
                self.assertFalse(e.routine_running)
            ref={'identity':{'url':'test'},'xyz_camera_mm':[0,0,180],'rvec':[0,0,0]}
            e.post('routine',([['wait',.1],['aruco',None]],85,'left',ref))
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                e.post('heartbeat');time.sleep(.03)
                if e.routine_step==2 and isinstance(e.primitive,ArucoRoutineBlock):break
            self.assertIsInstance(e.primitive,ArucoRoutineBlock)
            self.assertTrue(e.routine_running)
            self.assertNotIn('STOP\n',e.robot.commands)
        finally:
            e.quit=True;e.join(2)
