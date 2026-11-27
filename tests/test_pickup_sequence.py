import unittest
import time
from unittest.mock import Mock
from pickup_sequence import PickupSequence
from app import Engine


class PickupTests(unittest.TestCase):
    def test_pickup_steps_wait_and_keep_gripper_closed_through_safe_rescue(self):
        seq=PickupSequence(0)
        from pickup_sequence import pickup_servo_target
        step=seq.next_step(0)
        now=0.
        while step:
            name,joints,grip=step
            self.assertEqual(grip,15 if name in ('Cerrar pinza','DEPART','SAFE_Rescue') else 75)
            self.assertEqual(joints[0],115)
            channels=pickup_servo_target(joints,grip)
            seq.observe(channels,True,now+1)
            now+=3.1
            step=seq.observe(channels,False,now)
            if seq.wait_until is not None:
                now=seq.wait_until
                step=seq.observe(channels,False,now)
        self.assertTrue(seq.done)

    def test_folded_descent_then_level_insertion(self):
        import numpy as np
        from arm import ArmKinematics
        model=ArmKinematics()
        poses=[np.round(joints) for _,joints,_ in PickupSequence(0).steps]
        def tip(joints):
            xyz,frames=model.compute_fk(joints)
            return np.hypot(*xyz[:2]),xyz[2],frames[-1][2,0]
        start,lowered=map(tip,poses[:2])
        self.assertLess(lowered[0],start[0])
        self.assertLess(lowered[1],start[1]-40)
        for index in range(2,8):
            first,last=poses[index:index+2]
            for joints in np.linspace(first,last,21):
                radius,height,pitch_sine=tip(joints)
                self.assertTrue(20 < height < 30)
                if index==6:
                    self.assertLess(abs(pitch_sine),.025)
        self.assertGreater(tip(poses[7])[0]-tip(poses[6])[0],30)
        self.assertEqual(tuple(poses[8]),(115,14,96,163))

    def test_release_opens_only_after_reaching_drop_pose(self):
        seq=PickupSequence(0,release=True)
        self.assertEqual(seq.next_step(0),('DEPART',(120,55.2,81.5,123.6),15))
        seq.observe([120,55,82,90,124,15],True,1)
        self.assertEqual(seq.observe([120,55,82,90,124,15],False,3.1),('PICK',(120,14.3,95.8,162.9),15))
        seq.observe([120,14,96,90,163,15],True,4)
        self.assertIsNone(seq.observe([120,14,96,90,163,15],False,6.2))
        self.assertIsNone(seq.observe([120,14,96,90,163,15],False,7.1))
        self.assertEqual(seq.observe([120,14,96,90,163,15],False,7.3),('Abrir pinza',(120,14.3,95.8,162.9),75))
        seq.observe([120,14,96,90,163,75],True,8.3)
        self.assertIsNone(seq.observe([120,14,96,90,163,75],False,10.4))
        self.assertEqual(seq.observe([120,14,96,90,163,75],False,12.5),('Approach',(120,111.1,160.6,125.1),75))
        seq.observe([120,111,161,90,125,75],True,13.5)
        self.assertIsNone(seq.observe([120,111,161,90,125,75],False,15.6))
        self.assertTrue(seq.done)

    def test_pose_change_during_wait_cancels(self):
        seq=PickupSequence(0)
        while seq.next_step(0)[0]!='PICK':
            pass
        seq.observe([115,14,96,90,163,75],True,1)
        seq.observe([115,14,96,90,163,75],False,3.1)
        with self.assertRaisesRegex(RuntimeError,'espera'):
            seq.observe([115,14,96,90,163,70],False,3.2)

    def test_does_not_advance_without_busy_confirmation(self):
        seq=PickupSequence(0);seq.next_step(0)
        self.assertIsNone(seq.observe([115,149,180,90,7,75],False,3.1))
        with self.assertRaisesRegex(RuntimeError,'confirmación'):seq.observe([115,149,180,90,7,75],False,6.1)

    def test_release_requires_safe_estimate_and_stop_cancels(self):
        engine=Engine();self.addCleanup(engine.pool.shutdown)
        engine.robot=Mock();engine.last_robot=time.monotonic()
        engine.telemetry={'protocol':1,'armed':1,'pwm_ok':1,'pca_ready':1,'arm_protocol':1,'estop':0,
                          'arm_angles':[108,149,180,90,7,15]}
        with self.assertRaisesRegex(ValueError,'SAFE_Rescue.*pinza a 15'):engine.handle('release_start',None)
        engine.telemetry['arm_angles']=[115,110,121,90,100,25]
        with self.assertRaisesRegex(ValueError,'SAFE_Rescue.*pinza a 15'):engine.handle('release_start',None)
        engine.telemetry['arm_angles'][5]=15
        engine.handle('release_start',None)
        self.assertTrue(engine.pickup_sequence.release)
        engine.robot.send.assert_called_once_with('ARM:120,55,82,90,124,15,3000\n')
        engine.handle('stop',False)
        self.assertIsNone(engine.pickup_sequence)
