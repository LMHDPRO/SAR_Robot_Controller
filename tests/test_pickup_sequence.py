import unittest
import time
from unittest.mock import Mock
from pickup_sequence import PickupSequence
from app import Engine


class PickupTests(unittest.TestCase):
    def test_pickup_steps_wait_and_keep_gripper_closed_through_safe_rescue(self):
        seq=PickupSequence(0)
        self.assertEqual(seq.next_step(0),('Approach',(111.7,111.1,160.6,125.1),75))
        def complete(channels,start):
            self.assertIsNone(seq.observe(channels,True,start+1))
            return seq.observe(channels,False,start+3.1)
        step=complete([112,111,161,90,125,75],0)
        self.assertEqual(step,('PICK',(115.2,14.3,95.8,162.9),75))
        self.assertIsNone(complete([115,14,96,90,163,75],3.1))
        self.assertIsNone(seq.observe([115,14,96,90,163,75],False,7.0))
        step=seq.observe([115,14,96,90,163,75],False,7.2)
        self.assertEqual(step,('Cerrar pinza',(115.2,14.3,95.8,162.9),15))
        self.assertIsNone(complete([115,14,96,90,163,15],7.2))
        self.assertIsNone(seq.observe([115,14,96,90,163,15],False,12.2))
        step=seq.observe([115,14,96,90,163,15],False,12.4)
        self.assertEqual(step,('DEPART',(115.2,55.2,81.5,123.6),15))
        step=complete([115,55,82,90,124,15],12.4)
        self.assertEqual(step,('SAFE_Rescue',(115.2,110.5,121.2,99.7),15))
        self.assertIsNone(complete([115,110,121,90,100,15],15.5))
        self.assertTrue(seq.done)

    def test_release_opens_only_after_reaching_drop_pose(self):
        seq=PickupSequence(0,release=True)
        self.assertEqual(seq.next_step(0),('DEPART',(115.2,55.2,81.5,123.6),15))
        seq.observe([115,55,82,90,124,15],True,1)
        self.assertEqual(seq.observe([115,55,82,90,124,15],False,3.1),('PICK',(115.2,14.3,95.8,162.9),15))
        seq.observe([115,14,96,90,163,15],True,4)
        self.assertIsNone(seq.observe([115,14,96,90,163,15],False,6.2))
        self.assertIsNone(seq.observe([115,14,96,90,163,15],False,7.1))
        self.assertEqual(seq.observe([115,14,96,90,163,15],False,7.3),('Abrir pinza',(115.2,14.3,95.8,162.9),75))
        seq.observe([115,14,96,90,163,75],True,8.3)
        self.assertIsNone(seq.observe([115,14,96,90,163,75],False,10.4))
        self.assertEqual(seq.observe([115,14,96,90,163,75],False,12.5),('Approach',(111.7,111.1,160.6,125.1),75))
        seq.observe([112,111,161,90,125,75],True,13.5)
        self.assertIsNone(seq.observe([112,111,161,90,125,75],False,15.6))
        self.assertTrue(seq.done)

    def test_pose_change_during_wait_cancels(self):
        seq=PickupSequence(0)
        seq.next_step(0);seq.observe([112,111,161,90,125,75],True,1)
        seq.observe([112,111,161,90,125,75],False,3.1)
        seq.observe([115,14,96,90,163,75],True,4)
        seq.observe([115,14,96,90,163,75],False,6.2)
        with self.assertRaisesRegex(RuntimeError,'espera'):
            seq.observe([115,14,96,90,163,70],False,6.3)

    def test_does_not_advance_without_busy_confirmation(self):
        seq=PickupSequence(0);seq.next_step(0)
        self.assertIsNone(seq.observe([112,111,161,90,125,75],False,3.1))
        with self.assertRaisesRegex(RuntimeError,'confirmación'):seq.observe([112,111,161,90,125,75],False,6.1)

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
        engine.robot.send.assert_called_once_with('ARM:115,55,82,90,124,15,3000\n')
        engine.handle('stop',False)
        self.assertIsNone(engine.pickup_sequence)
