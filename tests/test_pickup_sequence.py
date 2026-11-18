import unittest
import time
from unittest.mock import Mock
from pickup_sequence import PickupSequence
from app import Engine


class PickupTests(unittest.TestCase):
    def test_pickup_steps_wait_and_keep_gripper_closed_on_return(self):
        seq=PickupSequence([108.5,149.4,180,6.8],0)
        self.assertEqual(seq.next_step(0)[0],'Approach')
        def complete(channels,start):
            self.assertIsNone(seq.observe(channels,True,start+1))
            return seq.observe(channels,False,start+3.1)
        step=complete([111,112,160,90,106,75],0)
        self.assertEqual(step[0],'Pick')
        self.assertIsNone(complete([111,71,143,90,85,75],3.1))
        self.assertIsNone(seq.observe([111,71,143,90,85,75],False,7.0))
        step=seq.observe([111,71,143,90,85,75],False,7.2)
        self.assertEqual((step[0],step[2]),('Cerrar pinza',10))
        self.assertIsNone(complete([111,71,143,90,85,10],7.2))
        self.assertIsNone(seq.observe([111,71,143,90,85,10],False,12.2))
        step=seq.observe([111,71,143,90,85,10],False,12.4)
        self.assertEqual((step[0],step[2]),('Volver a Approach',10))
        step=complete([111,112,160,90,106,10],12.4)
        self.assertEqual((step[0],step[2]),('Volver a posición segura',10))
        self.assertIsNone(complete([108,149,180,90,7,10],15.5))
        self.assertTrue(seq.done)

    def test_release_opens_only_after_reaching_drop_pose(self):
        seq=PickupSequence([108.5,149.4,180,6.8],0,release=True)
        self.assertEqual(seq.next_step(0)[2],10)
        seq.observe([111,112,160,90,106,10],True,1)
        self.assertEqual(seq.observe([111,112,160,90,106,10],False,3.1)[2],10)
        seq.observe([111,71,143,90,85,10],True,4)
        self.assertIsNone(seq.observe([111,71,143,90,85,10],False,6.2))
        self.assertIsNone(seq.observe([111,71,143,90,85,10],False,7.1))
        self.assertEqual(seq.observe([111,71,143,90,85,10],False,7.3)[2],75)

    def test_pose_change_during_wait_cancels(self):
        seq=PickupSequence([108.5,149.4,180,6.8],0)
        seq.next_step(0);seq.observe([111,112,160,90,106,75],True,1)
        seq.observe([111,112,160,90,106,75],False,3.1)
        seq.observe([111,71,143,90,85,75],True,4)
        seq.observe([111,71,143,90,85,75],False,6.2)
        with self.assertRaisesRegex(RuntimeError,'espera'):
            seq.observe([111,71,143,90,85,70],False,6.3)

    def test_does_not_advance_without_busy_confirmation(self):
        seq=PickupSequence([108.5,149.4,180,6.8],0);seq.next_step(0)
        self.assertIsNone(seq.observe([111,112,160,90,106,75],False,3.1))
        with self.assertRaisesRegex(RuntimeError,'confirmación'):seq.observe([111,112,160,90,106,75],False,6.1)

    def test_release_requires_safe_estimate_and_stop_cancels(self):
        engine=Engine();self.addCleanup(engine.pool.shutdown)
        engine.robot=Mock();engine.last_robot=time.monotonic()
        engine.telemetry={'protocol':1,'armed':1,'pwm_ok':1,'pca_ready':1,'arm_protocol':1,'estop':0,
                          'arm_angles':[108,149,180,90,7,75]}
        with self.assertRaisesRegex(ValueError,'pinza a 10'):engine.handle('release_start',None)
        engine.telemetry['arm_angles'][5]=10
        engine.handle('release_start',None)
        self.assertTrue(engine.pickup_sequence.release)
        engine.handle('stop',False)
        self.assertIsNone(engine.pickup_sequence)
