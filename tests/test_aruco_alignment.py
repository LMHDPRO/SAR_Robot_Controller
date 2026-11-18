import unittest
import numpy as np
import time
from types import SimpleNamespace
from unittest.mock import Mock

from aruco_alignment import ArucoAlignment
from app import Engine


class AlignmentTests(unittest.TestCase):
    def setUp(self):
        self.ref={'xyz_camera_mm':[-71.9,35.2,187.6], 'rvec':[0.,0.,0.]}
        self.controller=ArucoAlignment(self.ref,10.,80)

    def sample(self, xyz=None, timestamp=10., rvec=None):
        return {'id':12,'captured_at':timestamp,'xyz_camera_mm':xyz or self.ref['xyz_camera_mm'], 'rvec':rvec or [0.,0.,0.]}

    def test_turns_towards_reference_using_measured_signs(self):
        self.assertEqual(self.controller.output(self.sample([-72.1,83.5,191.1]),10.),(80,-80))
        other=ArucoAlignment(self.ref,10.,80)
        self.assertEqual(other.output(self.sample([-72.3,20.,187.]),10.),(-80,80))

    def test_missing_stale_and_near_pose_stop(self):
        self.assertEqual(self.controller.output(None,10.),(0,0))
        self.assertEqual(self.controller.output(self.sample(timestamp=8.),10.),(0,0))
        with self.assertRaises(RuntimeError):self.controller.output(self.sample([-40.,20.,140.]),10.)

    def test_tolerates_brief_detection_loss_then_stops_on_prolonged_loss(self):
        self.assertEqual(self.controller.output(self.sample([-72.,53.,280.]),10.),(90,90))
        self.assertEqual(self.controller.output(None,10.31),(0,0))
        self.assertEqual(self.controller.output(None,10.7),(0,0))
        with self.assertRaisesRegex(RuntimeError,'perdido'):
            self.controller.output(None,40.1)

    def test_waits_for_marker_then_checks_resolution_before_moving(self):
        reference=dict(self.ref,image_size=[640,480])
        controller=ArucoAlignment(reference,10.,80)
        self.assertEqual(controller.output(None,12.),(0,0))
        sample=dict(self.sample([-72.,53.,280.],12.1),image_size=[320,240])
        with self.assertRaisesRegex(RuntimeError,'resolución'):controller.output(sample,12.1)
        sample['image_size']=[640,480]
        controller=ArucoAlignment(reference,10.,80)
        self.assertEqual(controller.output(sample,12.1),(90,90))

    def test_forward_pulses_and_settles_before_next_action(self):
        sample=self.sample([-72.,53.,280.])
        self.assertEqual(self.controller.output(sample,10.),(90,90))
        self.assertEqual(self.controller.output(self.sample([-72.,53.,280.],10.13),10.13),(90,90))
        self.assertEqual(self.controller.output(self.sample([-72.,53.,280.],10.31),10.31),(0,0))
        self.assertEqual(self.controller.output(self.sample([-72.,53.,280.],11.2),11.2),(0,0))
        self.assertEqual(self.controller.output(self.sample([-72.,53.,280.],11.6),11.6),(90,90))

    def test_target_is_180mm_and_near_pulses_are_shorter(self):
        far=self.sample([-72.,53.,280.])
        self.controller.output(far,10.)
        self.assertAlmostEqual(self.controller.until-10.,.30)
        near=ArucoAlignment(self.ref,10.,80)
        near.output(self.sample([-72.,40.,205.]),10.)
        self.assertAlmostEqual(near.until-10.,.10)
        arrival=ArucoAlignment(self.ref,10.,80)
        target=(np.array(self.ref['xyz_camera_mm'])*180/np.linalg.norm(self.ref['xyz_camera_mm'])).tolist()
        for now in (10.,10.1,10.2,10.3):
            arrival.output(self.sample(target,now),now)
        self.assertTrue(arrival.done)

    def test_forward_uses_odometry_limit_and_stops_on_imu_loss(self):
        odom=SimpleNamespace(left=0.,right=0.,imu_yaw=0.,imu_valid=True,imu_segment=1,revision=1)
        self.assertEqual(self.controller.output(self.sample([-72.,53.,350.]),10.,odom),(90,90))
        odom.left=odom.right=8.
        self.assertEqual(self.controller.output(self.sample([-72.,53.,345.],10.2),10.2,odom),(90,90))
        odom.left=odom.right=20.
        self.assertEqual(self.controller.output(self.sample([-72.,53.,340.],10.4),10.4,odom),(0,0))
        other=ArucoAlignment(self.ref,10.,80)
        other.output(self.sample([-72.,53.,350.]),10.,odom)
        odom.imu_valid=False
        self.assertEqual(other.output(self.sample([-72.,53.,345.],10.2),10.2,odom),(0,0))

    def test_completion_requires_fresh_samples_and_orientation(self):
        target=(np.array(self.ref['xyz_camera_mm'])*180/np.linalg.norm(self.ref['xyz_camera_mm'])).tolist()
        for now in (10.,10.1,10.2,10.3):self.controller.output(self.sample(target,timestamp=now),now)
        self.assertTrue(self.controller.done)
        controller=ArucoAlignment(self.ref,10.,80)
        for now in (10.,10.1,10.2):controller.output(self.sample(target,timestamp=now,rvec=[0.,0.,.5]),now)
        with self.assertRaisesRegex(RuntimeError,'orientación'):controller.output(self.sample(target,timestamp=10.3,rvec=[0.,0.,.5]),10.3)

    def test_inclined_marker_cannot_report_ready(self):
        target=(np.array(self.ref['xyz_camera_mm'])*180/np.linalg.norm(self.ref['xyz_camera_mm'])).tolist()
        controller=ArucoAlignment(self.ref,10.,80,require_orientation=False)
        for now in (10.,10.1,10.2):controller.output(self.sample(target,timestamp=now,rvec=[.4,0.,0.]),now)
        with self.assertRaisesRegex(RuntimeError,'inclinado'):controller.output(self.sample(target,timestamp=10.3,rvec=[.4,0.,0.]),10.3)
        self.assertFalse(controller.done)

    def test_engine_allows_any_arm_pose_but_blocks_motion_and_stop_cancels(self):
        engine=Engine();self.addCleanup(engine.pool.shutdown)
        engine.robot=Mock();engine.last_robot=time.monotonic()
        engine.telemetry={'protocol':1,'armed':1,'arm_protocol':1,'pca_ready':1,'arm_busy':0,'arm_angles':[90,126.5,74.2,90,106,69]}
        engine.aruco_sample=self.sample(timestamp=time.monotonic())
        engine.handle('aruco_start',self.ref)
        self.assertIsNotNone(engine.aruco_alignment)
        with self.assertRaisesRegex(ValueError,'STOP'):
            engine.send_arm_pose([90,126.5,74.2,106],69,500,'PC')
        engine.halt()
        self.assertIsNone(engine.aruco_alignment)
        engine.telemetry['arm_angles'][1]=90
        engine.handle('aruco_start',self.ref)
        self.assertIsNotNone(engine.aruco_alignment)
        engine.halt()
        engine.arm_busy=True
        with self.assertRaisesRegex(ValueError,'brazo'):engine.handle('aruco_start',self.ref)

    def test_sequence_waits_for_both_arm_stages_and_new_marker(self):
        engine=Engine();self.addCleanup(engine.pool.shutdown)
        engine.robot=Mock();engine.last_robot=time.monotonic()
        engine.telemetry={'protocol':1,'armed':1,'pwm_ok':1,'arm_protocol':1,'pca_ready':1,'estop':0}
        engine.handle('aruco_sequence',self.ref)
        start=engine.arm_approach['started']
        engine.arm_busy=True
        engine.update_arm_approach([90,126,74,90,106,75],True,start+1)
        engine.arm_busy=False
        engine.update_arm_approach([90,126,74,90,106,75],False,start+3.1)
        self.assertIsNone(engine.aruco_alignment)
        second=engine.collection_sequence['started']
        engine.arm_busy=True
        engine.update_collection_sequence([108,149,180,90,7,75],True,second+1)
        engine.arm_busy=False
        engine.update_collection_sequence([108,149,180,90,7,75],False,second+3.1)
        self.assertIsNotNone(engine.aruco_alignment)
        self.assertIsNone(engine.aruco_sample)
        self.assertEqual(engine.aruco_alignment.output(None,second+3.2),(0,0))
        engine.halt()
        self.assertIsNone(engine.aruco_alignment)


if __name__=='__main__': unittest.main()
