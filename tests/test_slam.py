import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import math
import tempfile
import time
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from slam import LocalSLAM, transform, relative_prediction


def room():
    # Asymmetric room with an internal feature; enough geometry for all 3 DOF.
    x = np.linspace(-2, 3, 160)
    y = np.linspace(-1.5, 2, 120)
    return np.vstack((np.column_stack((x, np.full_like(x, -1.5))),
                      np.column_stack((x, np.full_like(x, 2))),
                      np.column_stack((np.full_like(y, -2), y)),
                      np.column_stack((np.full_like(y, 3), y)),
                      np.column_stack((np.linspace(.5, 1.3, 60), np.linspace(.1, .9, 60)))))


class SLAMTests(unittest.TestCase):
    def test_recovers_translation_and_rotation(self):
        model = LocalSLAM(); world = room()
        self.assertTrue(model.update(world, 1)['accepted'])
        pose = np.array([.09, -.04, math.radians(2)])
        sensor = transform(world-pose[:2], (0, 0, -pose[2]))
        result = model.update(sensor, 1.1)
        self.assertTrue(result['accepted'], result['note'])
        np.testing.assert_allclose(result['pose'], pose, atol=.025)

    def test_rejects_outliers_and_gap_without_changing_map(self):
        model = LocalSLAM(); model.update(room(), 1)
        before = model.map.copy()
        self.assertFalse(model.update(room()+100, 1.1)['accepted'])
        self.assertFalse(model.update(room(), 3)['accepted'])
        np.testing.assert_array_equal(before, model.map)
        self.assertEqual(len(model.path), 1)

    def test_rejects_degenerate_wall(self):
        model = LocalSLAM()
        points = np.column_stack((np.linspace(-2, 2, 100), np.ones(100)))
        model.update(points, 1)
        self.assertFalse(model.update(points, 1.1)['accepted'])

    def test_odometry_delta_rotates_into_map_frame(self):
        pose = np.array([2., 3., math.pi/2])
        predicted = relative_prediction(pose, np.zeros(3), np.array([1., 0., .1]))
        np.testing.assert_allclose(predicted, [2, 4, math.pi/2+.1])


class WorkspaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_mount_and_pose_reset_without_hardware(self):
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace, 'load'):
            panel = LidarWorkspace()
        panel.timer.stop()
        panel.odom_mode.setCurrentIndex(2)
        panel.odom.setChecked(True)
        old = panel.generation
        panel.receive_state(dict(pose_revision=1, fresh=True, x=1000, y=2000, yaw=90))
        self.assertGreater(panel.generation, old)
        np.testing.assert_allclose(panel.history[-1][1], [1, 2, math.pi/2])
        panel.model.update(room(), 1)
        panel.receive_state(dict(pose_revision=2, fresh=False))
        self.assertEqual(len(panel.model.map), 0)
        panel.source.setCurrentIndex(1); panel.reset_map(save=False)
        self.assertFalse(panel.use_odom)
        panel.shutdown()

    def test_acquisition_to_map_and_export(self):
        from lidar_workspace import LidarWorkspace
        from coind6.acquisition import Acquisition, DemoSource
        with patch.object(LidarWorkspace, 'load'):
            panel = LidarWorkspace()
        panel.timer.stop(); panel.live = False
        panel.reader = Acquisition(DemoSource()); panel.reader.start()
        try:
            deadline = time.monotonic()+4
            while panel.result is None and time.monotonic() < deadline:
                panel.poll(); self.app.processEvents(); time.sleep(.02)
            self.assertIsNotNone(panel.result)
            self.assertTrue(panel.result['accepted'])
            with tempfile.TemporaryDirectory() as tmp:
                destination = str(Path(tmp)/'map.npz')
                with patch('lidar_workspace.QtWidgets.QFileDialog.getSaveFileName', return_value=(destination, '')):
                    panel.export()
                with np.load(destination) as saved:
                    self.assertEqual(saved['map_xy_m'].shape[1], 2)
                    np.testing.assert_allclose(saved['mount_xy_yaw_deg'], [.115, 0, 0])
                    self.assertEqual(str(saved['source']), 'replay')
                    self.assertIn('created_utc',str(saved['metadata_json']))
                self.assertTrue(Path(destination).with_suffix('.png').is_file())
        finally:
            panel.shutdown()
        self.assertFalse(panel.reader.is_alive())

    def test_finish_waits_for_pending_scan_and_keeps_map_on_reset(self):
        from concurrent.futures import Future
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace,'load'):
            panel=LidarWorkspace()
        panel.timer.stop();panel.odom.setChecked(True)
        future=Future();panel.future=future;panel.job_generation=panel.generation
        result=panel.model.update(room(),1)
        with patch.object(panel,'export') as export:
            panel.finish_scan();panel.poll();export.assert_not_called()
            future.set_result(result);panel.poll();export.assert_called_once()
        self.assertIs(panel.result,result)
        panel.receive_state(dict(pose_revision=999,fresh=False))
        self.assertIs(panel.result,result)
        self.assertTrue(panel.finished)
        panel.shutdown()

    def test_single_encoder_requires_imu_and_resets_on_discontinuity(self):
        from lidar_workspace import LidarWorkspace
        with patch.object(LidarWorkspace, 'load'):
            panel = LidarWorkspace()
        panel.timer.stop(); panel.odom.setChecked(True)
        state=dict(pose_revision=1, imu_segment=0, fresh=True, imu_valid=True,
                   single_left=(1000,2000,.3), single_right=(3000,4000,.4))
        panel.receive_state(state)
        np.testing.assert_allclose(panel.history[-1][1], [1,2,.3])
        panel.model.update(room(),1)
        panel.receive_state(dict(state,imu_segment=1,imu_valid=False))
        self.assertFalse(panel.history)
        self.assertEqual(len(panel.model.map),0)
        panel.odom_mode.setCurrentIndex(1)
        panel.receive_state(dict(state,imu_segment=1))
        np.testing.assert_allclose(panel.history[-1][1], [3,4,.4])
        self.assertFalse(hasattr(panel,'serial'))
        self.assertEqual(panel.source.count(),2)
        panel.shutdown()


if __name__ == '__main__':
    unittest.main()
