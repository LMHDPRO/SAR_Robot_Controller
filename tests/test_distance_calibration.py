import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from app import Engine


class DistanceCalibrationTests(unittest.TestCase):
    def ready(self):
        engine=Engine();engine.robot=object();engine.last_robot=time.monotonic()
        engine.telemetry=dict(boot_id=1,protocol=1,armed=1)
        engine.odom.imu_valid=True
        self.addCleanup(engine.pool.shutdown)
        return engine

    def test_selected_encoder_saved_atomically_with_report(self):
        for side in ('left','right'):
            engine=self.ready();engine.handle('distance_begin',side)
            setattr(engine.odom,side,500)
            # The other encoder is deliberately wrong.
            setattr(engine.odom,'right' if side=='left' else 'left',-12345)
            with tempfile.TemporaryDirectory() as folder:
                with patch('app.__file__',str(Path(folder)/'app.py')):
                    engine.handle('distance_apply',45)
                saved=json.loads((Path(folder)/'settings.json').read_text())
                self.assertAlmostEqual(saved['scale_l' if side=='left' else 'scale_r'],.9)
                self.assertEqual(saved['scale_r' if side=='left' else 'scale_l'],1)
                self.assertEqual(saved['_distance_calibration']['measured_cm'],45)
                self.assertEqual(saved['_distance_calibration']['side'],side)
                self.assertFalse((Path(folder)/'settings.tmp').exists())

    def test_reject_short_run_imu_gap_and_heading_change(self):
        engine=self.ready();engine.handle('distance_begin','left')
        engine.odom.left=100
        with self.assertRaises(ValueError):engine.handle('distance_apply',10)
        engine.odom.left=500;engine.odom.imu_segment+=1
        with self.assertRaises(ValueError):engine.handle('distance_apply',50)
        engine.odom.imu_segment-=1;engine.odom.imu_yaw=10
        with self.assertRaises(ValueError):engine.handle('distance_apply',50)

    def test_save_failure_keeps_original_scale_and_measurement(self):
        engine=self.ready();engine.handle('distance_begin','right');engine.odom.right=500
        with patch.object(engine,'save_settings',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):engine.handle('distance_apply',45)
        self.assertEqual(engine.g.scale_r,1)
        self.assertIsNotNone(engine.distance_reference)

    def test_start_button_requests_50_cm_on_selected_encoder(self):
        engine=self.ready()
        with patch.object(engine,'start_test') as start:
            engine.handle('distance_test',('right',85))
        start.assert_called_once_with('distance_imu',50,85,'right')
