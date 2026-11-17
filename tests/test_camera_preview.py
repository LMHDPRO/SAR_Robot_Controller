import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
import cv2
import numpy as np
from PySide6 import QtWidgets
from app import Engine
from arm_workspace import ArmWorkspace
from camera_worker import rotate_preview, run as run_camera_worker
from robotarm_v7 import VisionThread


APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class CameraPreviewTests(unittest.TestCase):
    def test_preview_rotation_supports_both_ninety_degree_directions(self):
        frame=np.arange(6,dtype=np.uint8).reshape(2,3)

        clockwise=rotate_preview(frame,90)
        counterclockwise=rotate_preview(frame,-90)

        np.testing.assert_array_equal(clockwise,np.array([[3,0],[4,1],[5,2]],dtype=np.uint8))
        np.testing.assert_array_equal(counterclockwise,np.array([[2,5],[1,4],[0,3]],dtype=np.uint8))

    def test_camera_worker_retries_a_lost_stream(self):
        first=Mock()
        first.isOpened.return_value=False
        second=Mock()
        second.isOpened.return_value=True
        second.read.return_value=(False,None)
        class StopRetry(Exception):pass
        config={'url':'http://camera/stream','dictionary':'DICT_4X4_50','marker_mm':35}

        with patch('camera_worker.cv2.VideoCapture',side_effect=[first,second]) as capture, \
             patch('camera_worker.time.sleep',side_effect=[None,StopRetry]), \
             patch('camera_worker.emit') as emit:
            with self.assertRaises(StopRetry):
                run_camera_worker(config)

        self.assertEqual(capture.call_count,2)
        self.assertTrue(any('Reintento' in call.args[0].get('status','') for call in emit.call_args_list))

    def arm(self):
        engine=Engine()
        with patch.object(ArmWorkspace,'_load_vision_config'):
            arm=ArmWorkspace(engine)
        arm.timer.stop()
        self.addCleanup(engine.pool.shutdown)
        self.addCleanup(arm.deleteLater)
        return arm

    def test_preview_keeps_aspect_ratio_without_detection_pipeline(self):
        worker=VisionThread('http://camera',output_size=(320,320))
        frames=[];worker.frame_ready.connect(frames.append)
        worker._emit_processed_frame(cv2,np.zeros((480,640,3),np.uint8),time.time()-.1,0,0)
        self.assertEqual((frames[0].width(),frames[0].height()),(320,240))
        self.assertFalse(hasattr(worker,'detections_ready'))

    def test_camera_connect_has_no_motion_controls_or_commands(self):
        arm=self.arm()
        self.assertFalse(hasattr(arm,'btn_vision'))
        self.assertFalse(hasattr(arm,'chk_vision_physical'))
        self.assertFalse(hasattr(arm,'_toggle_vision'))
        arm.camera_source.setText('192.168.3.17')
        with patch.object(arm,'_save_vision_config'),patch.object(VisionThread,'start'):
            arm._start_camera_stream()
        self.assertTrue(arm.engine.commands.empty())
        self.assertTrue(arm.engine.urgent_commands.empty())
        self.assertEqual(arm.vision_thread.stream_url,'192.168.3.17')
        self.assertTrue(arm.vision_thread.arm_camera_api)
        arm._stop_camera_stream()

    def test_arm_camera_address_is_prefilled_and_customizable(self):
        arm=self.arm()
        self.assertEqual(arm.camera_source.text(),arm.engine.runtime['arm_camera_url'])
        custom_url='192.168.3.28'
        arm.camera_source.setText(custom_url)
        with patch.object(arm.engine,'post') as post:
            arm.save_camera_url()
        post.assert_called_once_with('arm_camera_url',custom_url)

    def test_manual_path_planning_survives_removed_search_controls(self):
        arm=self.arm()
        target=np.array(arm.current_xyz)
        with patch.object(arm,'_trajectory_point_collision_reason',return_value=''),patch.object(arm,'_find_safe_path_via_points',return_value=([target],'directa','')):
            arm._execute_path_planning()
        self.assertTrue(arm.path_active)
        arm._control_loop_tick_guarded()
        self.assertFalse(arm.path_active)
        self.assertTrue(arm.engine.commands.empty())

    def test_camera_settings_drop_obsolete_detector_options(self):
        arm=self.arm()
        with tempfile.TemporaryDirectory() as folder:
            arm.vision_config_path=Path(folder)/'camera.json'
            arm.vision_config_path.write_text(json.dumps({'camera':{'resolution':'640x480','mirror_horizontal':True,'model_path':'obsolete.pt','max_objects':4},'drop_zone':{'x_mm':20}}))
            arm._load_vision_config(silent=True,create_if_missing=True)
            saved=json.loads(arm.vision_config_path.read_text())
        self.assertEqual(saved,{'version':11,'camera':{'resolution':'640x480','mirror_horizontal':True}})
