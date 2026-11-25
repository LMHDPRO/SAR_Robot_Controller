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
from PySide6 import QtGui, QtWidgets
from app import Engine, Window
from arm_workspace import ArmWorkspace
from camera_worker import detect, rotate_preview, run as run_camera_worker
from cameras import CameraPanel
from robotarm_v7 import VisionThread


APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class CameraPreviewTests(unittest.TestCase):
    def test_routine_button_starts_at_the_selected_pick_step(self):
        with patch.object(Engine,'start'), patch.object(Engine,'join'), patch('cameras.CamerasPage.save'):
            window=Window()
            try:
                window.routine_steps=[('aruco',None),('pickup',None),('wait',1)]
                window.refresh_steps()
                window.routine_list.setCurrentRow(1)
                self.assertTrue(window.routine_start_step_button.isEnabled())
                with patch.object(window.engine,'post') as post:
                    window.routine_start_step_button.click()
                post.assert_called_once_with('routine',(window.routine_steps,window.cap.value(),'left',None,1))
            finally:
                window.close()
                window.engine.pool.shutdown()

    def test_routine_preview_reuses_camera_frame_and_clears_on_stop(self):
        with patch('cameras.CamerasPage.save'):
            window=Window()
            try:
                panel=window.cameras.panels[0]
                self.assertEqual(len(window.cameras.panels),1)
                self.assertGreaterEqual(window.workspaces.indexOf(window.routine_scroll),0)
                self.assertEqual(window.tabs.indexOf(window.routine_scroll),-1)
                panel.preview_frame.emit(QtGui.QImage(400,200,QtGui.QImage.Format.Format_RGB32))
                pixmap=window.routine_camera_preview.pixmap()
                self.assertFalse(pixmap.isNull())
                self.assertEqual((pixmap.width(),pixmap.height()),(200,100))
                panel.stop()
                self.assertEqual(window.routine_camera_preview.text(),'Cámara AGV sin imagen')
            finally:
                window.close()
                APP.processEvents()
    def test_reference_size_change_rescales_without_accepting_other_changes(self):
        saved={'url':'camera','dictionary':'DICT_6X6_250','marker_mm':35.,'calibration_sha256':'abc'}
        ref={'identity':saved,'xyz_camera_mm':[35.,70.,175.],'target_distance_mm':210.}
        current=dict(saved,marker_mm=30.)
        result,changed=CameraPanel.adapt_reference_size(ref,current)
        self.assertTrue(changed)
        np.testing.assert_allclose(result['xyz_camera_mm'],[30.,60.,150.])
        self.assertEqual(result['target_distance_mm'],180.)
        self.assertEqual(ref['identity']['marker_mm'],35.)
        with self.assertRaisesRegex(ValueError,'calibración'):
            CameraPanel.adapt_reference_size(ref,dict(current,calibration_sha256='different'))

    def test_reference_reports_measured_distance_and_saves_in_range(self):
        panel=CameraPanel('AGV',{})
        self.addCleanup(panel.shutdown)
        self.addCleanup(panel.deleteLater)
        with tempfile.TemporaryDirectory() as folder:
            panel.reference_path=Path(folder)/'reference.json'
            now=time.monotonic()
            panel.marker_readings=[{'captured_at':now,'xyz_camera_mm':[0,0,300],
                                    'rvec':[0,0,0],'image_size':[640,480]} for _ in range(5)]
            with patch.object(panel,'reference_identity',return_value={'camera':'test'}):
                panel.save_reference()
                self.assertIn('300 mm',panel.alignment_status.text())
                self.assertIn('Acerca',panel.alignment_status.text())
                self.assertFalse(panel.reference_path.exists())
                centered=[-61.7,-29.0,165.2]
                for sample in panel.marker_readings:sample['xyz_camera_mm']=centered
                panel.save_reference()
                saved=json.loads(panel.reference_path.read_text(encoding='utf8'))
                np.testing.assert_allclose(saved['xyz_camera_mm'],centered)
                self.assertAlmostEqual(saved['target_distance_mm'],178.7,places=1)

    def test_preview_rotation_supports_both_ninety_degree_directions(self):
        frame=np.arange(6,dtype=np.uint8).reshape(2,3)

        clockwise=rotate_preview(frame,90)
        counterclockwise=rotate_preview(frame,-90)

        np.testing.assert_array_equal(clockwise,np.array([[3,0],[4,1],[5,2]],dtype=np.uint8))
        np.testing.assert_array_equal(counterclockwise,np.array([[2,5],[1,4],[0,3]],dtype=np.uint8))

    def test_contrast_fallback_recovers_low_contrast_marker_with_valid_pose(self):
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters=cv2.aruco.DetectorParameters()
        parameters.minMarkerPerimeterRate=.015
        parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        detector=cv2.aruco.ArucoDetector(dictionary,parameters)
        marker=cv2.aruco.generateImageMarker(dictionary,12,48)
        frame=np.full((480,640,3),150,np.uint8)
        frame[188:236,284:332]=np.where(marker[...,None],150,137)
        calibration=(np.array([[600.,0.,320.],[0.,600.,240.],[0.,0.,1.]]),np.zeros(5),[640,480])
        contrast=cv2.createCLAHE(clipLimit=2.0,tileGridSize=(8,8))
        rows=detect(frame,detector,30,calibration,contrast)
        self.assertEqual([row['id'] for row in rows],[12])
        self.assertIn('distance_mm',rows[0])
        self.assertLessEqual(rows[0]['reprojection_px'],3.)

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
