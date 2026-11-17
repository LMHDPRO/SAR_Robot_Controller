import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from PySide6 import QtCore, QtWidgets
from app import Engine, Window
from arm import ArmKinematics
from arm_workspace import ArmWorkspace
from camera_worker import detect,load_calibration
from robotarm_v7 import RobotKinematics

APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class IntegrationTests(unittest.TestCase):
    def test_v7_and_sar_agree_on_fk(self):
        a,b=ArmKinematics(),RobotKinematics()
        for angles in ([90,126.5,74.2,106],[45,90,90,90],[130,80,110,65]):
            np.testing.assert_allclose(a.compute_fk(angles)[0],b.compute_fk(angles)[0])

    def test_gamepad_safe_target_matches_v7_preview_ik(self):
        sar, preview = ArmKinematics(), RobotKinematics()
        start = sar.startup_angles.copy()
        target = sar.enforce_safety_zone(sar.compute_fk(start)[0] + [2.5, 0.0, 0.0])
        sar.last_ik_theta = sar.convert_angles(start)
        preview.last_ik_theta = preview.convert_angles(start)

        sar_ok, sar_angles, _ = sar.compute_ik(target)
        preview_ok, preview_angles, _ = preview.compute_ik(target)

        self.assertTrue(sar_ok and preview_ok)
        np.testing.assert_allclose(sar_angles, preview_angles, atol=1e-6)

    def test_stop_clears_every_physical_source_and_recovery(self):
        with patch.object(ArmWorkspace,'_load_vision_config'):
            arm=ArmWorkspace(Engine())
        arm.timer.stop()
        arm.path_active=True;arm.planned_path=[np.ones(3)]
        arm.vision_state='SCAN';arm.vision_motion_queue=[1];arm.vision_actions=[1]
        arm.recovery_active=True;arm._force_hardware_send=True
        arm.stop_robot()
        self.assertFalse(arm.path_active or arm.recovery_active or arm._force_hardware_send)
        self.assertEqual(arm.vision_state,'IDLE')
        self.assertEqual(arm.vision_motion_queue,[])
        self.assertEqual(arm.engine.urgent_commands.get_nowait()[0],'stop')
        arm.deleteLater()

    def test_offline_slider_preview_moves_and_stop_resyncs_sliders(self):
        with patch.object(ArmWorkspace,'_load_vision_config'):
            arm=ArmWorkspace(Engine())
        arm.timer.stop()
        arm.fk_sliders[1].setValue(950)
        for _ in range(12):arm._control_loop_tick()
        self.assertAlmostEqual(arm.current_physical_angles[1],95)
        arm.cancel_motion()
        self.assertEqual(arm.fk_sliders[1].value(),950)
        self.assertTrue(arm.engine.commands.empty())
        arm.deleteLater()

    def test_disconnected_live_cannot_be_enabled(self):
        with patch.object(ArmWorkspace,'_load_vision_config'):
            arm=ArmWorkspace(Engine())
        arm.timer.stop();arm.chk_live.setChecked(True)
        self.assertFalse(arm.chk_live.isChecked())
        arm._send_to_arduino()
        self.assertTrue(arm.engine.commands.empty())
        arm.deleteLater()

    def test_arm_workspace_hides_agv_and_preserves_global_controls(self):
        with patch.object(Engine,'start'),patch.object(Engine,'join'),patch.object(Engine,'save_settings'),patch.object(ArmWorkspace,'_load_vision_config'),patch.object(ArmWorkspace,'_save_vision_config'):
            w=Window();w.show();APP.processEvents()
            w.workspaces.setCurrentIndex(1);APP.processEvents()
            self.assertFalse(w.plot.isVisible())
            self.assertTrue(w.arm_workspace.canvas.isVisible())
            self.assertTrue(w.arm_live_checkbox.isVisible())
            self.assertTrue(w.arm_send_button.isVisible())
            self.assertEqual(w.engine.urgent_commands.get_nowait()[0],'stop')
            blocker=QtCore.QSignalBlocker(w.arm_live_checkbox)
            w.arm_live_checkbox.setChecked(True)
            w.arm_live_changed(True)
            w.workspaces.setCurrentIndex(2);APP.processEvents()
            self.assertTrue(w.arm_live_checkbox.isChecked())
            self.assertTrue(w.arm_workspace.timer.isActive())
            self.assertFalse(w.arm_send_button.isEnabled())
            self.assertFalse(w.arm_send_button.isVisible())
            self.assertTrue(w.engine.urgent_commands.empty())
            del blocker
            self.assertTrue(w.ip.isVisible() and w.status.isVisible())
            self.assertLessEqual(w.height(),860)
            w.cameras.save=lambda:None
            w.close();APP.processEvents()

    def test_aruco_without_calibration_never_invents_distance(self):
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        frame=np.full((480,640),255,np.uint8)
        frame[140:340,220:420]=cv2.aruco.generateImageMarker(dictionary,7,200)
        detector=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters())
        rows=detect(cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR),detector,35,None)
        self.assertEqual(rows[0]['id'],7)
        self.assertNotIn('distance_mm',rows[0])
        k=np.array([[800,0,320],[0,800,240],[0,0,1]],dtype=float)
        metric=detect(cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR),detector,35,(k,np.zeros(5),np.array([640,480])))
        self.assertAlmostEqual(metric[0]['distance_mm'],800*35/199,delta=2)
        self.assertLess(metric[0]['reprojection_px'],1)

    def test_calibration_rejects_invalid_intrinsics(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'bad.npz'
            np.savez(p,camera_matrix=np.zeros((3,3)),dist_coeffs=np.zeros(5),image_size=[640,480])
            with self.assertRaises(ValueError):load_calibration(p)


if __name__=='__main__':unittest.main()
