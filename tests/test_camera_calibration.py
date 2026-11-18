import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
from PySide6 import QtWidgets

from camera_calibration import CalibrationSession
from camera_worker import load_calibration
from cameras import CameraPanel


class CameraCalibrationTests(unittest.TestCase):
    def test_board_capture_rejects_duplicate_and_missing_board(self):
        board = np.full((320, 400, 3), 255, np.uint8)
        for y in range(8):
            for x in range(10):
                if (x+y) % 2 == 0:
                    board[y*30+40:(y+1)*30+40, x*30+50:(x+1)*30+50] = 0
        session = CalibrationSession()
        self.assertIn('Vista 1', session.capture(board))
        self.assertIn('parecida', session.capture(board))
        self.assertIn('No se detectó', session.capture(np.zeros_like(board)))
        self.assertEqual(len(session.points), 1)

    def test_calibration_recovers_intrinsics_and_saves_loadable_file(self):
        session = CalibrationSession()
        matrix = np.array([[700., 0, 320], [0, 710., 240], [0, 0, 1.]])
        obj = np.zeros((63, 3), np.float32)
        obj[:, :2] = np.mgrid[0:9, 0:7].T.reshape(-1, 2)*25
        session.size = (640, 480)
        for i in range(15):
            r = np.array([.08*(i%5-2), .10*(i//5-1), .03*i])
            t = np.array([-100.+10*(i%3), -70.+8*(i%4), 650.+20*i])
            points, _ = cv2.projectPoints(obj, r, t, matrix, np.zeros(5))
            session.points.append(points.astype(np.float32))
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)/'calibration.npz'
            self.assertLess(session.save(target), .01)
            result, _, size = load_calibration(target)
            np.testing.assert_allclose(result, matrix, atol=1)
            np.testing.assert_array_equal(size, [640, 480])
        session.points = session.points[:11]
        with self.assertRaises(ValueError): session.save('unused.npz')

    def test_button_only_for_agv(self):
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        agv = CameraPanel('AGV', {})
        arm = CameraPanel('Arm', {}, arm_camera_api=True)
        try:
            self.assertTrue(any(b.text() == 'Calibrar cámara AGV' for b in agv.findChildren(QtWidgets.QPushButton)))
            self.assertFalse(any(b.text() == 'Calibrar cámara AGV' for b in arm.findChildren(QtWidgets.QPushButton)))
            requests=[]
            agv.approach_pose_requested.connect(lambda: requests.append(True))
            button=next(b for b in agv.findChildren(QtWidgets.QPushButton) if b.text() == 'Colocar brazo en postura de aproximación')
            button.click()
            self.assertEqual(requests,[True])
            self.assertFalse(any(b.text() == button.text() for b in arm.findChildren(QtWidgets.QPushButton)))
        finally:
            agv.shutdown(); arm.shutdown()


if __name__ == '__main__': unittest.main()
