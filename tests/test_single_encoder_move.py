import unittest

from core import Odometry, SingleEncoderMove


class SingleEncoderMoveTests(unittest.TestCase):
    def setUp(self):
        self.odom = Odometry()
        self.odom.imu_valid = True

    def test_selected_encoder_controls_distance_and_ignores_other_wheel(self):
        move = SingleEncoderMove(20, 'left', self.odom, 0, cap=100, min_pwm=80)
        self.odom.left = 197
        self.assertEqual(move.output(self.odom, 0.1), (0, 0))
        self.assertFalse(move.done)
        self.assertEqual(move.distance_cm(self.odom), 19.7)
        self.assertEqual(self.odom.right, 0)
        move.output(self.odom, 0.46)
        self.assertTrue(move.done)

    def test_heading_error_corrects_toward_starting_heading(self):
        move = SingleEncoderMove(20, 'right', self.odom, 0, cap=100, min_pwm=80)
        self.odom.imu_yaw = -5
        left, right = move.output(self.odom, 0.1)
        self.assertGreater(left, 0)
        self.assertGreater(right, left)
        self.assertLessEqual(right, move.cap)

    def test_reverse_move_keeps_both_wheels_reversing_while_correcting_heading(self):
        move = SingleEncoderMove(-20, 'left', self.odom, 0, cap=100, min_pwm=80)
        self.odom.imu_yaw = 5
        left, right = move.output(self.odom, 0.1)
        self.assertLess(left, 0)
        self.assertLess(right, 0)

    def test_stops_if_selected_encoder_stalls(self):
        move = SingleEncoderMove(20, 'left', self.odom, 0)
        self.odom.right = 100
        with self.assertRaisesRegex(RuntimeError, 'Left encoder: no progress'):
            move.output(self.odom, 1.6)

    def test_stops_if_imu_continuity_is_lost(self):
        move = SingleEncoderMove(20, 'left', self.odom, 0)
        self.odom.imu_valid = False
        with self.assertRaisesRegex(RuntimeError, 'IMU continuity lost'):
            move.output(self.odom, 0.1)

    def test_requires_fresh_imu_and_valid_encoder_choice(self):
        self.odom.imu_valid = False
        with self.assertRaisesRegex(ValueError, 'BNO055'):
            SingleEncoderMove(20, 'left', self.odom, 0)
        self.odom.imu_valid = True
        with self.assertRaisesRegex(ValueError, 'left or right'):
            SingleEncoderMove(20, 'unknown', self.odom, 0)


if __name__ == '__main__':
    unittest.main()
