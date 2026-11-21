import unittest

from core import Odometry, SingleEncoderMove, HeadingHold


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

    def test_derivative_filters_rate_spike_and_integral_does_not_wind_up(self):
        move = SingleEncoderMove(20, 'left', self.odom, 0, cap=100, min_pwm=80)
        self.odom.imu_rate = 100
        move.output(self.odom, .05)
        self.assertAlmostEqual(move.filtered_rate, 25)
        self.odom.imu_rate = 0
        self.odom.imu_yaw = -90
        move.output(self.odom, .1)
        self.assertEqual(move.integral, 0)

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

    def test_held_drive_corrects_heading_in_both_directions_and_stops_on_loss(self):
        for command in (100,-100):
            self.odom.imu_yaw=0;self.odom.imu_valid=True
            hold=HeadingHold(self.odom,0)
            self.odom.imu_yaw=-5
            left,right=hold.output(self.odom,.1,command)
            self.assertGreater(right,left)
            self.assertEqual(left>0,command>0)
            self.assertEqual(right>0,command>0)
            self.odom.imu_valid=False
            with self.assertRaisesRegex(RuntimeError,'continuity lost'):hold.output(self.odom,.2,command)

    def test_routine_distance_uses_selected_encoder_and_imu(self):
        from app import Engine
        import time
        engine=Engine();self.addCleanup(engine.pool.shutdown)
        engine.robot=object();engine.last_robot=time.monotonic()
        engine.telemetry={'armed':1,'protocol':1};engine.odom.imu_valid=True
        engine.handle('routine',([['distance',20]],90,'right'))
        self.assertIsInstance(engine.primitive,SingleEncoderMove)
        self.assertEqual(engine.primitive.side,'right')

    def test_live_heading_resets_after_turn_or_direction_change(self):
        from app import Engine
        engine=Engine();self.addCleanup(engine.pool.shutdown)
        engine.odom.imu_valid=True;engine.odom.imu_yaw=10
        engine.set_live_drive((85,85),0)
        self.assertEqual(engine.live_heading.target,10)
        engine.odom.imu_yaw=13
        engine.set_live_drive((110,110),.1)
        self.assertEqual(engine.live_heading.target,10)
        engine.set_live_drive((-85,85),.2)
        self.assertIsNone(engine.live_heading)
        engine.set_live_drive((-85,-85),.3)
        self.assertEqual(engine.live_heading.target,13)
        engine.set_live_drive(None,.4)
        self.assertIsNone(engine.live_heading)
        engine.odom.imu_valid=False
        with self.assertRaisesRegex(ValueError,'heading'):engine.set_live_drive((85,85),.5)


if __name__ == '__main__':
    unittest.main()
