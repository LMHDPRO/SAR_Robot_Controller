import time
import unittest
from concurrent.futures import Future

import numpy as np

from app import Engine


class RecordedRobot:
    def __init__(self):
        self.messages = []

    def send(self, message):
        self.messages.append(message)


class CocoOsTests(unittest.TestCase):
    def setUp(self):
        self.initialize_controller()

    def initialize_controller(self):
        self.engine = Engine()
        self.addCleanup(self.engine.pool.shutdown)
        self.robot = self.engine.robot = RecordedRobot()
        self.now = time.monotonic()
        self.engine.last_robot = self.engine.last_ui = self.now
        self.engine.telemetry = dict(
            protocol=1, armed=1, pwm_ok=1, arm_protocol=1, pca_ready=1,
            arm_angles=[108, 150, 180, 90, 90, 60], imu=1, imu_read_ok=1,
            imu_age_ms=0, heading=359,
        )
        self.engine.odom.imu_valid = True
        self.packet = dict(
            type='coco', coco_os=11, control_style='manual90', boot_id=123, seq=0,
            mode='B', menu_open=0, deadman=0, motion_enabled=0, drive=0, steer=0,
            owner_pending=0, arm_mode='XY', encoder_mode='Z', arm_filtered_x=0,
            arm_filtered_y=0, agv_filtered_x=0, agv_filtered_y=0,
        )

    def send(self, **values):
        self.now += .05
        self.engine.last_ui = self.engine.last_robot = self.now
        self.packet.update(values)
        self.packet['seq'] += 1
        self.engine.process_pad(self.packet.copy(), self.now)

    def arm_deadman(self, **values):
        self.send(deadman=0, motion_enabled=0, **values)
        self.send(deadman=1, motion_enabled=1)

    def test_handoff_acknowledges_and_rearms_in_both_directions(self):
        e = self.engine
        e.live = (85, 85)
        self.send(menu_open=1, owner_pending=1, owner_target='COCO', owner_request_id=1)
        self.assertEqual(e.source, 'COCO')
        self.assertIsNone(e.live)
        self.assertIn('OWNER:123,1,1\n', e.coco_state_frame(self.now))
        self.send(menu_open=0, owner_pending=0, deadman=1, motion_enabled=1, drive=1)
        self.assertIsNone(e.live)
        self.arm_deadman(drive=0)
        self.send(drive=1)
        self.assertEqual(e.live, (85, 85))
        self.send(menu_open=1, owner_pending=1, owner_target='PC', owner_request_id=2,
                  deadman=0, motion_enabled=0, drive=0)
        self.assertEqual(e.source, 'PC')
        self.assertIsNone(e.live)
        self.assertIn('OWNER:123,2,0\n', e.coco_state_frame(self.now))
        self.send()
        self.assertEqual(self.robot.messages.count('STOP\n'), 3)

    def test_mode_d_stops_arm_and_never_routes_arm_input(self):
        e = self.engine
        e.source = 'COCO'
        self.arm_deadman(mode='A', arm_joint_index=1)
        self.send(arm_filtered_y=100)
        self.assertTrue(self.robot.messages[-1].startswith('ARM:'))
        self.send(mode='D', drive=1)
        self.assertEqual(self.robot.messages[-1], 'STOP\n')
        self.assertIsNone(e.live)
        self.arm_deadman(drive=0, arm_filtered_y=0)
        self.robot.messages.clear()
        self.send(arm_filtered_y=100, drive=1, z_change_mm=50)
        self.assertEqual(e.live, (85, 85))
        self.assertEqual(self.robot.messages, [])
        self.send(boot_id=124)
        self.send()
        self.assertIsNone(e.live)

    def test_mode_d_requires_menu_to_turn_relays_on(self):
        self.engine.source = 'COCO'
        self.engine.telemetry['armed'] = 0
        self.send(mode='D', power_request_id=0)
        self.robot.messages.clear()
        self.send(power_request_id=1, power_pending=1, power_enable=1)
        self.assertNotIn('POWER:1\n', self.robot.messages)
        self.send(menu_open=1, deadman=1, power_request_id=2)
        self.assertNotIn('POWER:1\n', self.robot.messages)
        self.send(deadman=0, power_request_id=3)
        self.assertIn('POWER:1\n', self.robot.messages)
        pending_messages = list(self.robot.messages)
        self.send()
        self.assertEqual(self.robot.messages, pending_messages)
        self.engine.telemetry['armed'] = 1
        self.send(power_request_id=4, power_enable=0)
        self.assertEqual(self.robot.messages[-1], 'POWER:0\n')

    def test_motor_bars_receive_signed_logical_wheels(self):
        self.send(mode='D')
        e = self.engine
        e.g.swap_lr = 1
        e.g.motor_l = -1
        for logical in ((85, 40), (-85, -40), (-85, 40)):
            with self.subTest(logical=logical):
                e.telemetry['pl'], e.telemetry['pr'] = e.g.motor_output(*logical)
                self.assertIn(f'COCO11:1,{logical[0]},{logical[1]}\n', e.coco_state_frame(self.now))

    def test_mid_tcp_encoder_moves_with_one_detent(self):
        e = self.engine
        e.source = 'COCO'
        self.arm_deadman(mode='C', arm_speed_level=1)
        z_start = e.arm_model.compute_fk(e.arm_target_angles)[0][2]
        for _ in range(6):
            self.send(z_change_mm=-10)
        z_end = e.arm_model.compute_fk(e.arm_target_angles)[0][2]
        self.assertGreater(z_start - z_end, 4)
        self.assertLess(z_start - z_end, 6)

    def test_cartesian_moves_use_the_method_selected_on_coco(self):
        for mode in ('B', 'C'):
            for index, method in enumerate(self.engine.arm_model.IK_METHODS):
                with self.subTest(mode=mode, method=method):
                    self.initialize_controller()
                    e = self.engine
                    e.source = 'COCO'
                    self.arm_deadman(mode=mode, arm_ik_method=index, arm_speed_level=1)
                    self.assertIn('ARMOPT:0\n', e.coco_state_frame(self.now))
                    for _ in range(6):
                        self.send(arm_filtered_x=35)
                    self.assertEqual(e.arm_model.ik_method, method)
                    self.assertTrue(any(message.startswith('ARM:') for message in self.robot.messages))

    def test_coco_quintic_requires_agv_support_and_paces_arm_commands(self):
        for mode in ('A', 'B', 'C'):
            with self.subTest(mode=mode):
                self.initialize_controller()
                e = self.engine
                e.source = 'COCO'
                self.arm_deadman(mode=mode, arm_mode='RX_RY', arm_joint_index=3,
                                 arm_speed_level=2, arm_ik_method=0, arm_quintic=1)
                self.robot.messages.clear()
                self.send(arm_filtered_y=100)
                self.assertEqual(self.robot.messages, [])
                self.assertIn('firmware SAR_AGV', e.arm_error)

                e.telemetry['arm_quintic'] = 1
                self.assertIn('ARMOPT:1\n', e.coco_state_frame(self.now))
                self.send()
                self.assertEqual(len(self.robot.messages), 1)
                self.assertTrue(self.robot.messages[0].startswith('ARMQ:'))
                self.send()
                self.send()
                self.assertEqual(len(self.robot.messages), 1)
                self.now += .001
                self.send()
                self.assertEqual(len(self.robot.messages), 2)
                self.assertTrue(self.robot.messages[-1].startswith('ARMQ:'))
                self.send(deadman=0, motion_enabled=0, arm_filtered_y=0)
                self.assertEqual(self.robot.messages[-1], 'STOP\n')

    def test_arm_settings_change_requires_a_new_deadman_edge(self):
        self.engine.source = 'COCO'
        self.arm_deadman(mode='A', arm_joint_index=3)
        self.robot.messages.clear()
        self.send(arm_quintic=1, arm_filtered_y=100)
        self.send()
        self.assertEqual(self.robot.messages, ['STOP\n'])

    def test_gripper_uses_the_minimum_supported_by_the_agv(self):
        for mode in ('A', 'C'):
            with self.subTest(mode=mode):
                self.initialize_controller()
                e = self.engine
                e.source = 'COCO'
                self.arm_deadman(mode=mode, arm_joint_index=5, encoder_mode='GRIP',
                                 arm_speed_level=2)
                e.arm_target_gripper = 10
                for _ in range(3):
                    self.send(arm_filtered_y=-100 if mode=='A' else 0, grip_change_pct=-20)
                self.assertEqual(e.arm_target_gripper, 10)

                e.telemetry['arm_gripper_min'] = 5
                for _ in range(5):
                    self.send(grip_change_pct=-40)
                self.assertEqual(e.arm_target_gripper, 5)
                self.assertEqual(self.robot.messages[-1].split(',')[-2], '5')

    def test_pad_reconnect_requires_new_deadman_edge_even_with_same_boot(self):
        e = self.engine
        e.source = 'COCO'
        self.arm_deadman(mode='D')
        self.send(drive=1)
        self.assertIsNotNone(e.live)
        e.halt_loss()
        future = Future()
        future.set_result(object())
        e.pending_links['pad'] = (e.link_generation['pad'], future)
        e.poll_links()
        self.send()
        self.assertIsNone(e.live)
        self.arm_deadman(drive=0)
        self.send(drive=1)
        self.assertIsNotNone(e.live)

    def test_drive_and_arm_profiles_are_independent(self):
        e = self.engine
        e.source = 'COCO'
        self.arm_deadman(arm_mode='RX_RY', arm_speed_level=0, agv_speed_level=2)
        start = e.arm_target_angles[3]
        for _ in range(8):
            self.send(arm_filtered_y=100, drive=1)
        slow = start - e.arm_target_angles[3]
        self.assertGreater(slow, 0)
        self.assertEqual(e.live, (135, 135))
        self.arm_deadman(arm_filtered_y=0, drive=0, arm_speed_level=2, agv_speed_level=0)
        start = e.arm_target_angles[3]
        for _ in range(8):
            self.send(arm_filtered_y=100, drive=1)
        self.assertGreater(start - e.arm_target_angles[3], slow * 2)
        self.assertEqual(e.live, (85, 85))

    def test_low_moves_small_inputs_in_joints_and_tcp(self):
        for mode, axis in (('A', {'arm_filtered_y': 20}), ('C', {'arm_filtered_x': 35})):
            with self.subTest(mode=mode):
                deltas = []
                for level in (0, 2):
                    self.initialize_controller()
                    e = self.engine
                    e.source = 'COCO'
                    self.arm_deadman(mode=mode, arm_joint_index=3, arm_speed_level=level)
                    for _ in range(15):
                        self.send(**axis)
                    if mode == 'A':
                        deltas.append(e.arm_target_rotation - e.arm_rotation)
                    else:
                        start = e.arm_model.compute_fk(e.arm_angles)[0]
                        end = e.arm_model.compute_fk(e.arm_target_angles)[0]
                        deltas.append(np.linalg.norm(end - start))
                self.assertGreater(deltas[0], 0)
                self.assertGreater(deltas[1], 2 * deltas[0])

    def test_fine_tcp_encoder_clicks_accumulate_at_low(self):
        for submode in ('XY', 'RX_RY'):
            with self.subTest(submode=submode):
                self.initialize_controller()
                e = self.engine
                e.source = 'COCO'
                self.arm_deadman(mode='C', arm_mode=submode, arm_speed_level=0)
                z_start = e.arm_model.compute_fk(e.arm_target_angles)[0][2]
                for click in range(1, 25):
                    self.send(z_change_mm=-.5*click)
                z_end = e.arm_model.compute_fk(e.arm_target_angles)[0][2]
                self.assertGreater(z_start - z_end, .5)
                self.assertLess(z_start - z_end, 4)

    def test_turn_across_heading_wrap_and_no_repeat_until_center(self):
        e = self.engine
        e.source = 'COCO'
        unwrap = dict(boot_id=1, tl=0, tr=0, ms=0, **e.telemetry)
        e.odom.update(unwrap, e.g)
        e.odom.update(dict(unwrap, heading=1, ms=50), e.g)
        self.assertEqual(e.odom.imu_yaw, 2 * e.g.imu_sign)
        self.arm_deadman(mode='D', turn_step_deg=45)
        self.send(steer=1, agv_filtered_x=100)
        turn = e.coco_turn
        self.assertEqual(turn.target, e.odom.imu_yaw + 45)
        left, right = e.coco_turn_output(self.now)
        self.assertLess(left, 0)
        self.assertGreater(right, 0)
        e.odom.imu_yaw = turn.target
        e.coco_turn_output(self.now)
        self.now += .4
        e.last_robot = self.now
        self.assertEqual(e.coco_turn_output(self.now), (0, 0))
        self.assertEqual(e.coco_turn_state, 'DONE')
        self.send()
        self.assertIs(e.coco_turn, turn)
        self.send(steer=0, agv_filtered_x=0)
        self.send(steer=-1, agv_filtered_x=-100)
        self.assertEqual(e.coco_turn.value, -45)
        left, right = e.coco_turn_output(self.now)
        self.assertGreater(left, 0)
        self.assertLess(right, 0)
        self.send(deadman=0, motion_enabled=0)
        self.assertEqual(e.coco_turn_state, 'CANCEL')
        self.assertEqual(self.robot.messages[-1], 'STOP\n')
        self.arm_deadman(steer=0, agv_filtered_x=0)
        self.send(steer=-1, agv_filtered_x=-100)
        self.assertEqual(e.coco_turn_output(self.now + .26), (0, 0))
        self.assertEqual(e.coco_turn_state, 'CANCEL')

    def test_tcp_state_has_fk_pose_and_marks_missing_telemetry(self):
        self.send(mode='C')
        e = self.engine
        frames = e.coco_state_frame(self.now).splitlines()
        state = next(line for line in frames if line.startswith('STATE:'))
        self.assertEqual(len(state.split(',')), 10)
        pose = next(line for line in frames if line.startswith('POSE:'))
        self.assertEqual(len(pose.split(',')), 21)
        stale = e.coco_state_frame(self.now + .51)
        self.assertIn('COCO11:0,', stale)
        self.assertNotIn('POSE:', stale)
