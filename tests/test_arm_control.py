import time
import unittest
import numpy as np

from app import Engine


class RecordingLink:
    def __init__(self):
        self.messages = []

    def send(self, message):
        self.messages.append(message)

    def urgent(self, message):
        self.messages.append(message)


class ArmControlTests(unittest.TestCase):
    def ready_engine(self, owner="PC"):
        engine = Engine()
        engine.source = owner
        engine.robot = RecordingLink()
        engine.last_robot = time.monotonic()
        engine.telemetry = {
            "protocol": 1,
            "armed": 1,
            "pwm_ok": 1,
            "pca_ready": 1,
            "arm_protocol": 1,
            "estop": 0,
        }
        return engine

    def test_pc_pose_uses_the_existing_agv_link_and_fixed_wrist_rotation(self):
        engine = self.ready_engine()

        engine.handle("arm_pose", ([90.0, 126.5, 74.2, 106.0], 65.0, 2000))

        self.assertEqual(
            engine.robot.messages,
            ["ARM:90,126,74,90,106,65,2000\n"],
        )

    def test_home_resets_wrist_and_gripper_and_requires_pc_control(self):
        engine = self.ready_engine()
        engine.arm_target_rotation = 42
        engine.handle("arm_home", None)
        self.assertEqual(engine.robot.messages, ["ARM:108,150,180,90,90,60,2000\n"])
        engine.source = "COCO"
        with self.assertRaises(ValueError):
            engine.handle("arm_home", None)
        self.assertEqual(len(engine.robot.messages), 1)

    def test_approach_waits_for_first_stage_and_stop_cancels_it(self):
        engine = self.ready_engine()
        engine.handle('arm_approach', None)
        start = engine.arm_approach['started']
        self.assertEqual(engine.robot.messages, ['ARM:90,126,74,90,106,75,3000\n'])
        engine.update_arm_approach([90,126,74,90,106,75], False, start + 3.1)
        self.assertEqual(len(engine.robot.messages), 1)
        engine.update_arm_approach([90,126,74,90,106,75], True, start + 1)
        engine.update_arm_approach([90,126,74,90,106,75], False, start + 3.1)
        self.assertEqual(engine.robot.messages[-1], 'ARM:108,149,180,90,7,75,3000\n')
        engine.handle('arm_approach', None)
        engine.handle('stop', False)
        engine.update_arm_approach([90,126,74,90,106,75], False, start + 4)
        self.assertEqual(engine.robot.messages[-1], 'STOP\n')

    def test_pc_pose_is_rejected_when_pca9685_is_not_ready(self):
        engine = self.ready_engine()
        engine.telemetry["pca_ready"] = 0

        with self.assertRaisesRegex(ValueError, "PCA9685"):
            engine.handle("arm_pose", ([90, 126, 74, 106], 65, 2000))

        self.assertEqual(engine.robot.messages, [])

    def test_pc_pose_requires_arm_capable_agv_firmware(self):
        engine = self.ready_engine()
        engine.telemetry.pop("arm_protocol")

        with self.assertRaisesRegex(ValueError, "soporte del brazo"):
            engine.handle("arm_pose", ([90, 126, 74, 106], 65, 2000))

        self.assertEqual(engine.robot.messages, [])

    def test_stop_cancels_the_arm_target_over_the_existing_agv_link(self):
        engine = self.ready_engine()
        engine.handle("arm_pose", ([90, 126, 74, 106], 65, 2000))

        engine.handle("stop", False)

        self.assertEqual(engine.robot.messages[-1], "STOP\n")
        self.assertFalse(engine.arm_busy)
        self.assertEqual(engine.arm_target_gripper, engine.arm_gripper)

    def test_pc_cannot_send_an_arm_pose_when_coco_owns_control(self):
        engine = self.ready_engine(owner="COCO")

        with self.assertRaisesRegex(ValueError, "Seleccionar PC"):
            engine.handle("arm_pose", ([90, 126, 74, 106], 65, 2000))

        self.assertEqual(engine.robot.messages, [])

    def test_coco_arm_joystick_requires_deadman_and_uses_existing_agv_link(self):
        engine = self.ready_engine(owner="COCO")
        now = time.monotonic()
        engine.last_ui = now

        packet = {
            "type": "coco",
            "control_style": "manual90",
            "boot_id": 1,
            "seq": 1,
            "mode": "B",
            "deadman": 0,
            "motion_enabled": 0,
            "drive": 0,
            "steer": 0,
            "arm_mode": "RX_RY",
            "encoder_mode": "Z",
            "arm_filtered_x": 0,
            "arm_filtered_y": 0,
            "z_change_mm": 0,
            "grip_change_pct": 0,
        }
        engine.process_pad(dict(packet), now)
        self.assertEqual(engine.robot.messages[-1], "STOP\n")

        packet.update(seq=2, deadman=1, motion_enabled=1)
        engine.process_pad(dict(packet), now + 0.05)
        self.assertEqual(engine.arm_target_angles[3], 90.0)

        packet.update(seq=3, arm_filtered_y=100)
        engine.process_pad(dict(packet), now + 0.15)

        self.assertEqual(engine.robot.messages[-1], "ARM:108,150,180,90,88,60,150\n")

    def test_coco_joint_mode_controls_wrist_rotation_and_gripper(self):
        engine = self.ready_engine(owner="COCO")
        now = time.monotonic()
        engine.last_ui = now
        packet = {
            "type": "coco",
            "control_style": "manual90",
            "boot_id": 1,
            "seq": 1,
            "mode": "A",
            "deadman": 0,
            "motion_enabled": 0,
            "drive": 0,
            "steer": 0,
            "arm_mode": "XY",
            "encoder_mode": "Z",
            "arm_joint_index": 3,
            "arm_filtered_x": 0,
            "arm_filtered_y": 0,
        }
        engine.process_pad(dict(packet), now)
        packet.update(seq=2, deadman=1, motion_enabled=1)
        engine.process_pad(dict(packet), now + 0.05)
        packet.update(seq=3, arm_filtered_y=100)
        engine.process_pad(dict(packet), now + 0.15)

        self.assertAlmostEqual(engine.arm_target_rotation, 93.0)
        self.assertEqual(engine.robot.messages[-1], "ARM:108,150,180,93,90,60,150\n")

        packet.update(seq=4, arm_joint_index=5, arm_filtered_y=-100)
        engine.process_pad(dict(packet), now + 0.25)

        self.assertAlmostEqual(engine.arm_target_gripper, 57.0)
        self.assertEqual(engine.robot.messages[-1], "ARM:108,150,180,93,90,57,150\n")

    def test_coco_modes_b_and_c_follow_legacy_smooth_gamepad_motion(self):
        results = []
        for mode in ("B", "C"):
            engine = self.ready_engine(owner="COCO")
            start = engine.arm_model.startup_angles.copy()
            engine.arm_angles = start.copy()
            engine.arm_target_angles = start.copy()
            engine.arm_model.last_ik_theta = engine.arm_model.convert_angles(start)
            start_xyz = engine.arm_model.compute_fk(start)[0]
            now = time.monotonic()
            engine.last_ui = now
            packet = {
                "type": "coco",
                "control_style": "manual90",
                "boot_id": 1,
                "seq": 1,
                "mode": mode,
                "deadman": 0,
                "motion_enabled": 0,
                "drive": 0,
                "steer": 0,
                "arm_mode": "XY",
                "encoder_mode": "Z",
                "arm_filtered_x": 0,
                "arm_filtered_y": 0,
                "z_change_mm": 0,
                "grip_change_pct": 0,
            }
            engine.process_pad(dict(packet), now)
            packet.update(seq=2, deadman=1, motion_enabled=1)
            engine.process_pad(dict(packet), now + 0.05)
            packet.update(seq=3, arm_filtered_x=100)
            engine.process_pad(dict(packet), now + 0.10)

            actual_xyz = engine.arm_model.compute_fk(engine.arm_target_angles)[0]
            self.assertLessEqual(np.linalg.norm(actual_xyz-start_xyz),5.1)
            self.assertLess(abs(engine.arm_target_angles[0]-start[0]),2.0)
            results.append(engine.robot.messages[-1])

        self.assertEqual(results[0], results[1])

    def test_coco_z_encoder_moves_arm_in_rx_ry_mode(self):
        engine = self.ready_engine(owner="COCO")
        start_xyz = engine.arm_model.compute_fk(engine.arm_target_angles)[0]
        now = time.monotonic()
        engine.last_ui = now
        packet = {
            "type": "coco",
            "control_style": "manual90",
            "boot_id": 1,
            "seq": 1,
            "mode": "B",
            "deadman": 0,
            "motion_enabled": 0,
            "drive": 0,
            "steer": 0,
            "arm_mode": "RX_RY",
            "encoder_mode": "Z",
            "arm_filtered_x": 0,
            "arm_filtered_y": 0,
            "z_change_mm": 0,
            "grip_change_pct": 0,
        }
        engine.process_pad(dict(packet), now)
        packet.update(seq=2, deadman=1, motion_enabled=1)
        engine.process_pad(dict(packet), now + 0.05)
        packet.update(seq=3, z_change_mm=-20)
        engine.process_pad(dict(packet), now + 0.10)

        actual_xyz = engine.arm_model.compute_fk(engine.arm_target_angles)[0]
        self.assertLess(actual_xyz[2], start_xyz[2])
        self.assertLessEqual(np.linalg.norm(actual_xyz-start_xyz), 5.1)


if __name__ == "__main__":
    unittest.main()
