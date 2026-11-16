import unittest

import numpy as np

from arm import ArmKinematics


class ArmKinematicsTests(unittest.TestCase):
    def setUp(self):
        self.arm = ArmKinematics()

    def test_startup_pose_round_trips_through_fk_and_ik(self):
        position, _ = self.arm.compute_fk(self.arm.startup_angles)

        success, angles, error = self.arm.compute_ik(position)

        self.assertTrue(success)
        np.testing.assert_allclose(angles, self.arm.startup_angles, atol=0.1)
        self.assertAlmostEqual(error, 0.0, places=6)

    def test_ik_rejects_target_outside_safe_reach(self):
        previous_theta = self.arm.last_ik_theta.copy()

        success, angles, reason = self.arm.compute_ik(
            np.array([self.arm.max_reach, 0.0, 0.0])
        )

        self.assertFalse(success)
        self.assertIsNone(angles)
        self.assertEqual(reason, "WARN:SINGULARITY")
        np.testing.assert_array_equal(self.arm.last_ik_theta, previous_theta)

    def test_ik_solves_a_nearby_forward_kinematics_target(self):
        target_angles = self.arm.startup_angles + np.array([5.0, -4.0, 3.0, -2.0])
        target_position, _ = self.arm.compute_fk(target_angles)

        success, solved_angles, error = self.arm.compute_ik(target_position)

        self.assertTrue(success)
        self.assertTrue(np.all(solved_angles >= 0.0))
        self.assertTrue(np.all(solved_angles <= 180.0))
        self.assertLessEqual(error, 2.0)

    def test_safety_zone_pushes_axis_center_out_to_minimum_radius(self):
        safe_position = self.arm.enforce_safety_zone([0.0, 0.0, 120.0])

        self.assertAlmostEqual(np.hypot(*safe_position[:2]), 70.0)
        self.assertEqual(safe_position[2], 120.0)

    def test_safety_zone_keeps_high_targets_inside_safe_reach(self):
        safe_position = self.arm.enforce_safety_zone([0.0, 13.916, 490.928])

        self.assertAlmostEqual(np.hypot(*safe_position[:2]), 70.0)
        self.assertLessEqual(np.linalg.norm(safe_position), self.arm.max_reach * 0.98)

    def test_safety_zone_projects_startup_pose_to_a_reachable_ik_target(self):
        startup_position = self.arm.compute_fk(self.arm.startup_angles)[0]
        target = self.arm.enforce_safety_zone(startup_position + [2.5, 0.0, 0.0])

        success, angles, error = self.arm.compute_ik(target)

        self.assertTrue(success)
        self.assertLessEqual(error, 2.0)
        self.assertTrue(np.all(angles >= 0.0))
        self.assertTrue(np.all(angles <= 180.0))


if __name__ == "__main__":
    unittest.main()
