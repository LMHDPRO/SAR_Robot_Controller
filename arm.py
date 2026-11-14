"""Four-axis arm kinematics used by the desktop preview and AGV controller."""
import numpy as np


class ArmKinematics:
    def __init__(self):
        self.d = np.array([127.027, 0, 0, 0], dtype=float)
        self.a = np.array([13.916, 119.401, 122.229, 122.271], dtype=float)
        self.alpha = np.array([-np.pi / 2, 0, 0, 0], dtype=float)
        self.max_reach = float(np.sum(np.abs(self.d)) + np.sum(np.abs(self.a)))
        self.servo_offset = np.array([0.0, 126.5, 74.2, 106.0], dtype=float)
        self.servo_dir = np.array([1.0, -1.0, 1.0, -1.0], dtype=float)
        self.startup_angles = np.array([90.0, 126.5, 74.2, 106.0], dtype=float)
        phys_min = np.zeros(4, dtype=float)
        phys_max = np.full(4, 180.0, dtype=float)
        dh_bounds_1 = self.convert_angles(phys_min)
        dh_bounds_2 = self.convert_angles(phys_max)
        self.limit_min = np.minimum(dh_bounds_1, dh_bounds_2)
        self.limit_max = np.maximum(dh_bounds_1, dh_bounds_2)
        self.last_ik_theta = self.convert_angles(self.startup_angles)

    def convert_angles(self, values, to_servo=False):
        values = np.asarray(values, dtype=float)
        if to_servo:
            return np.rad2deg(values) * self.servo_dir + self.servo_offset
        return np.deg2rad((values - self.servo_offset) * self.servo_dir)

    def compute_fk_dh(self, theta_input):
        theta = [
            theta_input[0],
            theta_input[1] - np.pi / 2,
            theta_input[2],
            theta_input[3],
        ]
        transform = np.eye(4)
        transforms = [transform.copy()]
        for index in range(4):
            ct, st = np.cos(theta[index]), np.sin(theta[index])
            ca, sa = np.cos(self.alpha[index]), np.sin(self.alpha[index])
            matrix = np.array([
                [ct, -st * ca, st * sa, self.a[index] * ct],
                [st, ct * ca, -ct * sa, self.a[index] * st],
                [0, sa, ca, self.d[index]],
                [0, 0, 0, 1],
            ])
            transform = transform @ matrix
            transforms.append(transform.copy())
        return transform[:3, 3], transforms

    def compute_fk(self, servo_deg):
        return self.compute_fk_dh(self.convert_angles(servo_deg))

    def compute_ik(self, target_xyz):
        target_xyz = np.asarray(target_xyz, dtype=float)
        if target_xyz.shape != (3,) or not np.all(np.isfinite(target_xyz)):
            return False, None, "WARN:NON_FINITE_TARGET"
        if np.linalg.norm(target_xyz) > self.max_reach * 0.98:
            return False, None, "WARN:SINGULARITY"

        theta = self.last_ik_theta.copy()
        best_theta = theta.copy()
        best_error = float("inf")
        for _ in range(80):
            for index in range(3, -1, -1):
                position, transforms = self.compute_fk_dh(theta)
                current_error = float(np.linalg.norm(position - target_xyz))
                if current_error < best_error:
                    best_error = current_error
                    best_theta = theta.copy()
                if current_error < 1.0:
                    break

                pivot = transforms[index][:3, 3]
                axis = transforms[index][:3, 2]
                current_vector = position - pivot
                target_vector = target_xyz - pivot
                current_projected = current_vector - np.dot(current_vector, axis) * axis
                target_projected = target_vector - np.dot(target_vector, axis) * axis
                current_magnitude = np.linalg.norm(current_projected)
                target_magnitude = np.linalg.norm(target_projected)
                if current_magnitude < 0.0001 or target_magnitude < 0.0001:
                    continue
                current_projected /= current_magnitude
                target_projected /= target_magnitude
                cosine = np.clip(np.dot(current_projected, target_projected), -1.0, 1.0)
                sine = np.dot(axis, np.cross(current_projected, target_projected))
                theta[index] += np.clip(np.arctan2(sine, cosine), -0.1, 0.1)
                theta[index] = np.clip(
                    theta[index], self.limit_min[index], self.limit_max[index]
                )
            if best_error < 1.0:
                break

        final_position, _ = self.compute_fk_dh(best_theta)
        error = float(np.linalg.norm(final_position - target_xyz))
        servo_angles = self.convert_angles(best_theta, to_servo=True)
        if (
            error > 2.0
            or not np.all(np.isfinite(servo_angles))
            or np.any(servo_angles < -0.25)
            or np.any(servo_angles > 180.25)
        ):
            return False, None, f"WARN:IK_ERROR={error:.1f}mm"

        self.last_ik_theta = best_theta
        return True, np.clip(servo_angles, 0.0, 180.0), error

    def enforce_safety_zone(self, target_xyz, minimum_radius=70.0):
        x, y, z = np.asarray(target_xyz, dtype=float)
        safe_reach = self.max_reach * 0.98
        radius = float(np.hypot(x, y))
        if radius > safe_reach:
            x, y = x / radius * safe_reach, y / radius * safe_reach
            radius = safe_reach
        if radius < minimum_radius:
            if radius < 0.1:
                x, y = minimum_radius, 0.0
            else:
                x, y = x / radius * minimum_radius, y / radius * minimum_radius
            radius = minimum_radius
        max_z = float(np.sqrt(max(0.0, safe_reach * safe_reach - radius * radius)))
        z = min(max(0.0, z), max_z)
        return np.array([x, y, z], dtype=float)
