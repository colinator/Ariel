import math
import unittest

import numpy as np

from robots.armpifpv import kinematics
from robots.armpifpv.config import ARM_JOINT_NAMES


def _interp(lo: float, hi: float, alpha: float) -> float:
    return lo + alpha * (hi - lo)


class ArmPiFPVKinematicsTests(unittest.TestCase):
    def test_joint_angle_pulse_roundtrip_stays_close(self):
        for name in ARM_JOINT_NAMES:
            lo, hi = kinematics.JOINT_LIMITS_RAD[name]
            for alpha in (0.1, 0.3, 0.5, 0.7, 0.9):
                angle = _interp(lo, hi, alpha)
                pulse = kinematics.radians_to_pulse(name, angle)
                recovered = kinematics.pulse_to_radians(name, pulse)
                self.assertAlmostEqual(
                    recovered,
                    angle,
                    delta=math.radians(1.0),
                    msg=f"{name} roundtrip drifted too far at alpha={alpha}",
                )

    def test_fk_returns_normalized_quaternion_in_base_frame(self):
        joints = kinematics.home_arm_joints_rad()
        pose = kinematics.fk_pose(joints)
        quat = np.asarray(pose["orientation"], dtype=float)

        self.assertEqual(pose["frame"], "base_link")
        self.assertEqual(quat.shape, (4,))
        self.assertAlmostEqual(float(np.linalg.norm(quat)), 1.0, delta=1e-6)

    def test_ik_recovers_pose_generated_by_fk(self):
        sample_joints = {
            "base_yaw": math.radians(15.0),
            "shoulder": math.radians(-115.0),
            "elbow": math.radians(55.0),
            "wrist_pitch": math.radians(-35.0),
            "wrist_roll": math.radians(20.0),
        }
        pose = kinematics.fk_pose(sample_joints)
        result = kinematics.solve_ik(
            pose["position"],
            pose["orientation"],
            initial_joints=sample_joints,
            prefer_current=True,
            allow_approx=False,
            max_iters=300,
        )

        self.assertTrue(result["success"])
        self.assertFalse(result["approximated"])
        self.assertLessEqual(result["position_error_m"], 0.01)
        self.assertLessEqual(result["orientation_error_rad"], 0.20)

        for name in ARM_JOINT_NAMES:
            self.assertAlmostEqual(
                result["joints"][name],
                sample_joints[name],
                delta=math.radians(8.0),
                msg=f"{name} IK recovery drifted too far",
            )

    def test_base_yaw_step_rotates_xy_without_material_z_change(self):
        joints = {
            "base_yaw": math.radians(0.0),
            "shoulder": math.radians(-120.0),
            "elbow": math.radians(60.0),
            "wrist_pitch": math.radians(-20.0),
            "wrist_roll": math.radians(0.0),
        }
        pose0 = kinematics.fk_pose(joints)
        joints["base_yaw"] += math.radians(10.0)
        pose1 = kinematics.fk_pose(joints)

        p0 = np.asarray(pose0["position"], dtype=float)
        p1 = np.asarray(pose1["position"], dtype=float)
        r0 = float(np.linalg.norm(p0[:2]))
        r1 = float(np.linalg.norm(p1[:2]))

        self.assertAlmostEqual(p0[2], p1[2], delta=0.01)
        self.assertAlmostEqual(r0, r1, delta=0.01)

    def test_small_z_offset_ik_stays_close_to_seed(self):
        sample_joints = {
            "base_yaw": math.radians(-5.0),
            "shoulder": math.radians(-145.0),
            "elbow": math.radians(80.0),
            "wrist_pitch": math.radians(-15.0),
            "wrist_roll": math.radians(0.0),
        }
        pose = kinematics.fk_pose(sample_joints)
        requested_position = list(pose["position"])
        requested_position[2] += 0.01

        result = kinematics.solve_ik(
            requested_position,
            pose["orientation"],
            initial_joints=sample_joints,
            prefer_current=True,
            allow_approx=True,
            max_iters=300,
        )

        self.assertTrue(result["success"])
        self.assertLessEqual(result["position_error_m"], 0.02)
        self.assertLessEqual(result["orientation_error_rad"], 0.30)

        max_joint_delta = max(
            abs(result["joints"][name] - sample_joints[name])
            for name in ARM_JOINT_NAMES
        )
        self.assertLessEqual(max_joint_delta, math.radians(20.0))


if __name__ == "__main__":
    unittest.main()
