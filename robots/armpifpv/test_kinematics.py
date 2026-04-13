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


if __name__ == "__main__":
    unittest.main()
