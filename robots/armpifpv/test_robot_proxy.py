import math
import sys
import types
import unittest

from robots.armpifpv import kinematics
from robots.armpifpv.config import ARM_JOINT_NAMES


def _install_missing_roboflex_stubs():
    if "roboflex" not in sys.modules:
        rf = types.ModuleType("roboflex")

        class Node:
            pass

        class LastOne:
            pass

        rf.Node = Node
        rf.LastOne = LastOne
        sys.modules["roboflex"] = rf
    else:
        rf = sys.modules["roboflex"]

    for package_name in ["roboflex.transport", "roboflex.util"]:
        if package_name not in sys.modules:
            package = types.ModuleType(package_name)
            sys.modules[package_name] = package
            setattr(rf, package_name.rsplit(".", 1)[1], package)

    modules = {
        "roboflex.hiwonder_bus_servo": rf,
        "roboflex.transport.zmq": sys.modules["roboflex.transport"],
        "roboflex.util.jpeg": sys.modules["roboflex.util"],
        "roboflex.webcam_gst": rf,
    }
    for module_name, parent in modules.items():
        if module_name not in sys.modules:
            module = types.ModuleType(module_name)
            sys.modules[module_name] = module
            setattr(parent, module_name.rsplit(".", 1)[1], module)


_install_missing_roboflex_stubs()
from robots.armpifpv.robot import ArmProxy


class ArmPiFPVRobotProxyTests(unittest.TestCase):
    def test_arm_proxy_move_joints_accepts_partial_dict(self):
        class FakeRobot:
            def __init__(self):
                self.sent = None

            def _current_arm_joints(self):
                return kinematics.home_arm_joints_rad()

            def _send_arm_joints(self, joints, move_time):
                self.sent = (joints, move_time)

        fake = FakeRobot()
        arm = ArmProxy(fake)
        result = arm.move_joints({"base_yaw": math.radians(10.0)}, move_time=0.25)

        self.assertAlmostEqual(result["base_yaw"], math.radians(10.0))
        for name in ARM_JOINT_NAMES:
            self.assertIn(name, result)
        self.assertEqual(fake.sent[1], 0.25)


if __name__ == "__main__":
    unittest.main()
