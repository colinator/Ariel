"""ArmPi-FPV Ariel robot proxy."""

from __future__ import annotations

import base64
import io
import threading
import time

import roboflex as rf
import roboflex.hiwonder_bus_servo as rhs
import roboflex.transport.zmq as rzmq
import roboflex.util.jpeg as ruj
import roboflex.webcam_gst as rcw

from server.robot_base import RobotBase
from . import kinematics
from .config import (
    ALL_ACTUATOR_NAMES,
    ARM_JOINT_NAMES,
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_USE_JPEG,
    CAMERA_WIDTH,
    DEFAULT_MOVE_TIME_S,
    GRIPPER_CLOSED_PULSE,
    GRIPPER_OPEN_PULSE,
    NAME_BY_SERVO_ID,
    PULSE_MAP_BY_NAME,
    SERVO_ID_BY_NAME,
    STATE_MAX_AGE_S,
    TOOL_FRAME,
    ZMQ_CAMERA_CONNECT,
    ZMQ_SERVO_CMD_BIND,
    ZMQ_SERVO_STATE_CONNECT,
)


def _clamp_gripper_openness(value: float) -> float:
    return min(max(float(value), 0.0), 1.0)


def _gripper_openness_to_pulse(value: float) -> int:
    # Convenience mapping only: the ArmPi-FPV gripper is not calibrated as a
    # linear jaw-width actuator, so normalized openness is approximate.
    openness = _clamp_gripper_openness(value)
    pulse = GRIPPER_CLOSED_PULSE + (GRIPPER_OPEN_PULSE - GRIPPER_CLOSED_PULSE) * openness
    return int(round(pulse))


def _gripper_pulse_to_openness(pulse: float) -> float:
    if pulse is None:
        raise RuntimeError("gripper pulse is not available yet")
    denom = GRIPPER_OPEN_PULSE - GRIPPER_CLOSED_PULSE
    if denom == 0:
        return 0.0
    return _clamp_gripper_openness((float(pulse) - GRIPPER_CLOSED_PULSE) / denom)


class CameraProxy:
    def __init__(self, zmq_ctx, endpoint: str):
        self._latest_frame = None
        self._last_recv = None
        self._lock = threading.Lock()

        self._sub = rzmq.ZMQSubscriber(zmq_ctx, endpoint, name="ArmPiCamSub", max_queued_msgs=1)
        if CAMERA_USE_JPEG:
            self._jpeg = ruj.JPEGDecompressor(input_key="jpeg", output_key="rgb", name="ArmPiJPEGDec", debug=False)
            self._cb = rf.CallbackFun(self._on_frame, "ArmPiCamCB")
            self._sub > self._jpeg > self._cb
        else:
            self._jpeg = None
            self._cb = rf.CallbackFun(self._on_frame, "ArmPiCamCB")
            self._sub > self._cb

    def _on_frame(self, msg):
        rgb = rcw.WebcamDataRGB(msg).rgb
        with self._lock:
            self._latest_frame = rgb
            self._last_recv = time.time()

    def start(self):
        self._sub.start()

    def stop(self):
        self._sub.stop()

    def grab_frame(self, timeout: float = 5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._latest_frame is not None:
                    return self._latest_frame.copy()
            time.sleep(0.01)
        raise TimeoutError(f"no frame received within {timeout}s")

    @property
    def last_recv(self):
        return self._last_recv


class _ServoStateReceiver:
    def __init__(self, zmq_ctx, endpoint: str):
        self._lock = threading.Lock()
        self._latest_state = {}
        self._latest_joint_state = {}
        self._last_recv = None

        self._sub = rzmq.ZMQSubscriber(zmq_ctx, endpoint, name="ArmPiStateSub", max_queued_msgs=1)
        self._cb = rf.CallbackFun(self._on_state, "ArmPiStateCB")
        self._sub > self._cb

    def _on_state(self, msg):
        state = rhs.HiwonderBusServoGroupStateMessage(msg).state
        latest = {}
        joints = {}
        for servo_id, dyn_state in state.values.items():
            name = NAME_BY_SERVO_ID.get(servo_id)
            if name is None:
                continue
            latest[name] = {
                "servo_id": servo_id,
                "pulse": dyn_state.position,
                "voltage_mv": dyn_state.voltage_mv,
                "temperature_c": dyn_state.temperature_c,
                "torque_enabled": dyn_state.torque_enabled,
            }
            if name in ARM_JOINT_NAMES and dyn_state.position is not None:
                joints[name] = kinematics.pulse_to_radians(name, dyn_state.position)
        with self._lock:
            self._latest_state = latest
            self._latest_joint_state = joints
            self._last_recv = time.time()

    def start(self):
        self._sub.start()

    def stop(self):
        self._sub.stop()

    def snapshot(self):
        with self._lock:
            return dict(self._latest_state), dict(self._latest_joint_state), self._last_recv


class JointProxy:
    def __init__(self, robot: "ArmPiFPVRobotProxy", name: str):
        self._robot = robot
        self.name = name
        self.id = SERVO_ID_BY_NAME[name]

    def get_position(self) -> float:
        return self._robot.arm.get_joint_positions()[self.name]

    def set_position(self, value: float, move_time: float = DEFAULT_MOVE_TIME_S, wait: bool = False) -> float:
        self._robot.arm.set_joint_positions(move_time=move_time, wait=wait, **{self.name: value})
        return self.get_position()

    def get_limits(self):
        return kinematics.JOINT_LIMITS_RAD[self.name]


class GripperProxy:
    def __init__(self, robot: "ArmPiFPVRobotProxy"):
        self._robot = robot
        self.name = "gripper"
        self.id = SERVO_ID_BY_NAME["gripper"]

    def get_position(self) -> float:
        state = self._robot._latest_state_for("gripper")
        return _gripper_pulse_to_openness(state["pulse"])

    def get_raw_pulse(self) -> int:
        state = self._robot._latest_state_for("gripper")
        return int(state["pulse"])

    def set_position(self, value: float, move_time: float = DEFAULT_MOVE_TIME_S, wait: bool = False) -> float:
        self._robot._send_gripper_openness(value, move_time=move_time)
        if wait:
            self.wait_until_near(value, timeout=move_time + 2.0)
        return _clamp_gripper_openness(value)

    def open(self, move_time: float = DEFAULT_MOVE_TIME_S, wait: bool = False) -> float:
        return self.set_position(1.0, move_time=move_time, wait=wait)

    def close(self, move_time: float = DEFAULT_MOVE_TIME_S, wait: bool = False) -> float:
        return self.set_position(0.0, move_time=move_time, wait=wait)

    def wait_until_near(
        self,
        target_openness: float,
        timeout: float | None = None,
        tolerance: float = 0.08,
        settle_time: float = 0.25,
        poll_s: float = 0.05,
    ) -> float:
        target = _clamp_gripper_openness(target_openness)
        deadline = time.time() + (timeout if timeout is not None else DEFAULT_MOVE_TIME_S + 2.0)
        settled_since = None
        last_value = None

        while time.time() < deadline:
            last_value = self.get_position()
            if abs(last_value - target) <= tolerance:
                if settled_since is None:
                    settled_since = time.time()
                elif time.time() - settled_since >= settle_time:
                    return last_value
            else:
                settled_since = None
            time.sleep(poll_s)

        if last_value is None:
            raise TimeoutError("gripper state not available before wait timed out")
        return last_value

    def get_limits(self):
        return (0.0, 1.0)


class RawServoProxy:
    def __init__(self, robot: "ArmPiFPVRobotProxy", name: str):
        self._robot = robot
        self.name = name
        self.id = SERVO_ID_BY_NAME[name]

    def get_pulse(self) -> int:
        state = self._robot._latest_state_for(self.name)
        pulse = state["pulse"]
        if pulse is None:
            raise RuntimeError(f"pulse is not available yet for '{self.name}'")
        return int(pulse)

    def move_to_pulse(self, pulse: int, move_time: float = DEFAULT_MOVE_TIME_S) -> int:
        return self._robot._send_raw_servo_pulse(self.id, pulse, move_time=move_time)


class ArmProxy:
    def __init__(self, robot: "ArmPiFPVRobotProxy"):
        self._robot = robot

    def get_joint_positions(self) -> dict[str, float]:
        return self._robot._current_arm_joints()

    def set_joint_positions(
        self,
        move_time: float = DEFAULT_MOVE_TIME_S,
        wait: bool = False,
        **kwargs,
    ) -> dict[str, float]:
        current = self.get_joint_positions()
        target = dict(current)
        for name, value in kwargs.items():
            if name not in ARM_JOINT_NAMES:
                raise ValueError(f"unknown arm joint '{name}', available: {ARM_JOINT_NAMES}")
            target[name] = float(value)
        return self.move_joints(target, move_time=move_time, wait=wait)

    def move_joints(
        self,
        joints,
        move_time: float = DEFAULT_MOVE_TIME_S,
        wait: bool = False,
        tolerance_rad: float = 0.10,
    ) -> dict[str, float]:
        joint_dict = kinematics.coerce_joint_dict(joints)
        self._robot._send_arm_joints(joint_dict, move_time=move_time)
        if wait:
            return self.wait_until_joints_near(joint_dict, timeout=move_time + 2.0, tolerance_rad=tolerance_rad)
        return joint_dict

    def home(self, move_time: float = DEFAULT_MOVE_TIME_S, wait: bool = False) -> dict[str, float]:
        return self.move_joints(kinematics.home_arm_joints_rad(), move_time=move_time, wait=wait)

    def stop(self):
        self._robot._stop_servos()

    def wait_until_joints_near(
        self,
        target_joints,
        timeout: float | None = None,
        tolerance_rad: float = 0.10,
        settle_time: float = 0.25,
        poll_s: float = 0.05,
    ) -> dict[str, float]:
        target = kinematics.coerce_joint_dict(target_joints)
        deadline = time.time() + (timeout if timeout is not None else DEFAULT_MOVE_TIME_S + 2.0)
        settled_since = None
        last_joints = None

        while time.time() < deadline:
            last_joints = self.get_joint_positions()
            if all(abs(last_joints[name] - target[name]) <= tolerance_rad for name in ARM_JOINT_NAMES):
                if settled_since is None:
                    settled_since = time.time()
                elif time.time() - settled_since >= settle_time:
                    return last_joints
            else:
                settled_since = None
            time.sleep(poll_s)

        if last_joints is None:
            raise TimeoutError("arm joint state not available before wait timed out")
        return last_joints

    def fk(self, joints=None) -> dict:
        if joints is None:
            joints = self.get_joint_positions()
        return kinematics.fk_pose(joints)

    def get_pose(self) -> dict:
        return self.fk(self.get_joint_positions())

    def ik(
        self,
        position,
        orientation,
        prefer_current: bool = True,
        allow_approx: bool = True,
        resolution=None,
    ) -> dict:
        del resolution
        initial = self.get_joint_positions() if prefer_current else kinematics.home_arm_joints_rad()
        return kinematics.solve_ik(
            position,
            orientation,
            initial_joints=initial,
            prefer_current=prefer_current,
            allow_approx=allow_approx,
        )

    def move_pose(
        self,
        position,
        orientation,
        move_time: float = DEFAULT_MOVE_TIME_S,
        allow_approx: bool = True,
        wait: bool = False,
        resolution=None,
    ) -> dict:
        result = self.ik(position, orientation, prefer_current=True, allow_approx=allow_approx, resolution=resolution)
        reached = self.move_joints(result["joints"], move_time=move_time, wait=wait)
        if wait:
            result["joints_reached"] = reached
        return result


class ArmPiFPVRobotProxy(RobotBase):
    def __init__(self):
        super().__init__()
        self._zmq_ctx = rzmq.ZMQContext()
        self._connected = False

        self.cameras = {}
        self.motors = {}
        self.servos = {}
        self.arm = ArmProxy(self)
        self.gripper = GripperProxy(self)

        self._camera_proxy = None
        self._state_receiver = None
        self._cmd_pub = None

    def connect(self) -> None:
        if self._connected:
            return

        self._camera_proxy = CameraProxy(self._zmq_ctx, ZMQ_CAMERA_CONNECT)
        self._camera_proxy.start()
        self.cameras["main"] = self._camera_proxy

        self._cmd_pub = rzmq.ZMQPublisher(self._zmq_ctx, ZMQ_SERVO_CMD_BIND, name="ArmPiCmdPub", max_queued_msgs=1)
        self._state_receiver = _ServoStateReceiver(self._zmq_ctx, ZMQ_SERVO_STATE_CONNECT)
        self._state_receiver.start()

        for name in ARM_JOINT_NAMES:
            self.motors[name] = JointProxy(self, name)
            self.servos[name] = RawServoProxy(self, name)
        self.motors["gripper"] = self.gripper
        self.servos["gripper"] = RawServoProxy(self, "gripper")

        self._publish_command(rhs.HiwonderBusServoGroupCommand())
        time.sleep(1.0)
        self._connected = True

    def close(self) -> None:
        try:
            self.stop_all_tasks(timeout=1.0)
        except Exception:
            pass

        if self._camera_proxy is not None:
            self._camera_proxy.stop()
        if self._state_receiver is not None:
            self._state_receiver.stop()

        self._camera_proxy = None
        self._state_receiver = None
        self._cmd_pub = None
        self.cameras = {}
        self.motors = {}
        self.servos = {}
        self.arm = ArmProxy(self)
        self.gripper = GripperProxy(self)
        self._connected = False
        self._tasks = {}

    def _publish_command(self, command: rhs.HiwonderBusServoGroupCommand):
        message = rhs.HiwonderBusServoGroupCommandMessage(command)
        if hasattr(self._cmd_pub, "signal_self"):
            self._cmd_pub.signal_self(message)
        else:
            self._cmd_pub.publish(message)

    def _send_arm_joints(self, joints: dict[str, float], move_time: float):
        command = rhs.HiwonderBusServoGroupCommand()
        command.should_write = True
        command.duration_ms = int(round(max(move_time, 0.01) * 1000.0))
        command.positions = {
            SERVO_ID_BY_NAME[name]: kinematics.radians_to_pulse(name, joints[name])
            for name in ARM_JOINT_NAMES
        }
        self._publish_command(command)

    def _send_gripper_openness(self, openness: float, move_time: float):
        command = rhs.HiwonderBusServoGroupCommand()
        command.should_write = True
        command.duration_ms = int(round(max(move_time, 0.01) * 1000.0))
        command.positions = {SERVO_ID_BY_NAME["gripper"]: _gripper_openness_to_pulse(openness)}
        self._publish_command(command)

    def _send_raw_servo_pulse(self, servo_id: int, pulse: int, move_time: float) -> int:
        command = rhs.HiwonderBusServoGroupCommand()
        command.should_write = True
        command.duration_ms = int(round(max(move_time, 0.01) * 1000.0))
        clamped = int(round(min(max(pulse, 0), 1000)))
        command.positions = {servo_id: clamped}
        self._publish_command(command)
        return clamped

    def _stop_servos(self):
        command = rhs.HiwonderBusServoGroupCommand()
        command.should_write = False
        command.positions = {SERVO_ID_BY_NAME[name]: 0 for name in ALL_ACTUATOR_NAMES}
        self._publish_command(command)

    def _state_snapshot(self):
        if self._state_receiver is None:
            return {}, {}, None
        return self._state_receiver.snapshot()

    def _latest_state_for(self, name: str) -> dict:
        latest, _, _ = self._state_snapshot()
        if name not in latest:
            raise RuntimeError(f"no state received yet for '{name}'")
        return latest[name]

    def _current_arm_joints(self) -> dict[str, float]:
        _, joints, _ = self._state_snapshot()
        if any(name not in joints for name in ARM_JOINT_NAMES):
            raise RuntimeError("arm state not available yet")
        return {name: joints[name] for name in ARM_JOINT_NAMES}

    def is_alive(self, max_age: float = STATE_MAX_AGE_S) -> dict:
        now = time.time()
        camera_alive = (
            self._camera_proxy is not None and
            self._camera_proxy.last_recv is not None and
            (now - self._camera_proxy.last_recv) < max_age
        )
        _, _, last_state = self._state_snapshot()
        servo_alive = last_state is not None and (now - last_state) < max_age
        return {"camera": camera_alive, "servos": servo_alive, "all": camera_alive and servo_alive}

    def snapshot(self, device_name: str) -> dict:
        if device_name in self.cameras:
            from PIL import Image
            frame = self.cameras[device_name].grab_frame()
            buf = io.BytesIO()
            Image.fromarray(frame).save(buf, format="JPEG", quality=60)
            buf.seek(0)
            return {
                "type": "image",
                "format": "jpeg",
                "data": base64.b64encode(buf.read()).decode("ascii"),
                "width": frame.shape[1],
                "height": frame.shape[0],
            }

        if device_name == "arm":
            joints = self.arm.get_joint_positions()
            return {"type": "state", "name": "arm", "joints": joints, "pose": self.arm.get_pose()}

        if device_name == "gripper":
            state = self._latest_state_for("gripper")
            return {
                "type": "state",
                "name": "gripper",
                "servo_id": state["servo_id"],
                "openness": _gripper_pulse_to_openness(state["pulse"]),
                "pulse": state["pulse"],
                "limits": self.gripper.get_limits(),
            }

        if device_name in self.motors:
            proxy = self.motors[device_name]
            if device_name == "gripper":
                return self.snapshot("gripper")
            state = self._latest_state_for(device_name)
            return {
                "type": "state",
                "name": device_name,
                "servo_id": state["servo_id"],
                "position": proxy.get_position(),
                "pulse": state["pulse"],
                "limits": proxy.get_limits(),
            }

        available = list(self.cameras.keys()) + ["arm"] + list(self.motors.keys())
        raise ValueError(f"unknown device '{device_name}', available: {available}")

    def _device_docs(self) -> str:
        status = self.is_alive()
        lines = []
        lines.append("# Robot: ArmPi-FPV")
        lines.append("Hiwonder ArmPi-FPV with a 5-DOF arm, 1-DOF parallel gripper, and one USB camera.")
        lines.append("")

        if status["all"]:
            lines.append("**Status: CONNECTED** (camera and servos active)")
        else:
            parts = []
            if not status["camera"]:
                parts.append("camera: no data")
            if not status["servos"]:
                parts.append("servos: no data")
            lines.append(f"**Status: HARDWARE OFFLINE** ({', '.join(parts)})")
            lines.append("Is `python -m robots.armpifpv.hardware` running on the Pi?")
        lines.append("")

        lines.append("## Devices")
        lines.append("")
        lines.append("### Camera: 'main'")
        lines.append(f"  USB webcam, {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps, RGB frames")
        lines.append("  snapshot('main') -> returns a live JPEG image")
        lines.append("  robot.cameras['main'].grab_frame() -> numpy (H, W, 3) uint8, RGB")
        lines.append("")

        lines.append("### Arm")
        lines.append("  Semantic joints in order:")
        for name in ARM_JOINT_NAMES:
            lo, hi = kinematics.JOINT_LIMITS_RAD[name]
            lines.append(f"  - {name}: {lo:+.3f} .. {hi:+.3f} rad")
        lines.append("  All arm joint APIs use radians.")
        lines.append("  This hardware strongly prefers timed waypoint-style moves over high-rate streaming.")
        lines.append("  High-rate control loops are allowed, but they may produce visibly jerky motion.")
        lines.append("  On this robot, very slow creeping motions often look worse than small brisk steps.")
        lines.append("")

        lines.append("### Gripper")
        lines.append("  `robot.gripper` is a single parallel-jaw gripper DOF.")
        lines.append("  Gripper API uses approximate normalized openness: 0.0 = more closed, 1.0 = more open.")
        lines.append(f"  Practical first-pass pulses are about {GRIPPER_CLOSED_PULSE} for close-ish and {GRIPPER_OPEN_PULSE} for open-ish.")
        lines.append("  Do not assume linear jaw width or exact repeatability from intermediate openness values.")
        lines.append("")

        lines.append("## robot API Reference")
        lines.append("")
        lines.append("### Camera")
        lines.append("  robot.cameras['main'].grab_frame(timeout=5.0) -> numpy RGB image")
        lines.append("")
        lines.append("### Arm Joint Motion")
        lines.append("  robot.arm.get_joint_positions() -> {'base_yaw': ..., 'shoulder': ..., ...}")
        lines.append("  robot.arm.set_joint_positions(base_yaw=..., shoulder=..., move_time=1.0, wait=False)")
        lines.append("  robot.arm.move_joints({'base_yaw': ..., 'shoulder': ..., ...}, move_time=1.0, wait=False)")
        lines.append("  robot.arm.wait_until_joints_near(target_joints, timeout=3.0, tolerance_rad=0.10)")
        lines.append("  robot.arm.home(move_time=1.0, wait=False)")
        lines.append("  robot.arm.stop()")
        lines.append("")
        lines.append("### Pose / FK / IK")
        lines.append("  robot.arm.get_pose() -> {'position': [x, y, z], 'orientation': [x, y, z, w], 'frame': 'base_link'}")
        lines.append("  robot.arm.fk(joints=None) -> same pose shape")
        lines.append("  robot.arm.ik(position=[x, y, z], orientation=[x, y, z, w], prefer_current=True, allow_approx=True)")
        lines.append("  robot.arm.move_pose(position=[x, y, z], orientation=[x, y, z, w], move_time=1.0, allow_approx=True, wait=False)")
        lines.append("")
        lines.append("  IMPORTANT:")
        lines.append("  Pose APIs use explicit SE(3)-style position + quaternion orientation.")
        lines.append("  This robot is only 5-DOF plus gripper, so it cannot realize arbitrary 6-DOF end-effector orientations.")
        lines.append("  IK may therefore approximate the requested orientation.")
        lines.append("  `robot.arm.ik(...)` returns both the requested and achieved poses, plus position/orientation error metrics.")
        lines.append("  Cartesian execution on this robot is approximate: expect small errors, backlash, and visibly jerky motion.")
        lines.append("  Prefer small staged pose changes over large one-shot Cartesian jumps.")
        lines.append("  In practice, short-to-moderate timed moves often look better than long slow drags.")
        lines.append("  For multi-step tasks, prefer timed motion followed by `wait=True` or `robot.arm.wait_until_joints_near(...)` before the next action.")
        lines.append("  Small Cartesian moves in X/Z are usually more reliable than lateral Y moves from a typical forward-facing posture.")
        lines.append("")
        lines.append("### Individual Joints")
        for name in ARM_JOINT_NAMES:
            lines.append(f"  robot.motors['{name}'].get_position() -> radians")
            lines.append(f"  robot.motors['{name}'].set_position(value_rad, move_time=1.0, wait=False)")
        lines.append("")
        lines.append("### Gripper")
        lines.append("  robot.gripper.get_position() -> approximate openness in [0.0, 1.0]")
        lines.append("  robot.gripper.set_position(openness, move_time=1.0, wait=False)")
        lines.append("  robot.gripper.wait_until_near(target_openness, timeout=3.0, tolerance=0.08)")
        lines.append("  robot.gripper.open(move_time=1.0, wait=False)")
        lines.append("  robot.gripper.close(move_time=1.0, wait=False)")
        lines.append("")
        lines.append("### Introspection")
        lines.append("  robot.describe() -> this text")
        lines.append("  robot.snapshot('main' | 'arm' | 'gripper' | joint_name)")
        lines.append("  robot.is_alive() -> {'camera': bool, 'servos': bool, 'all': bool}")
        lines.append("")

        lines.append("## Tips For LLM Control")
        lines.append("- Prefer small staged moves. For a typical step of about 0.1 to 0.35 rad (6 to 20 deg), start with `move_time` around 0.3 to 0.5 seconds.")
        lines.append("- As a rough rule of thumb, effective joint motion around about 0.4 to 1.0 rad/s tends to look better than very slow creeping motion.")
        lines.append("- If a loop is needed, use repeated small quick corrections rather than one long slow tracking move.")
        lines.append("- For reliable multi-step behaviors, use `wait=True` or call the explicit wait helpers before the next step.")
        lines.append("- Use `robot.arm.get_pose()` or `snapshot('arm')` before planning a grasp.")
        lines.append("- For grasping, think in terms of target tool pose, but expect orientation approximation because the arm is 5-DOF.")
        lines.append("- For Cartesian moves, prefer small offsets and re-check `robot.arm.get_pose()` after each step instead of assuming the first move was exact.")
        lines.append("- In practice, small X/Z Cartesian adjustments are usually more effective than Y adjustments from a cobra-like forward-facing pose.")
        lines.append("- If a Y move is important, expect approximation and consider breaking it into several small steps with pose checks between them.")
        lines.append("- Prefer `robot.gripper.open()` / `robot.gripper.close()` over fine-grained openness targets unless you have calibrated this specific gripper.")
        lines.append("- Coordinated multi-joint moves usually look rougher than single-joint moves on this hardware.")
        lines.append("- If you need repeated closed-loop updates, keep the loop moderate. This is not a smooth high-bandwidth servo stack.")
        lines.append("- Frames from the camera are RGB, not BGR.")
        lines.append("")

        return "\n".join(lines)
