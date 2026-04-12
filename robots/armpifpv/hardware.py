#!/usr/bin/env python3
"""ArmPi-FPV hardware server.

Runs on the Raspberry Pi, owns the camera and Hiwonder servo bus, and exposes
camera/state/command channels over ZMQ. The local graph is rooted in GraphRoot
so Roboflex metrics are available immediately for operator debugging.
"""

from __future__ import annotations

import argparse
import signal
import time

import roboflex as rf
import roboflex.hiwonder_bus_servo as rhs
import roboflex.transport.zmq as rzmq
import roboflex.util.jpeg as ruj
import roboflex.webcam_gst as rcw

from .config import (
    ALL_SERVO_IDS,
    CAMERA_DEVICE_INDEX,
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_USE_JPEG,
    CAMERA_WIDTH,
    GRAPH_DEBUG,
    GRAPH_METRICS_PRINTING_HZ,
    SERVO_BAUD_RATE,
    SERVO_DEVICE_NAME,
    SERVO_DYNAMIC_READ_EVERY_N_LOOPS,
    SERVO_LOOP_SLEEP_MS,
    SERVO_RETRIES,
    SERVO_TELEMETRY_EVERY_N_LOOPS,
    SERVO_TIMEOUT_MS,
    ZMQ_CAMERA_BIND,
    ZMQ_SERVO_CMD_CONNECT,
    ZMQ_SERVO_STATE_BIND,
)


class ArmPiFPVHardware:
    """Owns ArmPi-FPV hardware and the local Roboflex graph."""

    def __init__(self):
        self._graph = None
        self._zmq_ctx = None
        self._camera = None
        self._servo_node = None
        self._command_sub = None
        self._camera_pub = None
        self._state_pub = None

    def start(self):
        self._zmq_ctx = rzmq.ZMQContext()
        self._graph = rf.GraphRoot(
            GRAPH_METRICS_PRINTING_HZ,
            name="ArmPiFPVGraph",
            debug=GRAPH_DEBUG,
        )

        self._camera = rcw.WebcamSensor(
            CAMERA_WIDTH,
            CAMERA_HEIGHT,
            CAMERA_FPS,
            device_index=CAMERA_DEVICE_INDEX,
            emit_rgb=True,
            name="ArmPiCamera",
        )

        self._camera_pub = rzmq.ZMQPublisher(
            self._zmq_ctx,
            ZMQ_CAMERA_BIND,
            name="ArmPiCamPub",
            max_queued_msgs=1,
        )

        if CAMERA_USE_JPEG:
            jpeg = ruj.JPEGCompressor(image_key="rgb", output_key="jpeg", name="ArmPiJPEG", debug=False)
            self._graph > self._camera > jpeg > self._camera_pub
        else:
            self._graph > self._camera > self._camera_pub

        controller = rhs.HiwonderBusServoController(
            SERVO_DEVICE_NAME,
            SERVO_BAUD_RATE,
            SERVO_TIMEOUT_MS,
            SERVO_RETRIES,
        )
        controller.set_loop_sleep_ms(SERVO_LOOP_SLEEP_MS)

        read_config = rhs.DynamicReadConfig()
        read_config.servo_ids = ALL_SERVO_IDS
        read_config.loop_sleep_ms = SERVO_LOOP_SLEEP_MS
        read_config.set_every_n_loops(rhs.DynamicReadField.Position, SERVO_DYNAMIC_READ_EVERY_N_LOOPS)
        read_config.set_every_n_loops(rhs.DynamicReadField.VoltageMV, SERVO_TELEMETRY_EVERY_N_LOOPS)
        read_config.set_every_n_loops(rhs.DynamicReadField.TemperatureC, SERVO_TELEMETRY_EVERY_N_LOOPS)
        read_config.set_every_n_loops(rhs.DynamicReadField.TorqueEnabled, SERVO_TELEMETRY_EVERY_N_LOOPS)

        self._servo_node = rhs.HiwonderBusServoGroupNode(
            controller,
            read_config,
            name="ArmPiServoNode",
        )

        self._command_sub = rzmq.ZMQSubscriber(
            self._zmq_ctx,
            ZMQ_SERVO_CMD_CONNECT,
            name="ArmPiServoCmdSub",
            max_queued_msgs=1,
        )
        self._state_pub = rzmq.ZMQPublisher(
            self._zmq_ctx,
            ZMQ_SERVO_STATE_BIND,
            name="ArmPiServoStatePub",
            max_queued_msgs=1,
        )

        self._graph > self._command_sub > self._servo_node > self._state_pub
        self._graph.profile()

        print("ArmPi-FPV hardware server running.", flush=True)
        print(f"  Camera: {CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}, device_index={CAMERA_DEVICE_INDEX}", flush=True)
        print(f"    Camera pub: {ZMQ_CAMERA_BIND}", flush=True)
        print(f"  Servo device: {SERVO_DEVICE_NAME} @ {SERVO_BAUD_RATE}", flush=True)
        print(f"    Servo state pub: {ZMQ_SERVO_STATE_BIND}", flush=True)
        print(f"    Servo cmd sub:   {ZMQ_SERVO_CMD_CONNECT}", flush=True)
        print(f"  Graph metrics: enabled via GraphRoot at {GRAPH_METRICS_PRINTING_HZ} Hz", flush=True)

    def stop(self):
        if self._graph is not None:
            self._graph.stop()
            self._graph = None

    def run_self_test(self, save_frame_path: str | None = None):
        """Run a conservative local smoke test through the normal proxy path."""
        from .robot import ArmPiFPVRobotProxy

        robot = ArmPiFPVRobotProxy()
        try:
            print("Connecting local proxy for self-test...", flush=True)
            robot.connect()

            deadline = time.time() + 5.0
            while time.time() < deadline:
                status = robot.is_alive()
                if status["all"]:
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError("hardware graph started, but local proxy did not receive camera/servo data")

            print("Self-test: initial status", robot.is_alive(), flush=True)
            initial_joints = robot.arm.get_joint_positions()
            initial_pose = robot.arm.get_pose()
            print("Self-test: initial joints", initial_joints, flush=True)
            print("Self-test: initial pose", initial_pose, flush=True)

            if save_frame_path:
                from PIL import Image

                frame = robot.cameras["main"].grab_frame(timeout=5.0)
                Image.fromarray(frame).save(save_frame_path)
                print(f"Self-test: saved frame to {save_frame_path}", flush=True)

            print("Self-test: opening gripper...", flush=True)
            robot.gripper.open(move_time=0.8)
            time.sleep(1.0)

            print("Self-test: closing gripper...", flush=True)
            robot.gripper.close(move_time=0.8)
            time.sleep(1.0)

            print("Self-test: reopening gripper...", flush=True)
            robot.gripper.open(move_time=0.8)
            time.sleep(1.0)

            print("Self-test: moving to home pose...", flush=True)
            robot.arm.home(move_time=1.2)
            time.sleep(1.5)

            home_joints = robot.arm.get_joint_positions()
            sequence = [
                {
                    "name": "base_yaw_left",
                    "joints": {
                        **home_joints,
                        "base_yaw": home_joints["base_yaw"] + 0.25,
                    },
                    "move_time": 1.0,
                },
                {
                    "name": "base_yaw_right",
                    "joints": {
                        **home_joints,
                        "base_yaw": home_joints["base_yaw"] - 0.25,
                    },
                    "move_time": 1.0,
                },
                {
                    "name": "shoulder_elbow_check",
                    "joints": {
                        **home_joints,
                        "shoulder": home_joints["shoulder"] + 0.18,
                        "elbow": home_joints["elbow"] - 0.18,
                    },
                    "move_time": 1.0,
                },
                {
                    "name": "wrist_check",
                    "joints": {
                        **home_joints,
                        "wrist_pitch": home_joints["wrist_pitch"] + 0.18,
                        "wrist_roll": home_joints["wrist_roll"] + 0.20,
                    },
                    "move_time": 1.0,
                },
                {
                    "name": "return_home",
                    "joints": home_joints,
                    "move_time": 1.2,
                },
            ]

            for step in sequence:
                print(f"Self-test: {step['name']}...", flush=True)
                robot.arm.move_joints(step["joints"], move_time=step["move_time"])
                time.sleep(step["move_time"] + 0.4)
                print("  joints", robot.arm.get_joint_positions(), flush=True)
                print("  pose", robot.arm.get_pose(), flush=True)

            pose = robot.arm.get_pose()
            ik_result = robot.arm.ik(
                position=pose["position"],
                orientation=pose["orientation"],
                prefer_current=True,
                allow_approx=False,
            )
            print("Self-test: FK/IK consistency check passed", flush=True)
            print("  IK result", ik_result, flush=True)
        finally:
            try:
                robot.close()
            except Exception:
                pass


def _parse_args():
    parser = argparse.ArgumentParser(description="Run the ArmPi-FPV hardware server.")
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="start the hardware graph, run a conservative local test sequence, then stop",
    )
    parser.add_argument(
        "--self-test-frame",
        default=None,
        help="optional output path for a single saved self-test camera frame",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    robot = ArmPiFPVHardware()
    robot.start()

    try:
        if args.self_test:
            robot.run_self_test(save_frame_path=args.self_test_frame)
        else:
            def _on_sigint(sig, frame):
                del sig, frame
                globals()["_armpifpv_shutdown"] = True

            globals()["_armpifpv_shutdown"] = False
            signal.signal(signal.SIGINT, _on_sigint)

            while not globals()["_armpifpv_shutdown"]:
                time.sleep(0.1)
    finally:
        robot.stop()
