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
import roboflex.transport.mqtt as rtm
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
    GRAPH_METRICS_BROKER,
    GRAPH_METRICS_PORT,
    GRAPH_METRICS_PRINTING_HZ,
    GRAPH_METRICS_QOS,
    GRAPH_METRICS_TOPIC,
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
        self._mqtt_ctx = None
        self._metrics_pub = None

    def _make_graph_root(self):
        if GRAPH_METRICS_BROKER:
            self._mqtt_ctx = rtm.MQTTContext()
            self._metrics_pub = rtm.MQTTPublisher(
                self._mqtt_ctx,
                GRAPH_METRICS_BROKER,
                GRAPH_METRICS_PORT,
                GRAPH_METRICS_TOPIC,
                name="ArmPiMetricsPub",
                qos=GRAPH_METRICS_QOS,
                debug=False,
            )
            return rf.GraphRoot(
                self._metrics_pub,
                GRAPH_METRICS_PRINTING_HZ,
                name="ArmPiFPVGraph",
                debug=GRAPH_DEBUG,
            )

        # No broker configured: keep metrics off stdout and simply run the graph.
        return rf.GraphRoot(
            0.0,
            name="ArmPiFPVGraph",
            debug=GRAPH_DEBUG,
        )

    def start(self):
        self._zmq_ctx = rzmq.ZMQContext()
        self._graph = self._make_graph_root()

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

        self._servo_node = rhs.HiwonderBusServoOneShotGroupNode(
            controller,
            read_config,
            name="ArmPiServoOneShotNode",
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
        if GRAPH_METRICS_BROKER:
            self._graph.profile()
        else:
            self._graph.start_all()

        print("ArmPi-FPV hardware server running.", flush=True)
        print(f"  Camera: {CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}, device_index={CAMERA_DEVICE_INDEX}", flush=True)
        print(f"    Camera pub: {ZMQ_CAMERA_BIND}", flush=True)
        print(f"  Servo device: {SERVO_DEVICE_NAME} @ {SERVO_BAUD_RATE}", flush=True)
        print(f"    Servo state pub: {ZMQ_SERVO_STATE_BIND}", flush=True)
        print(f"    Servo cmd sub:   {ZMQ_SERVO_CMD_CONNECT}", flush=True)
        if GRAPH_METRICS_BROKER:
            print(
                f"  Graph metrics: MQTT {GRAPH_METRICS_BROKER}:{GRAPH_METRICS_PORT} topic={GRAPH_METRICS_TOPIC} @ {GRAPH_METRICS_PRINTING_HZ} Hz",
                flush=True,
            )
        else:
            print("  Graph metrics: disabled (set ARMPIFPV_GRAPH_METRICS_BROKER to enable MQTT metrics)", flush=True)

    def stop(self):
        if self._graph is not None:
            self._graph.stop()
            self._graph = None

    def _wait_for_servo_pulse(
        self,
        robot,
        joint_name: str,
        target_pulse: int,
        *,
        move_time: float,
        tolerance: int = 8,
        settle_window_s: float = 0.35,
        timeout_pad_s: float = 2.0,
        poll_s: float = 0.05,
    ) -> int | None:
        """Wait until a servo readback is near the requested pulse or timeout expires."""
        deadline = time.time() + move_time + timeout_pad_s
        settled_since = None
        last_pulse = None

        while time.time() < deadline:
            pulse = robot.servos[joint_name].get_pulse()
            last_pulse = pulse
            if pulse is not None and abs(pulse - target_pulse) <= tolerance:
                if settled_since is None:
                    settled_since = time.time()
                elif time.time() - settled_since >= settle_window_s:
                    return pulse
            else:
                settled_since = None
            time.sleep(poll_s)

        return last_pulse

    def run_self_test(self, save_frame_path: str | None = None):
        """Run a conservative local smoke test through the normal proxy path."""
        from .robot import ArmPiFPVRobotProxy

        robot = ArmPiFPVRobotProxy()
        initial_joints = None
        initial_gripper = None
        initial_base_pulse = None
        initial_servo_pulses = None
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

            # Servo state may be alive before every actuator has a valid position sample.
            deadline = time.time() + 5.0
            while time.time() < deadline:
                try:
                    initial_joints = robot.arm.get_joint_positions()
                    initial_gripper = robot.gripper.get_position()
                    initial_base_pulse = robot.servos["base_yaw"].get_pulse()
                    initial_servo_pulses = {
                        "base_yaw": robot.servos["base_yaw"].get_pulse(),
                        "shoulder": robot.servos["shoulder"].get_pulse(),
                        "elbow": robot.servos["elbow"].get_pulse(),
                        "wrist_pitch": robot.servos["wrist_pitch"].get_pulse(),
                        "wrist_roll": robot.servos["wrist_roll"].get_pulse(),
                        "gripper": robot.servos["gripper"].get_pulse(),
                    }
                    break
                except Exception:
                    time.sleep(0.1)
            else:
                raise RuntimeError("did not receive a complete initial arm/gripper/base state for self-test")

            print("Self-test: initial status", robot.is_alive(), flush=True)
            print("Self-test: initial joints", initial_joints, flush=True)
            print("Self-test: initial gripper openness", initial_gripper, flush=True)
            print("Self-test: initial gripper pulse", robot.gripper.get_raw_pulse(), flush=True)
            print("Self-test: initial base_yaw pulse", initial_base_pulse, flush=True)
            print("Self-test: initial raw servo pulses", initial_servo_pulses, flush=True)

            if save_frame_path:
                from PIL import Image

                frame = robot.cameras["main"].grab_frame(timeout=5.0)
                Image.fromarray(frame).save(save_frame_path)
                print(f"Self-test: saved frame to {save_frame_path}", flush=True)

            counts_per_deg = 1000.0 / 240.0
            delta_deg = 20.0
            delta_counts = int(round(delta_deg * counts_per_deg))

            print("Self-test: raw per-servo calibration across all arm joints...", flush=True)
            for joint_name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"):
                start_pulse = robot.servos[joint_name].get_pulse()
                pos_target = max(0, min(1000, start_pulse + delta_counts))
                neg_target = max(0, min(1000, start_pulse - delta_counts))

                print(
                    f"Self-test: {joint_name} / servo {robot.servos[joint_name].id} "
                    f"start={start_pulse} pos_target={pos_target} neg_target={neg_target}",
                    flush=True,
                )

                print(f"  {joint_name}: move BY +20deg in 1s...", flush=True)
                robot.servos[joint_name].move_to_pulse(pos_target, move_time=1.0)
                settled = self._wait_for_servo_pulse(robot, joint_name, pos_target, move_time=1.0)
                print("    measured pulse", settled, flush=True)
                print("    measured joints", robot.arm.get_joint_positions(), flush=True)

                print(f"  {joint_name}: move BY -40deg in 2s (to start-20deg)...", flush=True)
                robot.servos[joint_name].move_to_pulse(neg_target, move_time=2.0)
                settled = self._wait_for_servo_pulse(robot, joint_name, neg_target, move_time=2.0)
                print("    measured pulse", settled, flush=True)
                print("    measured joints", robot.arm.get_joint_positions(), flush=True)

                print(f"  {joint_name}: move BY +20deg back to start in 1s...", flush=True)
                robot.servos[joint_name].move_to_pulse(start_pulse, move_time=1.0)
                settled = self._wait_for_servo_pulse(robot, joint_name, start_pulse, move_time=1.0)
                print("    measured pulse", settled, flush=True)
                print("    measured joints", robot.arm.get_joint_positions(), flush=True)

            print("Self-test: restoring initial state...", flush=True)
            if initial_servo_pulses is not None:
                for joint_name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"):
                    robot.servos[joint_name].move_to_pulse(initial_servo_pulses[joint_name], move_time=1.0)
                    self._wait_for_servo_pulse(robot, joint_name, initial_servo_pulses[joint_name], move_time=1.0)
                robot.servos["gripper"].move_to_pulse(initial_servo_pulses["gripper"], move_time=0.8)
                self._wait_for_servo_pulse(robot, "gripper", initial_servo_pulses["gripper"], move_time=0.8)
            print("Self-test: final measured joints", robot.arm.get_joint_positions(), flush=True)
            print("Self-test: final measured gripper openness", robot.gripper.get_position(), flush=True)
            print("Self-test: final measured gripper pulse", robot.gripper.get_raw_pulse(), flush=True)
            for joint_name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"):
                print(f"Self-test: final measured {joint_name} pulse {robot.servos[joint_name].get_pulse()}", flush=True)
        finally:
            try:
                print("Self-test: final restore attempt before disconnect...", flush=True)
                if initial_servo_pulses is not None:
                    for joint_name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"):
                        robot.servos[joint_name].move_to_pulse(initial_servo_pulses[joint_name], move_time=1.0)
                        self._wait_for_servo_pulse(robot, joint_name, initial_servo_pulses[joint_name], move_time=1.0)
                    robot.servos["gripper"].move_to_pulse(initial_servo_pulses["gripper"], move_time=0.8)
                    self._wait_for_servo_pulse(robot, "gripper", initial_servo_pulses["gripper"], move_time=0.8)
                elif initial_base_pulse is not None:
                    robot.servos["base_yaw"].move_to_pulse(initial_base_pulse, move_time=1.0)
                    self._wait_for_servo_pulse(robot, "base_yaw", initial_base_pulse, move_time=1.0)
            except Exception:
                pass
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
