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

from . import kinematics
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
    GRIPPER_CLOSED_PULSE,
    GRIPPER_OPEN_PULSE,
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
        self._controller = None
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

    def start(self, *, enable_servo_graph: bool = True):
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
        self._controller = controller
        controller.set_loop_sleep_ms(SERVO_LOOP_SLEEP_MS)

        if enable_servo_graph:
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
        if enable_servo_graph:
            print(f"    Servo state pub: {ZMQ_SERVO_STATE_BIND}", flush=True)
            print(f"    Servo cmd sub:   {ZMQ_SERVO_CMD_CONNECT}", flush=True)
        else:
            print("    Servo graph: disabled (direct local controller mode)", flush=True)
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

    def _gripper_pulse_to_openness(self, pulse: int | float | None) -> float:
        if pulse is None:
            raise RuntimeError("gripper pulse is not available yet")
        denom = GRIPPER_OPEN_PULSE - GRIPPER_CLOSED_PULSE
        if denom == 0:
            return 0.0
        openness = (float(pulse) - GRIPPER_CLOSED_PULSE) / denom
        return min(max(openness, 0.0), 1.0)

    def _direct_read_pulse(self, name: str) -> int:
        if self._controller is None:
            raise RuntimeError("controller is not initialized")
        servo_id = {"base_yaw": 6, "shoulder": 5, "elbow": 4, "wrist_pitch": 3, "wrist_roll": 2, "gripper": 1}[name]
        pulse = self._controller.read_position(servo_id)
        if pulse is None:
            raise RuntimeError(f"no direct pulse available for '{name}'")
        return int(pulse)

    def _direct_read_servo_pulses(self) -> dict[str, int]:
        return {
            "base_yaw": self._direct_read_pulse("base_yaw"),
            "shoulder": self._direct_read_pulse("shoulder"),
            "elbow": self._direct_read_pulse("elbow"),
            "wrist_pitch": self._direct_read_pulse("wrist_pitch"),
            "wrist_roll": self._direct_read_pulse("wrist_roll"),
            "gripper": self._direct_read_pulse("gripper"),
        }

    def _direct_arm_joints(self) -> dict[str, float]:
        pulses = self._direct_read_servo_pulses()
        return kinematics.arm_pulses_to_joints({name: pulses[name] for name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll")})

    def _direct_fk_pose(self) -> dict:
        return kinematics.fk_pose(self._direct_arm_joints())

    def _direct_write_positions(self, positions: dict[str, int], move_time: float):
        if self._controller is None:
            raise RuntimeError("controller is not initialized")
        by_id = {
            {"base_yaw": 6, "shoulder": 5, "elbow": 4, "wrist_pitch": 3, "wrist_roll": 2, "gripper": 1}[name]: int(round(min(max(pulse, 0), 1000)))
            for name, pulse in positions.items()
        }
        self._controller.write_positions_ms(int(round(max(move_time, 0.01) * 1000.0)), by_id)

    def _wait_for_direct_servo_pulse(
        self,
        joint_name: str,
        target_pulse: int,
        *,
        move_time: float,
        tolerance: int = 8,
        settle_window_s: float = 0.35,
        timeout_pad_s: float = 2.0,
        poll_s: float = 0.1,
    ) -> int | None:
        deadline = time.time() + move_time + timeout_pad_s
        settled_since = None
        last_pulse = None
        while time.time() < deadline:
            pulse = self._direct_read_pulse(joint_name)
            last_pulse = pulse
            if abs(pulse - target_pulse) <= tolerance:
                if settled_since is None:
                    settled_since = time.time()
                elif time.time() - settled_since >= settle_window_s:
                    return pulse
            else:
                settled_since = None
            time.sleep(poll_s)
        return last_pulse

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
        poll_s: float = 0.1,
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
            print("Self-test: initial FK pose", robot.arm.get_pose(), flush=True)
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

            speed = 0.3
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

                print(f"  {joint_name}: move BY +20deg in {speed}s...", flush=True)
                robot.servos[joint_name].move_to_pulse(pos_target, move_time=speed)
                settled = self._wait_for_servo_pulse(robot, joint_name, pos_target, move_time=2.0*speed)
                print("    measured pulse", settled, flush=True)
                print("    measured joints", robot.arm.get_joint_positions(), flush=True)
                print("    measured FK pose", robot.arm.get_pose(), flush=True)

                print(f"  {joint_name}: move BY -40deg in {2.0*speed}s (to start-20deg)...", flush=True)
                robot.servos[joint_name].move_to_pulse(neg_target, move_time=2.0*speed)
                settled = self._wait_for_servo_pulse(robot, joint_name, neg_target, move_time=4.0*speed)
                print("    measured pulse", settled, flush=True)
                print("    measured joints", robot.arm.get_joint_positions(), flush=True)
                print("    measured FK pose", robot.arm.get_pose(), flush=True)

                print(f"  {joint_name}: move BY +20deg back to start in {speed}s...", flush=True)
                robot.servos[joint_name].move_to_pulse(start_pulse, move_time=speed)
                settled = self._wait_for_servo_pulse(robot, joint_name, start_pulse, move_time=2.0*speed)
                print("    measured pulse", settled, flush=True)
                print("    measured joints", robot.arm.get_joint_positions(), flush=True)
                print("    measured FK pose", robot.arm.get_pose(), flush=True)

            gripper_start = robot.servos["gripper"].get_pulse()
            gripper_open_target = int(min(gripper_start, GRIPPER_OPEN_PULSE))
            gripper_closed_target = int(max(gripper_start, GRIPPER_CLOSED_PULSE))
            print(
                "Self-test: gripper / servo "
                f"{robot.servos['gripper'].id} start={gripper_start} "
                f"open_target={gripper_open_target} closed_target={gripper_closed_target}",
                flush=True,
            )

            print("  gripper: open in 0.8s...", flush=True)
            robot.servos["gripper"].move_to_pulse(gripper_open_target, move_time=0.8)
            settled = self._wait_for_servo_pulse(robot, "gripper", gripper_open_target, move_time=0.8)
            print("    measured pulse", settled, flush=True)
            print("    measured openness", robot.gripper.get_position(), flush=True)
            print("    measured FK pose", robot.arm.get_pose(), flush=True)

            print("  gripper: close in 0.8s...", flush=True)
            robot.servos["gripper"].move_to_pulse(gripper_closed_target, move_time=0.8)
            settled = self._wait_for_servo_pulse(robot, "gripper", gripper_closed_target, move_time=0.8)
            print("    measured pulse", settled, flush=True)
            print("    measured openness", robot.gripper.get_position(), flush=True)
            print("    measured FK pose", robot.arm.get_pose(), flush=True)

            print("  gripper: return to start in 0.8s...", flush=True)
            robot.servos["gripper"].move_to_pulse(gripper_start, move_time=0.8)
            settled = self._wait_for_servo_pulse(robot, "gripper", gripper_start, move_time=0.8)
            print("    measured pulse", settled, flush=True)
            print("    measured openness", robot.gripper.get_position(), flush=True)
            print("    measured FK pose", robot.arm.get_pose(), flush=True)

            combined_start_pulses = {
                "base_yaw": robot.servos["base_yaw"].get_pulse(),
                "shoulder": robot.servos["shoulder"].get_pulse(),
                "elbow": robot.servos["elbow"].get_pulse(),
                "wrist_pitch": robot.servos["wrist_pitch"].get_pulse(),
                "wrist_roll": robot.servos["wrist_roll"].get_pulse(),
                "gripper": robot.servos["gripper"].get_pulse(),
            }
            combined_pos_targets = {
                name: max(0, min(1000, pulse + delta_counts))
                for name, pulse in combined_start_pulses.items()
                if name != "gripper"
            }
            combined_neg_targets = {
                name: max(0, min(1000, pulse - delta_counts))
                for name, pulse in combined_start_pulses.items()
                if name != "gripper"
            }
            combined_pos_targets["gripper"] = max(0, min(1000, combined_start_pulses["gripper"] + delta_counts))
            combined_neg_targets["gripper"] = max(0, min(1000, combined_start_pulses["gripper"] - delta_counts))

            print("Self-test: coordinated whole-arm + gripper motion...", flush=True)
            print("  combined start pulses", combined_start_pulses, flush=True)
            print("  combined +20deg/+delta targets", combined_pos_targets, flush=True)
            print("  combined -20deg/-delta targets", combined_neg_targets, flush=True)

            print("  combined: move all BY +20deg/+delta in 1s...", flush=True)
            robot.arm.move_joints(
                {
                    name: kinematics.pulse_to_radians(name, combined_pos_targets[name])
                    for name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll")
                },
                move_time=1.0,
            )
            robot.servos["gripper"].move_to_pulse(combined_pos_targets["gripper"], move_time=1.0)
            time.sleep(1.2)
            print("    measured arm pulses", {
                name: robot.servos[name].get_pulse()
                for name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll")
            }, flush=True)
            print("    measured gripper pulse", robot.servos["gripper"].get_pulse(), flush=True)
            print("    measured joints", robot.arm.get_joint_positions(), flush=True)
            print("    measured FK pose", robot.arm.get_pose(), flush=True)

            print("  combined: move all BY -40deg/-2delta in 2s (to start-20deg)...", flush=True)
            robot.arm.move_joints(
                {
                    name: kinematics.pulse_to_radians(name, combined_neg_targets[name])
                    for name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll")
                },
                move_time=2.0,
            )
            robot.servos["gripper"].move_to_pulse(combined_neg_targets["gripper"], move_time=2.0)
            time.sleep(2.2)
            print("    measured arm pulses", {
                name: robot.servos[name].get_pulse()
                for name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll")
            }, flush=True)
            print("    measured gripper pulse", robot.servos["gripper"].get_pulse(), flush=True)
            print("    measured joints", robot.arm.get_joint_positions(), flush=True)
            print("    measured FK pose", robot.arm.get_pose(), flush=True)

            print("  combined: move all BY +20deg/+delta back to start in 1s...", flush=True)
            robot.arm.move_joints(
                {
                    name: kinematics.pulse_to_radians(name, combined_start_pulses[name])
                    for name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll")
                },
                move_time=1.0,
            )
            robot.servos["gripper"].move_to_pulse(combined_start_pulses["gripper"], move_time=1.0)
            time.sleep(1.2)
            print("    measured arm pulses", {
                name: robot.servos[name].get_pulse()
                for name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll")
            }, flush=True)
            print("    measured gripper pulse", robot.servos["gripper"].get_pulse(), flush=True)
            print("    measured joints", robot.arm.get_joint_positions(), flush=True)
            print("    measured FK pose", robot.arm.get_pose(), flush=True)

            print("Self-test: FK/IK diagnostics on live state...", flush=True)
            live_joints = robot.arm.get_joint_positions()
            live_pose = robot.arm.get_pose()
            print("  live joints", live_joints, flush=True)
            print("  live pose", live_pose, flush=True)
            ik_result = robot.arm.ik(
                live_pose["position"],
                live_pose["orientation"],
                prefer_current=True,
                allow_approx=True,
            )
            print("  ik(current_pose) ->", ik_result, flush=True)

            print("Self-test: restoring initial state...", flush=True)
            if initial_servo_pulses is not None:
                for joint_name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"):
                    robot.servos[joint_name].move_to_pulse(initial_servo_pulses[joint_name], move_time=1.0)
                    self._wait_for_servo_pulse(robot, joint_name, initial_servo_pulses[joint_name], move_time=1.0)
                robot.servos["gripper"].move_to_pulse(initial_servo_pulses["gripper"], move_time=0.8)
                self._wait_for_servo_pulse(robot, "gripper", initial_servo_pulses["gripper"], move_time=0.8)
            print("Self-test: final measured joints", robot.arm.get_joint_positions(), flush=True)
            print("Self-test: final FK pose", robot.arm.get_pose(), flush=True)
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

    def run_self_test_direct(self):
        """Run the same smoke test shape, but talk straight to the local controller."""
        if self._controller is None:
            raise RuntimeError("controller is not initialized")

        initial_servo_pulses = self._direct_read_servo_pulses()
        initial_joints = self._direct_arm_joints()
        print("Direct self-test: initial joints", initial_joints, flush=True)
        print("Direct self-test: initial FK pose", self._direct_fk_pose(), flush=True)
        print("Direct self-test: initial gripper openness", self._gripper_pulse_to_openness(initial_servo_pulses["gripper"]), flush=True)
        print("Direct self-test: initial raw servo pulses", initial_servo_pulses, flush=True)

        counts_per_deg = 1000.0 / 240.0
        delta_counts = int(round(20.0 * counts_per_deg))
        speed = 0.3

        try:
            print("Direct self-test: raw per-servo calibration across all arm joints...", flush=True)
            for joint_name in ("base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"):
                start_pulse = self._direct_read_pulse(joint_name)
                pos_target = max(0, min(1000, start_pulse + delta_counts))
                neg_target = max(0, min(1000, start_pulse - delta_counts))
                print(f"Direct self-test: {joint_name} start={start_pulse} pos_target={pos_target} neg_target={neg_target}", flush=True)

                print(f"  {joint_name}: move BY +20deg...", flush=True)
                self._direct_write_positions({joint_name: pos_target}, speed)
                print("    measured pulse", self._wait_for_direct_servo_pulse(joint_name, pos_target, move_time=2.0 * speed), flush=True)
                print("    measured joints", self._direct_arm_joints(), flush=True)
                print("    measured FK pose", self._direct_fk_pose(), flush=True)

                print(f"  {joint_name}: move BY -40deg...", flush=True)
                self._direct_write_positions({joint_name: neg_target}, 2.0 * speed)
                print("    measured pulse", self._wait_for_direct_servo_pulse(joint_name, neg_target, move_time=4.0 * speed), flush=True)
                print("    measured joints", self._direct_arm_joints(), flush=True)
                print("    measured FK pose", self._direct_fk_pose(), flush=True)

                print(f"  {joint_name}: move BY +20deg back to start...", flush=True)
                self._direct_write_positions({joint_name: start_pulse}, speed)
                print("    measured pulse", self._wait_for_direct_servo_pulse(joint_name, start_pulse, move_time=2.0 * speed), flush=True)
                print("    measured joints", self._direct_arm_joints(), flush=True)
                print("    measured FK pose", self._direct_fk_pose(), flush=True)

            gripper_start = self._direct_read_pulse("gripper")
            gripper_open_target = int(min(gripper_start, GRIPPER_OPEN_PULSE))
            gripper_closed_target = int(max(gripper_start, GRIPPER_CLOSED_PULSE))
            print(f"Direct self-test: gripper start={gripper_start} open_target={gripper_open_target} closed_target={gripper_closed_target}", flush=True)
            for label, target in (("open", gripper_open_target), ("close", gripper_closed_target), ("return", gripper_start)):
                self._direct_write_positions({"gripper": target}, 0.8)
                print(f"  gripper: {label}...", flush=True)
                print("    measured pulse", self._wait_for_direct_servo_pulse("gripper", target, move_time=0.8), flush=True)
                print("    measured openness", self._gripper_pulse_to_openness(self._direct_read_pulse("gripper")), flush=True)
                print("    measured FK pose", self._direct_fk_pose(), flush=True)

            combined_start = self._direct_read_servo_pulses()
            combined_pos = {name: max(0, min(1000, pulse + delta_counts)) for name, pulse in combined_start.items()}
            combined_neg = {name: max(0, min(1000, pulse - delta_counts)) for name, pulse in combined_start.items()}
            print("Direct self-test: coordinated whole-arm + gripper motion...", flush=True)
            print("  combined start pulses", combined_start, flush=True)
            print("  combined +delta targets", combined_pos, flush=True)
            print("  combined -delta targets", combined_neg, flush=True)

            self._direct_write_positions(combined_pos, 1.0)
            time.sleep(1.2)
            print("    measured arm pulses", self._direct_read_servo_pulses(), flush=True)
            print("    measured joints", self._direct_arm_joints(), flush=True)
            print("    measured FK pose", self._direct_fk_pose(), flush=True)

            self._direct_write_positions(combined_neg, 2.0)
            time.sleep(2.2)
            print("    measured arm pulses", self._direct_read_servo_pulses(), flush=True)
            print("    measured joints", self._direct_arm_joints(), flush=True)
            print("    measured FK pose", self._direct_fk_pose(), flush=True)

            self._direct_write_positions(combined_start, 1.0)
            time.sleep(1.2)
            print("    measured arm pulses", self._direct_read_servo_pulses(), flush=True)
            print("    measured joints", self._direct_arm_joints(), flush=True)
            print("    measured FK pose", self._direct_fk_pose(), flush=True)

            live_joints = self._direct_arm_joints()
            live_pose = kinematics.fk_pose(live_joints)
            print("Direct self-test: FK/IK diagnostics on live state...", flush=True)
            print("  live joints", live_joints, flush=True)
            print("  live pose", live_pose, flush=True)
            print("  ik(current_pose) ->", kinematics.solve_ik(live_pose["position"], live_pose["orientation"], initial_joints=live_joints, prefer_current=True, allow_approx=True), flush=True)

            print("Direct self-test: restoring initial state...", flush=True)
            self._direct_write_positions(initial_servo_pulses, 1.0)
            time.sleep(1.2)
            print("Direct self-test: final measured joints", self._direct_arm_joints(), flush=True)
            print("Direct self-test: final FK pose", self._direct_fk_pose(), flush=True)
            print("Direct self-test: final measured gripper openness", self._gripper_pulse_to_openness(self._direct_read_pulse("gripper")), flush=True)
            print("Direct self-test: final raw servo pulses", self._direct_read_servo_pulses(), flush=True)
        finally:
            try:
                print("Direct self-test: final restore attempt before exit...", flush=True)
                self._direct_write_positions(initial_servo_pulses, 1.0)
                time.sleep(1.2)
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
        "--self-test-direct",
        action="store_true",
        help="run the same servo smoke test directly against the local controller, bypassing the Ariel proxy/ZMQ servo state path",
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
    robot.start(enable_servo_graph=not args.self_test_direct)

    try:
        if args.self_test_direct:
            robot.run_self_test_direct()
        elif args.self_test:
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
