#!/usr/bin/env python3
"""PanTiltRobot — owns camera + motor hardware, publishes over ZMQ.

Usage:
    python -m robots.pantilt.hardware

ZMQ endpoints:
    See robots.pantilt.config for bind/connect endpoint values.

Ctrl+C to quit.
"""

import signal
import time
from pathlib import Path
import roboflex.webcam_gst as rcw
import roboflex.dynamixel as rfd
import roboflex.transport.zmq as rzmq
from .config import ZMQ_CAMERA_BIND, ZMQ_MOTOR_CMD_CONNECT, ZMQ_MOTOR_STATE_BIND

# --- Hardware config ---
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 30
MOTOR_DEVICE = "/dev/cu.usbserial-FT2KQC4K"
MOTOR_BAUD = 115_200
MOTOR_IDS = [5, 6]


class PanTiltRobot:
    """Owns camera + motor hardware. Publishes state and receives commands over ZMQ."""

    def __init__(self):
        self._zmq_ctx = rzmq.ZMQContext()
        self._camera = None
        self._motor_node = None
        self._motor_cmd_sub = None
        self._cam_index = None

    def start(self):
        """Discover hardware, build roboflex graph, and start all nodes."""
        # --- Find USB camera ---
        devices = rcw.get_device_list()
        self._cam_index = None
        for i, dev in enumerate(devices):
            print(f"  Camera [{i}] {dev.display_name}")
            if "USB" in dev.display_name:
                self._cam_index = i

        if self._cam_index is None:
            print("ERROR: No USB camera found.")
            raise SystemExit(1)

        # --- Camera graph ---
        self._camera = rcw.WebcamSensor(
            CAM_WIDTH, CAM_HEIGHT, CAM_FPS,
            device_index=self._cam_index, emit_rgb=True,
        )
        cam_pub = rzmq.ZMQPublisher(self._zmq_ctx, ZMQ_CAMERA_BIND, max_queued_msgs=1)
        self._camera > cam_pub

        # --- Motor graph ---
        print(
            f"  Motor serial device (configured): {MOTOR_DEVICE}",
            flush=True,
        )
        try:
            motor_controller = rfd.DynamixelGroupController.PositionController(
                device_name=MOTOR_DEVICE,
                baud_rate=MOTOR_BAUD,
                dxl_ids=MOTOR_IDS,
            )
        except Exception:
            # Helpful host-side context when the configured device path is wrong
            # or the adapter was assigned a different tty name.
            cu_candidates = sorted(str(p) for p in Path("/dev").glob("cu.*"))
            tty_candidates = sorted(str(p) for p in Path("/dev").glob("tty.*"))
            print("ERROR: Failed to open motor serial device.", flush=True)
            print(f"  Configured path: {MOTOR_DEVICE}", flush=True)
            if cu_candidates or tty_candidates:
                print("  Detected serial candidates:", flush=True)
                for p in (cu_candidates[:20] + tty_candidates[:20]):
                    print(f"    {p}", flush=True)
            else:
                print("  No /dev/cu.* or /dev/tty.* devices found.", flush=True)
            raise
        motor_controller.set_loop_sleep_ms(10)

        self._motor_node = rfd.DynamixelGroupNode(motor_controller)

        motor_state_pub = rzmq.ZMQPublisher(
            self._zmq_ctx, ZMQ_MOTOR_STATE_BIND, max_queued_msgs=1,
        )
        self._motor_cmd_sub = rzmq.ZMQSubscriber(
            self._zmq_ctx, ZMQ_MOTOR_CMD_CONNECT, max_queued_msgs=1,
        )

        self._motor_node > motor_state_pub
        self._motor_cmd_sub > self._motor_node

        # --- Start nodes ---
        self._camera.start()
        self._motor_cmd_sub.start()
        self._motor_node.start()

        dev = devices[self._cam_index]
        print(f"\nHardware server running.", flush=True)
        print(
            f"  Camera: [{self._cam_index}] {dev.display_name}, "
            f"{CAM_WIDTH}x{CAM_HEIGHT}@{CAM_FPS}fps -> {ZMQ_CAMERA_BIND}",
            flush=True,
        )
        print(f"  Motors: IDs {MOTOR_IDS} on {MOTOR_DEVICE} @ {MOTOR_BAUD}", flush=True)
        print(f"    State:    {ZMQ_MOTOR_STATE_BIND}", flush=True)
        print(f"    Commands: {ZMQ_MOTOR_CMD_CONNECT}", flush=True)
        print("Ctrl+C to quit.", flush=True)

    def stop(self):
        """Stop all nodes and release hardware."""
        print("\nShutting down...", flush=True)
        if self._camera:
            self._camera.stop()
        if self._motor_cmd_sub:
            self._motor_cmd_sub.stop()
        time.sleep(0.1)
        if self._motor_node:
            self._motor_node.stop()
        print("Done.")


if __name__ == "__main__":
    robot = PanTiltRobot()
    robot.start()

    _shutdown = False

    def _on_sigint(sig, frame):
        del sig, frame
        global _shutdown
        _shutdown = True

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        while not _shutdown:
            time.sleep(0.1)
    finally:
        robot.stop()
