"""PanTiltRobotProxy — the `robot` object available in the REPL.

Thin ZMQ proxy that communicates with a running PanTiltRobot (hardware.py).
All motor commands are safety-clamped before publishing.

Usage:
    from robots.pantilt.robot import PanTiltRobotProxy
    robot = PanTiltRobotProxy()
    robot.connect()

    frame = robot.cameras['main'].grab_frame()
    pos = robot.motors['pan'].get_position()
    robot.motors['pan'].set_position(2048)
    robot.close()
"""

import io
import time
import base64
import threading
import numpy as np
import roboflex.dynamixel as rfd
import roboflex.transport.zmq as rzmq
import roboflex.webcam_gst as rcw
from roboflex import CallbackFun

from server.robot_base import RobotBase
from .config import (
    MOTOR_LIMITS,
    TICKS_PER_DEG,
    ZMQ_CAMERA_CONNECT,
    ZMQ_MOTOR_CMD_BIND,
    ZMQ_MOTOR_STATE_CONNECT,
)


class CameraProxy:
    """Proxy for a camera stream over ZMQ."""

    def __init__(self, name, zmq_ctx, endpoint):
        self.name = name
        self._endpoint = endpoint
        self._latest_frame = None
        self._last_recv = None  # time.time() of last frame
        self._lock = threading.Lock()

        self._sub = rzmq.ZMQSubscriber(zmq_ctx, endpoint, max_queued_msgs=1)
        self._cb = CallbackFun(self._on_frame)
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

    def grab_frame(self, timeout=5.0):
        """Return latest frame as numpy (H,W,3) uint8. Waits up to timeout for first frame."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._latest_frame is not None:
                    return self._latest_frame.copy()
            time.sleep(0.01)
        raise TimeoutError(f"No frame received from '{self.name}' within {timeout}s")


class MotorProxy:
    """Proxy for a single motor, reading state from ZMQ and publishing commands."""

    def __init__(self, motor_id, limits, state_store, cmd_publisher):
        self.id = motor_id
        self.name = limits['name']
        self._limits = limits
        self._state_store = state_store  # shared dict updated by _MotorStateReceiver
        self._cmd_pub = cmd_publisher     # shared ZMQ publisher

    def get_position(self):
        """Current position in ticks (int)."""
        state = self._state_store.get(self.id)
        if state is None:
            raise RuntimeError(f"No state received for motor {self.id} ('{self.name}')")
        return state.get('position')

    def set_position(self, value):
        """Set goal position (ticks). Clamped to safety limits."""
        clamped = max(self._limits['min'], min(self._limits['max'], int(value)))
        command = rfd.DynamixelGroupCommand()
        command.set(self.id, rfd.DXLControlTable.GoalPosition, clamped)
        msg = rfd.DynamixelGroupCommandMessage(command)
        self._cmd_pub.signal_self(msg)
        return clamped

    def nudge(self, delta):
        """Move by delta ticks from current position. Returns clamped target."""
        current = self.get_position()
        return self.set_position(current + delta)

    def get_limits(self):
        """Return (min, max) position limits in ticks."""
        return (self._limits['min'], self._limits['max'])


class _MotorStateReceiver:
    """Receives motor state over ZMQ and stores latest values per motor."""

    def __init__(self, zmq_ctx, endpoint, motor_ids):
        self._motor_ids = motor_ids
        self._state = {}  # {motor_id: {'position': int, 'timestamp': float}}
        self._last_recv = None  # time.time() of last state message
        self._lock = threading.Lock()
        self._loggers = []  # list of logging callbacks

        self._sub = rzmq.ZMQSubscriber(zmq_ctx, endpoint, max_queued_msgs=1)
        self._cb = CallbackFun(self._on_state)
        self._sub > self._cb

    def _on_state(self, msg):
        state = rfd.DynamixelGroupStateMessage(msg).state
        t = state.timestamp.t0
        with self._lock:
            self._last_recv = time.time()
            for mid in self._motor_ids:
                vals = state.values[mid]
                self._state[mid] = {
                    'position': vals[rfd.DXLControlTable.PresentPosition],
                    'timestamp': t,
                }
            # Notify loggers
            snapshot = dict(self._state)
        for logger in self._loggers:
            logger._on_snapshot(snapshot)

    def get(self, motor_id):
        with self._lock:
            return self._state.get(motor_id)

    def start(self):
        self._sub.start()

    def stop(self):
        self._sub.stop()

    def add_logger(self, logger):
        self._loggers.append(logger)

    def remove_logger(self, logger):
        self._loggers.remove(logger)


class DataLogger:
    """Accumulates motor state snapshots for later analysis."""

    def __init__(self, motor_ids):
        self._motor_ids = motor_ids
        self._data = []  # list of (timestamp, {mid: position, ...})
        self._lock = threading.Lock()
        self._active = True

    def _on_snapshot(self, snapshot):
        if not self._active:
            return
        with self._lock:
            entry = {'t': snapshot[self._motor_ids[0]]['timestamp']}
            for mid in self._motor_ids:
                entry[mid] = snapshot[mid]['position']
            self._data.append(entry)

    def get_data(self):
        """Return logged data as dict of numpy arrays.

        Returns:
            {'t': array, 5: array, 6: array, ...}
        """
        with self._lock:
            if not self._data:
                return {}
            result = {'t': np.array([d['t'] for d in self._data])}
            for mid in self._motor_ids:
                result[mid] = np.array([d[mid] for d in self._data])
            return result

    def stop(self):
        self._active = False

    def clear(self):
        with self._lock:
            self._data.clear()


class PanTiltRobotProxy(RobotBase):
    """Pan/tilt robot proxy. Connects to PanTiltRobot (hardware.py) over ZMQ."""

    def __init__(self):
        super().__init__()
        self._zmq_ctx = rzmq.ZMQContext()
        self._connected = False

        # Will be populated by connect()
        self.cameras = {}
        self.motors = {}
        self._camera_proxies = []
        self._motor_state_receiver = None
        self._cmd_pub = None

    def connect(self):
        """Connect to hardware_server ZMQ endpoints."""
        if self._connected:
            return

        # Camera
        cam = CameraProxy('main', self._zmq_ctx, ZMQ_CAMERA_CONNECT)
        cam.start()
        self.cameras['main'] = cam
        self._camera_proxies.append(cam)

        # Motor command publisher (we bind, server's sub connects to us)
        self._cmd_pub = rzmq.ZMQPublisher(self._zmq_ctx, ZMQ_MOTOR_CMD_BIND, max_queued_msgs=1)

        # Force the publisher to bind NOW by sending a no-op command.
        # ZMQPublisher lazily binds on first receive(), so without this
        # the socket isn't bound until the first set_position() call,
        # and the hardware_server's subscriber misses early messages
        # (ZMQ "slow joiner" problem).
        noop = rfd.DynamixelGroupCommandMessage(rfd.DynamixelGroupCommand())
        self._cmd_pub.signal_self(noop)

        # Motor state receiver
        motor_ids = list(MOTOR_LIMITS.keys())
        self._motor_state_receiver = _MotorStateReceiver(
            self._zmq_ctx, ZMQ_MOTOR_STATE_CONNECT, motor_ids
        )
        self._motor_state_receiver.start()

        # Create motor proxies
        for mid, limits in MOTOR_LIMITS.items():
            proxy = MotorProxy(mid, limits, self._motor_state_receiver, self._cmd_pub)
            self.motors[limits['name']] = proxy

        # Give ZMQ connections time to establish (subscriber must detect
        # our newly-bound publisher and connect)
        time.sleep(1.0)
        self._connected = True

    def close(self):
        """Disconnect from hardware_server and stop background activity.

        Safe to call multiple times.
        """
        try:
            self.stop_all_tasks(timeout=1.0)
        except Exception:
            pass

        if self._connected:
            for cam in self._camera_proxies:
                try:
                    cam.stop()
                except Exception:
                    pass
            if self._motor_state_receiver:
                try:
                    self._motor_state_receiver.stop()
                except Exception:
                    pass
            self._connected = False

        self.cameras = {}
        self.motors = {}
        self._camera_proxies = []
        self._motor_state_receiver = None
        self._cmd_pub = None
        self._tasks = {}

    def is_alive(self, max_age=2.0):
        """Check if hardware_server is actively sending data.

        Returns dict with per-subsystem status. A subsystem is 'alive' if
        we received data from it within the last max_age seconds.
        """
        now = time.time()
        cam_alive = False
        for cam in self._camera_proxies:
            if cam._last_recv is not None and (now - cam._last_recv) < max_age:
                cam_alive = True
        motor_alive = False
        if self._motor_state_receiver and self._motor_state_receiver._last_recv is not None:
            if (now - self._motor_state_receiver._last_recv) < max_age:
                motor_alive = True
        return {
            'camera': cam_alive,
            'motors': motor_alive,
            'all': cam_alive and motor_alive,
        }

    def set_positions(self, **kwargs):
        """Set multiple motors simultaneously in a single command.

        Args:
            **kwargs: motor_name=position pairs, e.g. pan=1500, tilt=2800

        Returns:
            dict of {name: clamped_position} actually sent.
        """
        command = rfd.DynamixelGroupCommand()
        result = {}
        for name, value in kwargs.items():
            motor = self.motors.get(name)
            if motor is None:
                raise ValueError(f"Unknown motor '{name}'. Available: {list(self.motors.keys())}")
            clamped = max(motor._limits['min'], min(motor._limits['max'], int(value)))
            command.set(motor.id, rfd.DXLControlTable.GoalPosition, clamped)
            result[name] = clamped
        msg = rfd.DynamixelGroupCommandMessage(command)
        self._cmd_pub.signal_self(msg)
        return result

    def snapshot(self, device_name):
        """Grab latest state from a named device.

        For cameras: returns {'type': 'image', 'format': 'jpeg', 'data': '<base64>', ...}
        For motors:  returns {'type': 'state', 'name': ..., 'position': ..., 'limits': ...}
        """
        if device_name in self.cameras:
            from PIL import Image
            frame = self.cameras[device_name].grab_frame()
            buf = io.BytesIO()
            Image.fromarray(frame).save(buf, format='JPEG', quality=55)
            buf.seek(0)
            return {
                'type': 'image',
                'format': 'jpeg',
                'data': base64.b64encode(buf.read()).decode('ascii'),
                'width': frame.shape[1],
                'height': frame.shape[0],
            }
        elif device_name in self.motors:
            motor = self.motors[device_name]
            return {
                'type': 'state',
                'name': motor.name,
                'id': motor.id,
                'position': motor.get_position(),
                'limits': motor.get_limits(),
            }
        else:
            available = list(self.cameras.keys()) + list(self.motors.keys())
            raise ValueError(f"Unknown device '{device_name}'. Available: {available}")

    def _device_docs(self):
        """Pan/tilt-specific hardware documentation."""
        lines = []
        lines.append("# Robot: Pan/Tilt Camera Mount")
        lines.append("Two Dynamixel servo motors controlling a USB camera on a pan/tilt bracket.")
        lines.append("")

        # Connection status
        status = self.is_alive()
        if status['all']:
            lines.append("**Status: CONNECTED** (camera and motors active)")
        else:
            parts = []
            if not status['camera']:
                parts.append("camera: no data")
            if not status['motors']:
                parts.append("motors: no data")
            lines.append(f"**Status: HARDWARE OFFLINE** ({', '.join(parts)})")
            lines.append("Is hardware_server.py running?")
        lines.append("")

        # Devices
        lines.append("## Devices")
        lines.append("")
        for name, cam in self.cameras.items():
            lines.append(f"### Camera: '{name}'")
            lines.append(f"  USB webcam, 640x480 @ 30fps, RGB")
            lines.append(f"  snapshot('{name}') → returns a live image")
            lines.append(f"  robot.cameras['{name}'].grab_frame() → numpy (480, 640, 3) uint8")
            lines.append("")
        for name, motor in self.motors.items():
            limits = motor._limits
            deg_below = (limits['center'] - limits['min']) / TICKS_PER_DEG
            deg_above = (limits['max'] - limits['center']) / TICKS_PER_DEG
            pos = None
            try:
                pos = motor.get_position()
            except Exception:
                pass
            lines.append(f"### Motor: '{name}' (Dynamixel XH430-V350, ID {motor.id})")
            if name == 'pan':
                lines.append(f"  Range: ±{deg_below:.0f}° from center")
                lines.append(f"  Note: increasing ticks = pan LEFT, decreasing ticks = pan RIGHT (from camera's perspective)")
            elif name == 'tilt':
                lines.append(f"  Range: +{deg_below:.0f}° (ceiling) to -{deg_above:.0f}° (floor) from horizontal")
                lines.append(f"  Note: decreasing ticks = toward ceiling, increasing ticks = toward floor")
            lines.append(f"  Position range: {limits['min']}–{limits['max']} ticks (center={limits['center']})")
            lines.append(f"  Resolution: {TICKS_PER_DEG:.1f} ticks/degree")
            if pos is not None:
                deg_from_center = (pos - limits['center']) / TICKS_PER_DEG
                lines.append(f"  Current position: {pos} ticks ({deg_from_center:+.1f}° from center)")
            lines.append(f"  snapshot('{name}') → returns position and limits")
            lines.append("")

        # Device-specific API reference
        lines.append("## robot API Reference")
        lines.append("")
        lines.append("### Camera")
        lines.append("  robot.cameras['main'].grab_frame()        → numpy (H,W,3) uint8, RGB")
        lines.append("  Note: frames are RGB. Convert explicitly if using OpenCV code that assumes BGR.")
        lines.append("")
        lines.append("### Motors")
        lines.append("  robot.motors['pan'].get_position()         → int (ticks)")
        lines.append("  robot.motors['pan'].set_position(ticks)    → int (clamped to safety limits)")
        lines.append("  robot.motors['pan'].nudge(delta_ticks)     → int (relative move from current)")
        lines.append("  robot.motors['pan'].get_limits()           → (min, max) ticks")
        lines.append("  Same for 'tilt'.")
        lines.append("")
        lines.append("  **Coordinated motion (IMPORTANT):**")
        lines.append("  robot.set_positions(pan=1500, tilt=2800)   → move both motors at once")
        lines.append("  Always use set_positions() when moving multiple motors together.")
        lines.append("  Separate set_position() calls can race — the second may overwrite the first.")
        lines.append("")
        lines.append("### Data Logging")
        lines.append("  logger = robot.start_logging()             → starts recording motor state")
        lines.append("  robot.stop_logging(logger)")
        lines.append("  data = logger.get_data()                   → {'t': array, 5: array, 6: array}")
        lines.append("")
        lines.append("### Introspection")
        lines.append("  robot.describe()                           → this text")
        lines.append("  robot.snapshot('device_name')              → image or state dict")
        lines.append("  robot.is_alive()                           → {'camera': bool, 'motors': bool, 'all': bool}")
        lines.append("")

        # Available libraries
        lines.append("## Available Python Libraries")
        lines.append("  numpy, PIL/Pillow, matplotlib, cv2 (OpenCV, headless), math, time")
        lines.append("  All imports persist across execute() calls.")
        lines.append("")

        # Tips
        lines.append("## Tips")
        lines.append("- Call snapshot('main') to see what the camera sees before writing code.")
        lines.append("- Keep data on-device: analyze with numpy, return plots via robot.show(fig).")
        lines.append("- The REPL is persistent: define helper functions, build up state incrementally.")
        lines.append("- Motor commands are safety-clamped — you cannot exceed joint limits.")
        lines.append("- If code crashes, the hardware is fine. Fix your code and re-run.")
        lines.append("- Use robot.run(fn, duration, hz) for short control loops (< 30s).")
        lines.append("- Use robot.start_task(name, fn) for longer-running behaviors. Stop with robot.stop_task(name).")
        lines.append("- If background behavior is not doing what you expect, inspect robot.list_tasks() or robot.task_status(name).")
        lines.append("")

        return '\n'.join(lines)

    # ── Data logging (pan-tilt-specific) ─────────────────────────────

    def start_logging(self):
        """Start logging motor state. Returns a DataLogger."""
        motor_ids = list(MOTOR_LIMITS.keys())
        logger = DataLogger(motor_ids)
        self._motor_state_receiver.add_logger(logger)
        return logger

    def stop_logging(self, logger):
        """Remove a logger from the motor state receiver."""
        self._motor_state_receiver.remove_logger(logger)
        logger.stop()
