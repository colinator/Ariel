"""ArmPi-FPV robot configuration."""

from __future__ import annotations

import os

ARM_JOINT_NAMES = (
    "base_yaw",
    "shoulder",
    "elbow",
    "wrist_pitch",
    "wrist_roll",
)
ALL_ACTUATOR_NAMES = ARM_JOINT_NAMES + ("gripper",)

SERVO_ID_BY_NAME = {
    "gripper": 1,
    "wrist_roll": 2,
    "wrist_pitch": 3,
    "elbow": 4,
    "shoulder": 5,
    "base_yaw": 6,
}
NAME_BY_SERVO_ID = {servo_id: name for name, servo_id in SERVO_ID_BY_NAME.items()}
ARM_SERVO_IDS = [SERVO_ID_BY_NAME[name] for name in ARM_JOINT_NAMES]
ALL_SERVO_IDS = [SERVO_ID_BY_NAME[name] for name in ALL_ACTUATOR_NAMES]

# Pulse-domain mappings inferred from vendor transform.py:
# [pulse_min, pulse_max, pulse_center, angle_deg_min, angle_deg_max, angle_deg_center]
PULSE_MAP_BY_NAME = {
    "base_yaw": [0, 1000, 500, -120.0, 120.0, 0.0],
    "shoulder": [0, 1000, 500, 30.0, -210.0, -90.0],
    "elbow": [0, 1000, 500, -120.0, 120.0, 0.0],
    "wrist_pitch": [0, 1000, 500, 30.0, -210.0, -90.0],
    "wrist_roll": [0, 1000, 500, -120.0, 120.0, 0.0],
}

JOINT_LIMITS_DEG = {
    "base_yaw": (-120.2, 120.2),
    "shoulder": (-180.2, 0.2),
    "elbow": (-120.2, 120.2),
    "wrist_pitch": (-200.2, 20.2),
    "wrist_roll": (-120.2, 120.2),
}

HOME_ARM_PULSES = {
    "base_yaw": 500,
    "shoulder": 500,
    "elbow": 500,
    "wrist_pitch": 500,
    "wrist_roll": 500,
}

STOW_ARM_PULSES = {
    "base_yaw": 500,
    "shoulder": 625,
    "elbow": 825,
    "wrist_pitch": 80,
    "wrist_roll": 500,
}

GRIPPER_PULSE_MIN = 0
GRIPPER_PULSE_MAX = 1000
GRIPPER_OPEN_PULSE = 200
GRIPPER_CLOSED_PULSE = 570
GRIPPER_NEUTRAL_PULSE = 700

BASE_HEIGHT_M = 0.064605
LINK1_M = 0.10048
LINK2_M = 0.094714
LINK3_M = 0.05071
TOOL_LINK_M = 0.1126
TOOL_FRAME = "base_link"

SERVO_DEVICE_NAME = os.getenv("ARMPIFPV_SERVO_DEVICE", "/dev/rrc")
SERVO_BAUD_RATE = int(os.getenv("ARMPIFPV_SERVO_BAUD", "1000000"))
SERVO_TIMEOUT_MS = int(os.getenv("ARMPIFPV_SERVO_TIMEOUT_MS", "100"))
SERVO_RETRIES = int(os.getenv("ARMPIFPV_SERVO_RETRIES", "5"))
SERVO_LOOP_SLEEP_MS = int(os.getenv("ARMPIFPV_SERVO_LOOP_SLEEP_MS", "20"))
SERVO_DYNAMIC_READ_EVERY_N_LOOPS = int(os.getenv("ARMPIFPV_SERVO_POSITION_EVERY_N", "1"))
SERVO_TELEMETRY_EVERY_N_LOOPS = int(os.getenv("ARMPIFPV_SERVO_TELEMETRY_EVERY_N", "25"))

CAMERA_WIDTH = int(os.getenv("ARMPIFPV_CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("ARMPIFPV_CAMERA_HEIGHT", "480"))
CAMERA_FPS = int(os.getenv("ARMPIFPV_CAMERA_FPS", "30"))
CAMERA_DEVICE_INDEX = int(os.getenv("ARMPIFPV_CAMERA_DEVICE_INDEX", "0"))
CAMERA_USE_JPEG = os.getenv("ARMPIFPV_CAMERA_USE_JPEG", "1") != "0"

GRAPH_METRICS_PRINTING_HZ = float(os.getenv("ARMPIFPV_GRAPH_METRICS_HZ", "1.0"))
GRAPH_DEBUG = os.getenv("ARMPIFPV_GRAPH_DEBUG", "0") == "1"
GRAPH_METRICS_BROKER = os.getenv("ARMPIFPV_GRAPH_METRICS_BROKER", "")
GRAPH_METRICS_PORT = int(os.getenv("ARMPIFPV_GRAPH_METRICS_PORT", "1883"))
GRAPH_METRICS_TOPIC = os.getenv("ARMPIFPV_GRAPH_METRICS_TOPIC", "metrics")
GRAPH_METRICS_QOS = int(os.getenv("ARMPIFPV_GRAPH_METRICS_QOS", "0"))

DEFAULT_MOVE_TIME_S = float(os.getenv("ARMPIFPV_DEFAULT_MOVE_TIME_S", "1.0"))
STATE_MAX_AGE_S = float(os.getenv("ARMPIFPV_STATE_MAX_AGE_S", "2.0"))

IK_MAX_ITERS = int(os.getenv("ARMPIFPV_IK_MAX_ITERS", "200"))
IK_DAMPING = float(os.getenv("ARMPIFPV_IK_DAMPING", "0.02"))
IK_STEP_SCALE = float(os.getenv("ARMPIFPV_IK_STEP_SCALE", "0.7"))
IK_ORIENTATION_WEIGHT = float(os.getenv("ARMPIFPV_IK_ORIENTATION_WEIGHT", "0.35"))
IK_POSITION_TOLERANCE_M = float(os.getenv("ARMPIFPV_IK_POSITION_TOLERANCE_M", "0.01"))
IK_ORIENTATION_TOLERANCE_RAD = float(os.getenv("ARMPIFPV_IK_ORIENTATION_TOLERANCE_RAD", "0.20"))

# Bind/connect split supports single-host use and container/remote use.
ZMQ_CAMERA_BIND = os.getenv("ROBOFLEX_ZMQ_CAMERA_BIND", "tcp://127.0.0.1:5555")
ZMQ_CAMERA_CONNECT = os.getenv("ROBOFLEX_ZMQ_CAMERA_CONNECT", "tcp://127.0.0.1:5555")

ZMQ_SERVO_STATE_BIND = os.getenv("ROBOFLEX_ZMQ_SERVO_STATE_BIND", "tcp://127.0.0.1:5556")
ZMQ_SERVO_STATE_CONNECT = os.getenv("ROBOFLEX_ZMQ_SERVO_STATE_CONNECT", "tcp://127.0.0.1:5556")

ZMQ_SERVO_CMD_BIND = os.getenv("ROBOFLEX_ZMQ_SERVO_CMD_BIND", "tcp://0.0.0.0:5558")
ZMQ_SERVO_CMD_CONNECT = os.getenv("ROBOFLEX_ZMQ_SERVO_CMD_CONNECT", "tcp://127.0.0.1:5558")
