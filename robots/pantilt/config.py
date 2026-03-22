"""Pan/tilt robot configuration."""

import os

# XH430: 0-4095, center = 2048, 4096 ticks / 360 deg
TICKS_PER_DEG = 4096.0 / 360.0

MOTOR_LIMITS = {
    5: {
        "name": "pan",
        "min": int(2048 - 90 * TICKS_PER_DEG),
        "max": int(2048 + 90 * TICKS_PER_DEG),
        "center": 2048,
    },
    6: {
        "name": "tilt",
        "min": int(3267 - 80 * TICKS_PER_DEG),
        "max": int(3267 + 10 * TICKS_PER_DEG),
        "center": 3267,
    },
}

MOTOR_NAME_TO_ID = {v["name"]: k for k, v in MOTOR_LIMITS.items()}

# ZMQ endpoints are split into bind/connect roles so host-hardware + container-MCP works.
# Defaults target single-host execution (everything on host).
# Containerized MCP should override *_CONNECT via docker-compose env vars.
ZMQ_CAMERA_BIND = os.getenv("ROBOFLEX_ZMQ_CAMERA_BIND", "tcp://127.0.0.1:5555")
ZMQ_CAMERA_CONNECT = os.getenv("ROBOFLEX_ZMQ_CAMERA_CONNECT", "tcp://127.0.0.1:5555")

ZMQ_MOTOR_STATE_BIND = os.getenv("ROBOFLEX_ZMQ_MOTOR_STATE_BIND", "tcp://127.0.0.1:5556")
ZMQ_MOTOR_STATE_CONNECT = os.getenv("ROBOFLEX_ZMQ_MOTOR_STATE_CONNECT", "tcp://127.0.0.1:5556")

ZMQ_MOTOR_CMD_BIND = os.getenv("ROBOFLEX_ZMQ_MOTOR_CMD_BIND", "tcp://0.0.0.0:5558")
ZMQ_MOTOR_CMD_CONNECT = os.getenv("ROBOFLEX_ZMQ_MOTOR_CMD_CONNECT", "tcp://127.0.0.1:5558")
