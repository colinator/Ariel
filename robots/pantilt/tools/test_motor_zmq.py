#!/usr/bin/env python3
"""Test motor control via ZMQ.

Requires the hardware process to be running:
    python -m robots.pantilt.hardware

Reads current state, then runs a ±10° sine wave on both motors for 2 seconds.

Usage:
    python -m robots.pantilt.tools.test_motor_zmq
"""

import math
import time
import threading
import sys
from pathlib import Path
import roboflex.dynamixel as rfd
import roboflex.transport.zmq as rzmq
from roboflex import CallbackFun

try:
    from robots.pantilt.config import ZMQ_MOTOR_CMD_BIND, ZMQ_MOTOR_STATE_CONNECT
except ModuleNotFoundError:
    # Support direct execution: python robots/pantilt/tools/test_motor_zmq.py
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from robots.pantilt.config import ZMQ_MOTOR_CMD_BIND, ZMQ_MOTOR_STATE_CONNECT

IDS = [5, 6]
TICKS_PER_DEG = 4096.0 / 360.0
AMPLITUDE = 10.0 * TICKS_PER_DEG
DURATION = 2.0

zmq_ctx = rzmq.ZMQContext()

# Subscribe to state (server publishes, we connect)
state_sub = rzmq.ZMQSubscriber(zmq_ctx, ZMQ_MOTOR_STATE_CONNECT, max_queued_msgs=1)

# Publish commands (we bind, server's subscriber connects to us)
cmd_pub = rzmq.ZMQPublisher(zmq_ctx, ZMQ_MOTOR_CMD_BIND, max_queued_msgs=1)

# Grab initial state via callback
got_state = threading.Event()
start_pos = {}

def on_state(msg):
    if got_state.is_set():
        return
    state = rfd.DynamixelGroupStateMessage(msg).state
    for mid in IDS:
        start_pos[mid] = state.values[mid][rfd.DXLControlTable.PresentPosition]
    got_state.set()

cb = CallbackFun(on_state)
state_sub > cb

print("Connecting to motor stream...")
state_sub.start()

if not got_state.wait(timeout=5.0):
    print("ERROR: No state received (is python -m robots.pantilt.hardware running?)")
    state_sub.stop()
    exit(1)

for mid in IDS:
    print(f"  Motor {mid}: position = {start_pos[mid]}")

# Give the server's cmd_sub a moment to connect to our publisher
time.sleep(0.5)

print(f"\nSine wave test via ZMQ: ±10°, {DURATION}s")
t_start = time.time()

while True:
    elapsed = time.time() - t_start
    if elapsed > DURATION:
        break

    phase = (elapsed / DURATION) * 2 * math.pi
    offset = int(AMPLITUDE * math.sin(phase))

    command = rfd.DynamixelGroupCommand()
    for mid in IDS:
        command.set(mid, rfd.DXLControlTable.GoalPosition, start_pos[mid] + offset)
    msg = rfd.DynamixelGroupCommandMessage(command)
    cmd_pub.signal_self(msg)
    time.sleep(0.033)  # ~30Hz — don't overwhelm the serial bus

# Return to start
command = rfd.DynamixelGroupCommand()
for mid in IDS:
    command.set(mid, rfd.DXLControlTable.GoalPosition, start_pos[mid])
msg = rfd.DynamixelGroupCommandMessage(command)
cmd_pub.signal_self(msg)

time.sleep(0.5)
state_sub.stop()
print("Done.")
