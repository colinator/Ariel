#!/usr/bin/env python3
"""View the camera ZMQ stream in an SDL2 window.

Requires the hardware process to be running:
    python -m robots.pantilt.hardware

Usage:
    python -m robots.pantilt.tools.view_stream [width] [height] [fps]

Defaults: 640x480@30fps.
"""

import sys
from pathlib import Path
import roboflex.transport.zmq as rzmq
import roboflex.visualization as rv

try:
    from robots.pantilt.config import ZMQ_CAMERA_CONNECT
except ModuleNotFoundError:
    # Support direct execution: python robots/pantilt/tools/view_stream.py
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from robots.pantilt.config import ZMQ_CAMERA_CONNECT

width = int(sys.argv[1]) if len(sys.argv) > 1 else 640
height = int(sys.argv[2]) if len(sys.argv) > 2 else 480
fps = int(sys.argv[3]) if len(sys.argv) > 3 else 30

zmq_ctx = rzmq.ZMQContext()
sub = rzmq.ZMQSubscriber(zmq_ctx, ZMQ_CAMERA_CONNECT, max_queued_msgs=1)
tv = rv.RGBImageTV(frequency_hz=float(fps), width=width, height=height)

sub > tv

print(f"Viewing stream from {ZMQ_CAMERA_CONNECT} ({width}x{height}@{fps}fps)")
print("Close the SDL2 window to quit.")

sub.start()
try:
    tv.run()  # blocks on main thread (macOS SDL2 requirement)
finally:
    print("\nShutting down...")
    sub.stop()
    tv.stop()
    print("Done.")
