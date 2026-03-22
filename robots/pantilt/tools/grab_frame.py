#!/usr/bin/env python3
"""Grab a single frame from the camera ZMQ stream and save as JPEG.

Requires the hardware process to be running:
    python -m robots.pantilt.hardware

Usage:
    python -m robots.pantilt.tools.grab_frame [output_path]

Defaults: output_path=/tmp/rf_frame.jpg
"""

import sys
import threading
from pathlib import Path
from PIL import Image
import roboflex.transport.zmq as rzmq
import roboflex.webcam_gst as rcw
from roboflex import CallbackFun

try:
    from robots.pantilt.config import ZMQ_CAMERA_CONNECT
except ModuleNotFoundError:
    # Support direct execution: python robots/pantilt/tools/grab_frame.py
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from robots.pantilt.config import ZMQ_CAMERA_CONNECT

output_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rf_frame.jpg"

got_frame = threading.Event()
frame_rgb = [None]

def on_frame(msg):
    frame_rgb[0] = rcw.WebcamDataRGB(msg).rgb
    got_frame.set()

zmq_ctx = rzmq.ZMQContext()
sub = rzmq.ZMQSubscriber(zmq_ctx, ZMQ_CAMERA_CONNECT, max_queued_msgs=1)
cb = CallbackFun(on_frame)
sub > cb

print("Waiting for frame...")
sub.start()

if not got_frame.wait(timeout=5.0):
    print("ERROR: No frame received (is python -m robots.pantilt.hardware running?)")
    sub.stop()
    sys.exit(1)

sub.stop()

rgb = frame_rgb[0]
print(f"Got frame: {rgb.shape}")
Image.fromarray(rgb).save(output_path)
print(f"Saved to {output_path}")
