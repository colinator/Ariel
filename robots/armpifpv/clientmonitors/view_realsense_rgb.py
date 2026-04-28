#!/usr/bin/env python3
"""View the low-rate ArmPi-FPV RealSense RGB monitor feed."""

from __future__ import annotations

import sys
from pathlib import Path
from urllib.parse import urlparse

import roboflex.transport.zmq as rzmq
import roboflex.visualization as rv

try:
    from robots.armpifpv.config import (
        MONITOR_HZ,
        REALSENSE_HEIGHT,
        REALSENSE_WIDTH,
        ZMQ_MONITOR_REALSENSE_BIND,
    )
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from robots.armpifpv.config import (
        MONITOR_HZ,
        REALSENSE_HEIGHT,
        REALSENSE_WIDTH,
        ZMQ_MONITOR_REALSENSE_BIND,
    )


def _connect_endpoint(bind_endpoint: str, host: str) -> str:
    parsed = urlparse(bind_endpoint)
    if parsed.scheme != "tcp":
        raise ValueError(f"monitor endpoint must be tcp, got {bind_endpoint!r}")
    return f"tcp://{host}:{parsed.port}"


host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
width = int(sys.argv[2]) if len(sys.argv) > 2 else REALSENSE_WIDTH
height = int(sys.argv[3]) if len(sys.argv) > 3 else REALSENSE_HEIGHT
fps = float(sys.argv[4]) if len(sys.argv) > 4 else max(MONITOR_HZ, 1.0)
endpoint = _connect_endpoint(ZMQ_MONITOR_REALSENSE_BIND, host)

zmq_ctx = rzmq.ZMQContext()
sub = rzmq.ZMQSubscriber(zmq_ctx, endpoint, name="ArmPiMonitorRealSenseSub", max_queued_msgs=1)
tv = rv.RGBImageTV(
    frequency_hz=fps,
    width=width,
    height=height,
    image_key="rgb",
    debug=False,
    mirror=False,
    name="ArmPi RealSense RGB Monitor",
)

sub > tv

print(f"Viewing RealSense RGB monitor from {endpoint} ({width}x{height}@{fps:g}Hz)")
print("Close the SDL2 window to quit.")

sub.start()
try:
    tv.run()
finally:
    sub.stop()
    tv.stop()
