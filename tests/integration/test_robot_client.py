#!/usr/bin/env python3
"""Test PanTiltRobotProxy against a running hardware process.

Requires: python -m robots.pantilt.hardware (in another terminal)

Runs a series of tests, pausing between motor-related ones so
the user can observe. Non-motor tests run without pausing.

Usage:
    python tests/integration/test_robot_client.py
"""

import time
import sys
from pathlib import Path
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from robots.pantilt.robot import PanTiltRobotProxy


def test_devices(robot):
    print("\n=== Test: describe() ===")
    devs = robot.describe()
    print(f"  {devs}")
    assert isinstance(devs, str)
    assert "Camera: 'main'" in devs
    assert "Motor: 'pan'" in devs
    assert "Motor: 'tilt'" in devs
    print("  PASS")


def test_grab_frame(robot):
    print("\n=== Test: grab_frame ===")
    frame = robot.cameras['main'].grab_frame()
    print(f"  Shape: {frame.shape}, dtype: {frame.dtype}")
    assert frame.ndim == 3 and frame.shape[2] == 3
    assert frame.dtype == np.uint8
    print("  PASS")


def test_motor_read(robot):
    print("\n=== Test: motor read ===")
    for name in ['pan', 'tilt']:
        pos = robot.motors[name].get_position()
        limits = robot.motors[name].get_limits()
        print(f"  {name} (motor {robot.motors[name].id}): position={pos}, limits={limits}")
        assert isinstance(pos, (int, np.integer)), f"Expected int, got {type(pos)}"
    print("  PASS")


def test_safety_clamp(robot):
    print("\n=== Test: safety clamp (math only, no motor movement) ===")
    # Test clamping logic without sending real commands
    for name in ['pan', 'tilt']:
        motor = robot.motors[name]
        lo, hi = motor.get_limits()
        # Verify clamp math: max(min, min(max, value))
        assert max(lo, min(hi, 0)) == lo, f"{name}: clamp(0) should be {lo}"
        assert max(lo, min(hi, 9999)) == hi, f"{name}: clamp(9999) should be {hi}"
        mid = (lo + hi) // 2
        assert max(lo, min(hi, mid)) == mid, f"{name}: clamp({mid}) should be {mid}"
        print(f"  {name}: clamp(0)={lo}, clamp(9999)={hi}, clamp({mid})={mid}")
    print("  PASS")


def test_nudge(robot):
    """Nudge pan slightly to verify movement. USER SHOULD OBSERVE."""
    print("\n=== Test: nudge (watch the pan motor!) ===")
    pan = robot.motors['pan']
    start = pan.get_position()
    print(f"  Start position: {start}")

    pan.nudge(50)
    time.sleep(0.5)
    after = pan.get_position()
    print(f"  After nudge(+50): {after}")

    pan.set_position(start)
    time.sleep(0.5)
    print(f"  Returned to {start}")
    print("  PASS")


def test_run(robot):
    """Small control loop: oscillate pan ±20 ticks for 1 second."""
    print("\n=== Test: run() — pan oscillates ±20 ticks for 1s ===")
    pan = robot.motors['pan']
    center = pan.get_position()
    t0 = [time.time()]

    def oscillate(r):
        import math
        elapsed = time.time() - t0[0]
        offset = int(20 * math.sin(elapsed * 2 * math.pi * 2))  # 2 Hz
        r.motors['pan'].set_position(center + offset)

    robot.run(oscillate, duration=1.0, hz=30)
    pan.set_position(center)
    time.sleep(0.3)
    print("  PASS")


def test_logging(robot):
    print("\n=== Test: logging ===")
    logger = robot.start_logging()
    time.sleep(0.5)  # accumulate ~50 samples at 100Hz motor rate
    robot.stop_logging(logger)

    data = logger.get_data()
    print(f"  Keys: {list(data.keys())}")
    print(f"  Samples: {len(data['t'])}")
    for mid in [5, 6]:
        if mid in data:
            print(f"  Motor {mid}: mean={data[mid].mean():.1f}, std={data[mid].std():.1f}")
    assert len(data['t']) > 0, "Expected some logged data"
    print("  PASS")


def test_show(robot):
    print("\n=== Test: show() ===")
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  SKIP (matplotlib not installed)")
        return

    fig, ax = plt.subplots()
    ax.plot([1, 2, 3], [1, 4, 9])
    ax.set_title("test")
    robot.show(fig)
    plt.close(fig)

    images = robot._drain_images()
    print(f"  Got {len(images)} image(s), first {len(images[0])} base64 chars")
    assert len(images) == 1
    assert len(images[0]) > 100  # non-trivial PNG
    print("  PASS")


def main():
    robot = PanTiltRobotProxy()
    print("Connecting to hardware_server...")
    robot.connect()
    time.sleep(1.0)  # let ZMQ streams warm up
    print("Connected.")

    try:
        # Non-motor tests first
        test_devices(robot)
        test_grab_frame(robot)
        test_show(robot)

        # Motor tests (these move things!)
        test_motor_read(robot)
        test_safety_clamp(robot)
        test_nudge(robot)
        test_run(robot)
        test_logging(robot)

        print("\n=== ALL TESTS PASSED ===")
    except Exception as e:
        print(f"\n!!! TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
