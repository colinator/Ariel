# ArmPi-FPV Robot For Ariel

This directory contains an Ariel robot integration for the Hiwonder ArmPi-FPV.

It exposes:
- a 5-DOF arm with semantic joint names
- a 1-DOF parallel gripper
- one USB camera
- optional Intel RealSense RGB-D camera support
- local FK/IK helpers
- a Pi-side hardware server built on Roboflex
- an Ariel proxy used by the REPL and MCP server

The target actuator path is the Hiwonder controller-board protocol on `/dev/rrc` rather than direct servo UART. Camera and servo traffic between the hardware process and the Ariel proxy use local ZMQ endpoints, and this robot is currently configured to use `ipc://...` endpoints rather than `tcp://...`. That means the normal Ariel stack for this robot is intended to run on the same machine, typically the Raspberry Pi.

## Files

- [config.py](/Users/colinprepscius/code/ariel/robots/armpifpv/config.py): robot config, servo mapping, ZMQ endpoints, kinematics constants
- [hardware.py](/Users/colinprepscius/code/ariel/robots/armpifpv/hardware.py): Pi-side hardware process
- [robot.py](/Users/colinprepscius/code/ariel/robots/armpifpv/robot.py): Ariel robot proxy injected into the REPL as `robot`
- [kinematics.py](/Users/colinprepscius/code/ariel/robots/armpifpv/kinematics.py): FK/IK helpers
- [test_kinematics.py](/Users/colinprepscius/code/ariel/robots/armpifpv/test_kinematics.py): pure software FK/IK tests
- calibration and validation helpers:
  - [calibrate_servo_semantics.py](/Users/colinprepscius/code/ariel/robots/armpifpv/calibrate_servo_semantics.py)
  - [calibrate_servo_offset_interactive.py](/Users/colinprepscius/code/ariel/robots/armpifpv/calibrate_servo_offset_interactive.py)
  - [validate_fk_direct.py](/Users/colinprepscius/code/ariel/robots/armpifpv/validate_fk_direct.py)
  - [validate_ik_direct.py](/Users/colinprepscius/code/ariel/robots/armpifpv/validate_ik_direct.py)

## Hardware Notes

- Servo ids are:
  - `1`: gripper
  - `2`: wrist_roll
  - `3`: wrist_pitch
  - `4`: elbow
  - `5`: shoulder
  - `6`: base_yaw
- Arm joint APIs use radians.
- Gripper openness is approximate and coarse. Treat it as open-ish / close-ish, not a calibrated jaw-width model.
- The regular camera can be disabled with `ARMPIFPV_CAMERA_ENABLE=0`.
- The regular camera local IPC feed is raw RGB by default; set `ARMPIFPV_CAMERA_USE_JPEG=1` only if IPC bandwidth matters more than CPU.
- The low-rate TCP monitor camera feed is raw RGB by default on a wired link; set `ARMPIFPV_MONITOR_CAMERA_USE_JPEG=1` to compress only that monitor branch.
- RealSense support is opt-in with `ARMPIFPV_REALSENSE_ENABLE=1`.
- Low-rate TCP camera monitor streams are enabled by default at 1 Hz:
  - regular camera: `tcp://0.0.0.0:5560`
  - RealSense frameset, when enabled: `tcp://0.0.0.0:5561`
- Motion is visibly jerky on this robot, including with the vendor stack. This is mostly the hardware, not a sign that Ariel is necessarily doing something wrong.
- In practice, small brisk staged moves work better than very slow creeping moves.
- Cartesian control is approximate. Small `x/z` moves are usually more reliable than `y` moves from a typical forward-facing pose.

## Python Dependencies

Robot-local dependencies are listed in [requirements.txt](/Users/colinprepscius/code/ariel/robots/armpifpv/requirements.txt).

On the Pi, the important runtime packages are typically:
- `roboflex`
- `roboflex.hiwonder_bus_servo`
- `roboflex.transport.zmq`
- `roboflex.transport.mqtt`
- `roboflex.webcam_gst`
- `roboflex.util.jpeg`
- `numpy`
- `Pillow`
- `matplotlib`
- `opencv-python-headless`

For the raw Ariel MCP stack on the Pi, you also need the top-level MCP server dependencies from [/Users/colinprepscius/code/ariel/requirements-mcp.txt](/Users/colinprepscius/code/ariel/requirements-mcp.txt), especially:
- `mcp`
- `fastapi`
- `uvicorn`

## Quick Checks

Direct hardware self-test:

```bash
python -m robots.armpifpv.hardware --self-test-direct
```

Proxy-path self-test:

```bash
python -m robots.armpifpv.hardware --self-test
```

Pure software FK/IK tests:

```bash
python -m unittest robots.armpifpv.test_kinematics
```

## Running Ariel On The Robot Without Docker

This section assumes everything runs directly on the Raspberry Pi, with no Docker. Because this robot uses `ipc://...` ZMQ endpoints, that is the intended topology.

### 1. Make sure `robot.conf` points at ArmPi-FPV

Set [/Users/colinprepscius/code/ariel/robot.conf](/Users/colinprepscius/code/ariel/robot.conf) to:

```text
robots.armpifpv.robot:ArmPiFPVRobotProxy
```

Without that, the REPL and MCP server will still try to load whatever robot is currently configured there.

### 2. Stop the vendor stack

Free `/dev/rrc` before starting Ariel:

```bash
sudo systemctl disable --now start_node.service
docker stop armpi_fpv
```

Then verify the vendor processes are gone.

### 3. Activate the Python environment

Example:

```bash
cd /home/pi/arieltesting/Ariel
source robots/armpifpv/.pyvenv/bin/activate
```

### 4. Start the hardware process

In terminal 1:

```bash
cd /home/pi/arieltesting/Ariel
source robots/armpifpv/.pyvenv/bin/activate
ARMPIFPV_CAMERA_DEVICE_INDEX=2 \
ARMPIFPV_REALSENSE_ENABLE=1 \
python -m robots.armpifpv.hardware
```

Servo-only low-CPU mode:

```bash
cd /home/pi/arieltesting/Ariel
source robots/armpifpv/.pyvenv/bin/activate
ARMPIFPV_CAMERA_ENABLE=0 \
ARMPIFPV_REALSENSE_ENABLE=0 \
ARMPIFPV_MONITOR_ENABLE=0 \
python -m robots.armpifpv.hardware
```

This process owns:
- the camera
- the RealSense, when `ARMPIFPV_REALSENSE_ENABLE=1`
- `/dev/rrc`
- the local Roboflex graph
- the local ZMQ `ipc://...` endpoints

### 5. Optional: start the REPL directly

In terminal 2, if you want to test the raw Ariel REPL without MCP:

```bash
cd /home/pi/arieltesting/Ariel
source robots/armpifpv/.pyvenv/bin/activate
python -m server.repl_server --robot-conf robot.conf
```

This loads `robot = ArmPiFPVRobotProxy()` inside the REPL subprocess. In normal MCP use you do not launch this manually, because the MCP server launches it for you.

### 6. Start the MCP server

In terminal 2 for normal use, or terminal 3 if you also started the raw REPL manually:

```bash
cd /home/pi/arieltesting/Ariel
source robots/armpifpv/.pyvenv/bin/activate
export ARIEL_IMAGE_RETURN_MODE=inline
export ARMPIFPV_REALSENSE_ENABLE=1
python -m server.mcp_server
```

`ARIEL_IMAGE_RETURN_MODE=inline` is the simplest raw non-Docker choice because it avoids path-mapping requirements for cached image files.

The MCP endpoint is:

```text
http://127.0.0.1:8750/mcp
```

... but for remote clients:
```text
http://192.168.0.195:8750/mcp
```

### 7. What is running

For the normal non-Docker stack, the processes are:

- `python -m robots.armpifpv.hardware`
  - immortal hardware-side process
  - owns camera and servos

- `python -m server.mcp_server`
  - MCP HTTP server
  - manages the REPL subprocess

- `python -m server.repl_server --robot-conf robot.conf`
  - launched automatically by the MCP server
  - contains the `robot` proxy object
  - talks to the hardware process over local ZMQ IPC

There is no separate standalone proxy process. The proxy lives inside the REPL process.

## Practical Workflow

Typical raw bringup on the Pi:

Terminal 1:

```bash
cd /home/pi/arieltesting/Ariel
source robots/armpifpv/.pyvenv/bin/activate
ARMPIFPV_CAMERA_DEVICE_INDEX=2 \
ARMPIFPV_REALSENSE_ENABLE=1 \
python -m robots.armpifpv.hardware
```

Terminal 2:

```bash
cd /home/pi/arieltesting/Ariel
source robots/armpifpv/.pyvenv/bin/activate
export ARIEL_IMAGE_RETURN_MODE=inline
export ARMPIFPV_REALSENSE_ENABLE=1
python -m server.mcp_server
```

Then connect your MCP client to:

```text
http://127.0.0.1:8750/mcp
```

## Notes

- Because ZMQ uses `ipc://...` here, the MCP server and hardware server should run on the same machine.
- If you later want host/container or multi-machine topologies, switch the ZMQ endpoints in [config.py](/Users/colinprepscius/code/ariel/robots/armpifpv/config.py) back to `tcp://...` and update bind/connect roles accordingly.
- `robot.arm.stop()` now maps to a real hardware stop request, but only if the updated `roboflex.hiwonder_bus_servo` build is installed on the Pi.
