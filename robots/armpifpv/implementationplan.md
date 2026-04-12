# ArmPi-FPV Ariel Robot Implementation Plan

## Goal

Add a new Ariel robot at `robots/armpifpv` that supports:

- 5-DOF arm control
- 1-DOF parallel gripper control
- onboard USB camera access
- FK/IK helpers
- local execution on the Raspberry Pi
- remote/containerized use via ZMQ, following the same general split as `robots/pantilt`

This plan is for the first implementation pass. It describes the intended API and architecture before code is written.

## Source Material Reviewed

- Ariel template/reference:
  - `robots/pantilt/robot.py`
  - `robots/pantilt/hardware.py`
  - `robots/pantilt/config.py`
- Vendor ArmPi-FPV assets:
  - `../ArmPi-FPV/src/hiwonder_servo_controllers/config/hiwonder_servo_controller.yaml`
  - `../ArmPi-FPV/src/armpi_fpv_kinematics/src/armpi_fpv_kinematics/transform.py`
  - `../ArmPi-FPV/src/armpi_fpv_kinematics/scripts/search_kinematics_solutions_node.py`
  - `../ArmPi-FPV/src/armpi_fpv_descrption/urdf/armpi_fpv.urdf`
  - `../ArmPi-FPV/src/armpi_fpv_moveit_config/config/joint_limits.yaml`
- Existing Roboflex support already built elsewhere:
  - `roboflex_hiwonder_bus_servo`
  - `roboflex_webcam_gst`
  - `roboflex_transport_zmq`
  - `roboflex_util_jpeg` now available
  - `../flexrobotics/roboflex/include/roboflex_core/core_nodes/graph_root.h`
  - `../flexrobotics/roboflex/python/pybindings.cpp`

## What We Know

### Hardware / transport

- The real actuator target is the Hiwonder controller-board protocol on `/dev/rrc` or `/dev/ttyAMA0`.
- The low-level Roboflex support is already implemented in `roboflex_hiwonder_bus_servo`.
- Writes are naturally grouped timed moves; reads are more expensive and should remain optional/lower-rate.
- The camera is a USB webcam on `/dev/video0`, 640x480, and `roboflex_webcam_gst` is known-good.

### Servo mapping

From vendor config:

- servo 1 -> `r_joint`
- servo 2 -> `joint5`
- servo 3 -> `joint4`
- servo 4 -> `joint3`
- servo 5 -> `joint2`
- servo 6 -> `joint1`

From vendor URDF:

- `joint1..joint5` are the 5 arm joints used by FK/IK
- `r_joint` is the actuated gripper joint
- the left gripper jaw is modeled with mimic joints, so Ariel should expose a single gripper DOF

### Kinematics

Vendor code already defines:

- link lengths
- joint angle ranges
- pulse <-> joint-angle mappings
- FK and IK helper logic

Important constraint:

- vendor FK/IK is for the 5 arm joints only, not the gripper
- the vendor code selects IK solutions by minimizing distance from current servo positions

## Planned Ariel Surface

The Ariel robot should present semantic names rather than vendor raw names.

### Cameras

- `robot.cameras["main"]`

Primary camera API:

- `grab_frame(timeout=...)`

### Arm joints

Expose semantic joint names in arm order:

- `base_yaw`
- `shoulder`
- `elbow`
- `wrist_pitch`
- `wrist_roll`

Internal mapping:

- `base_yaw` -> vendor `joint1` -> servo 6
- `shoulder` -> vendor `joint2` -> servo 5
- `elbow` -> vendor `joint3` -> servo 4
- `wrist_pitch` -> vendor `joint4` -> servo 3
- `wrist_roll` -> vendor `joint5` -> servo 2
- `gripper` -> vendor `r_joint` -> servo 1

### Gripper

Expose a single gripper DOF:

- `robot.gripper`

Expected API:

- `get_position()`
- `set_position(value, move_time=...)`
- `open(move_time=...)`
- `close(move_time=...)`
- `get_limits()`

The exact position domain should be servo-native at first pass unless we discover a robust jaw-width calibration in repo assets. If there is no trustworthy metric width model, we should document that this is a normalized or pulse-domain command surface.

### Arm object

Expose a higher-level arm interface:

- `robot.arm.get_joint_positions()`
- `robot.arm.set_joint_positions(..., move_time=...)`
- `robot.arm.move_joints(joints, move_time=...)`
- `robot.arm.home(move_time=...)`
- `robot.arm.stop()`

The primary command mode should be coordinated timed moves.

Streaming-style loops must remain possible through repeated commands, but the docs should state clearly that this hardware behaves best with waypoint-style motion and may move jerkily under high-rate host-side updates.

### FK / IK helpers

Expose FK/IK through the arm object:

- `robot.arm.fk(joints=None)`
- `robot.arm.ik(position, orientation, prefer_current=True, allow_approx=True, resolution=...)`
- `robot.arm.move_pose(position, orientation, move_time=..., allow_approx=True, resolution=...)`
- `robot.arm.get_pose()`

First-pass return types should be simple Python dicts or small value objects that are easy for the REPL and MCP to inspect.

Pose API decision:

- the Ariel API will be SE(3)-shaped at the boundary
- `position` is Cartesian XYZ in meters in the base frame
- `orientation` is a quaternion `[x, y, z, w]` or equivalent clearly-documented representation
- FK returns a full pose in that same shape
- IK and pose motion accept full pose requests in that same shape

Important implementation rule:

- the robot is still a 5-DOF arm plus gripper, not a full 6-DOF manipulator
- the solver must therefore treat orientation as constrained and may need to approximate the request
- the API should report that explicitly rather than silently pretending the requested pose was exactly achievable

Recommended IK result shape:

- `success`
- `joints`
- `pose_requested`
- `pose_achieved`
- `position_error_m`
- `orientation_error_rad`
- `approximated`

Secondary convenience wrappers may be added later for pitch/roll style calls, but they are not the primary Ariel interface.

## Architectural Plan

Match the broad pattern from `robots/pantilt`:

- `hardware.py` owns local Pi hardware and publishes data / receives commands over ZMQ
- `robot.py` is the proxy class used by Ariel REPL/MCP
- `config.py` holds endpoints, device names, mappings, limits, defaults
- `__init__.py` exports the robot

### Proposed files

- `robots/armpifpv/__init__.py`
- `robots/armpifpv/config.py`
- `robots/armpifpv/hardware.py`
- `robots/armpifpv/robot.py`
- `robots/armpifpv/kinematics.py`
- `robots/armpifpv/requirements.txt`

## Planned Data Flow

### Hardware side

`hardware.py` should:

- open the webcam via `roboflex.webcam_gst.WebcamSensor`
- open the bus-servo controller via `roboflex_hiwonder_bus_servo`
- assemble the local hardware graph under `roboflex.GraphRoot`
- publish camera frames over ZMQ
- publish servo/joint state over ZMQ
- subscribe for command messages over ZMQ

`GraphRoot` requirement:

- the hardware-side robot graph should be rooted in `GraphRoot` so local Roboflex graph metrics are available immediately
- this is for local operator/debug visibility, not for MCP exposure
- the Ariel proxy and `_device_docs()` do not need to expose graph metrics features
- implementation should use the existing Python binding for `GraphRoot`
- prefer a structure where startup/shutdown goes through `GraphRoot.start_all(...)` / `GraphRoot.profile(...)` and `GraphRoot.stop()`

Command handling should support:

- grouped timed arm/gripper moves
- optional single-servo commands
- optional stop/torque helpers if supported cleanly by the existing module

State publication should include enough information for:

- current servo positions
- converted semantic joint positions
- latest telemetry if cheap enough or already available

Because reads are expensive, the state publisher should use a modest polling cadence and allow a fake/estimated position path if the underlying Roboflex module already supports command-tracked state. We should not accidentally turn this into a high-rate poll loop.

### Proxy side

`robot.py` should:

- connect to the hardware-side ZMQ streams
- expose `cameras`, `arm`, `gripper`, and possibly raw `motors`/`servos`
- keep the `RobotBase` contract:
  - `connect()`
  - `close()`
  - `snapshot(device_name)`
  - `_device_docs()`

It should also provide:

- `is_alive()`
- coordinated motion helpers
- optional lightweight logging helpers if cheap to add

## Motion and Units

### Internal representation

Use a clear separation:

- wire/hardware layer: servo pulse domain or servo-native units
- Ariel-facing semantic layer: named joints in radians

Rationale:

- vendor FK/IK and URDF are joint-centric
- radians are the least surprising format for robotics helpers
- pulse conversion is hardware-specific and should stay internal

### Command duration

Timed moves should be first-class.

Every coordinated move helper should accept `move_time` in seconds and map it to the bus-servo timed command semantics.

### Limits

Initial limits should come from the vendor kinematics transform and URDF, not guessed safety values.

We should document two layers of limits:

- semantic joint limits in radians
- servo-domain min/max pulses

## FK / IK Strategy

Implement FK/IK behind a dedicated Ariel-local `kinematics.py`.

Preferred strategy:

- first check whether the vendor FK/IK `.so` artifacts can be imported and called directly from Python without bringing in ROS runtime assumptions
- if that works cleanly, wrap them behind Ariel-local helpers
- if that does not work cleanly, port the required FK/IK math into Ariel-local Python

Constraints:

- Ariel should not depend on ROS node startup or catkin service wiring
- `robot.py` and `hardware.py` should not know whether kinematics is backed by vendor binaries or a pure-Python implementation
- IK solution selection should still prefer the result closest to current joint state when multiple solutions exist
- the public pose API should remain SE(3)-shaped even if the underlying vendor solver is expressed in a reduced pitch/roll-style parameterization

`kinematics.py` should own:

- semantic joint ordering
- pulse <-> radians conversion
- FK helpers
- IK helpers
- pose/joint normalization and validation

## Camera Plan

Start with the same camera pattern as `pantilt`:

- one `"main"` camera
- `snapshot("main")` returns JPEG-encoded image data
- `grab_frame()` returns RGB numpy arrays

If JPEG compression is easy to insert cleanly on the transport path, do it now. If it creates extra complexity in the first robot pass, defer it and keep the API stable.

## Remote + Local Use

The design should support both:

- local Pi process running `hardware.py`
- remote/containerized Ariel REPL connecting to its ZMQ endpoints

This means `config.py` should follow the same bind/connect split used in `pantilt`.

## Dependency Manifest Plan

Add a robot-local dependency manifest at:

- `robots/armpifpv/requirements.txt`

Rationale:

- this repo already uses `requirements.txt`-style dependency files
- there is no existing per-robot `pyproject.toml` or packaging convention under `robots/`
- a robot-local requirements file gives a concrete install target for the Raspberry Pi and for anyone bringing up this robot independently

The initial dependency set should reflect the packages already verified on the Pi:

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

Notes:

- the file should contain the actual pip package names as required by installation, even if import names differ
- it is acceptable for this robot-specific requirements file to be narrower than the repo-root requirements files
- MQTT can remain in the manifest even if not used in the first robot pass, since it is already validated on the target Pi and likely useful for follow-on work

## Documentation Plan

`robot.describe()` should explain:

- semantic joint naming
- gripper behavior
- timed-move preference
- FK/IK helpers and expected arguments
- exactly what pose notation is supported at the Ariel layer
- that pose requests use explicit SE(3)-style position + orientation
- that orientation reachability is constrained because the arm is 5-DOF
- that IK may approximate the requested orientation and will report the achieved pose/error
- camera API
- background-task and control-loop guidance inherited from `RobotBase`
- the warning that high-rate loops are allowed but may produce jerky motion on this hardware

This is especially important because `_device_docs()` is the primary LLM-facing contract for how the robot should be driven.

## Open Assumptions To Carry Into Implementation

- Semantic naming will replace vendor raw names in the public API.
- Gripper control will be treated as one scalar DOF.
- Ariel-facing joint units will be radians.
- First implementation will prioritize clean and usable semantics over exposing every low-level servo register.
- Camera starts as one device named `"main"`.

## Execution Steps

1. Create `config.py` with:
   - ZMQ endpoints
   - camera defaults
   - servo/joint mapping tables
   - semantic joint limits
   - home/default poses
   - dependency assumptions aligned with `requirements.txt`

2. Create `robots/armpifpv/requirements.txt`:
   - include the robot-specific Roboflex and Python dependencies
   - prefer exact package names that are known to install on the Pi

3. Create the hardware server in `hardware.py`:
   - build the camera/servo/ZMQ graph under `GraphRoot`
   - webcam sensor + publisher
   - Hiwonder bus-servo controller setup
   - ZMQ command subscriber
   - modest-rate joint/state publication
   - local metrics/profiling hook for operator use

4. Create semantic conversion helpers:
   - pulse <-> radians
   - semantic joint order <-> servo id order
   - pose/joint helper structures

5. Implement FK/IK helpers:
   - probe vendor `.so` usability first
   - otherwise port vendor transform constants
   - implement FK
   - implement IK solution selection against current state

6. Create the Ariel proxy in `robot.py`:
   - camera proxy
   - arm proxy
   - gripper proxy
   - `snapshot`, `is_alive`, `describe`

7. Add coordinated motion helpers:
   - `set_joint_positions`
   - `move_pose`
   - `home`
   - gripper open/close

8. Verify against expected UX:
   - local hardware server start
   - remote proxy connect
   - snapshot camera
   - read current joints
   - run a simple timed joint move
   - run FK on live state
   - run IK for a reachable pose and execute it

9. Add brief usage docs or inline comments only where needed for non-obvious behavior.

## Risks

- The exact Roboflex bus-servo Python API in `roboflex_hiwonder_bus_servo` may differ from the assumptions above and will need to drive final command/state shapes.
- Vendor IK behavior may depend on compiled helpers that are not directly reusable; a Python reimplementation may take some iteration.
- Gripper semantics may remain pulse-based unless there is enough geometry/calibration data to expose a meaningful physical aperture.
- Camera transport may need a follow-up compression pass even if the first functional version works uncompressed.

## Definition of Done

This first robot is done when:

- `robots/armpifpv` contains a working Ariel robot implementation
- Ariel can connect to a running ArmPi-FPV hardware server over ZMQ
- the robot exposes camera, arm, gripper, and FK/IK helpers
- timed coordinated joint moves work
- `robot.describe()` clearly documents the intended usage and constraints

## Execution Checklist

- [x] Create `robots/armpifpv/config.py`
- [x] Create `robots/armpifpv/requirements.txt`
- [x] Create `robots/armpifpv/kinematics.py`
- [x] Check whether vendor FK/IK `.so` files are directly reusable from Ariel
- [x] Define semantic joint names, mappings, limits, and home pose
- [x] Implement hardware-side camera graph
- [x] Implement hardware-side bus-servo graph
- [x] Root the hardware graph in `roboflex.GraphRoot`
- [x] Implement hardware-side ZMQ command/state transport
- [x] Implement proxy-side camera interface
- [x] Implement proxy-side arm interface
- [x] Implement proxy-side gripper interface
- [x] Implement SE(3)-shaped FK interface
- [x] Implement SE(3)-shaped IK interface with approximation/error reporting
- [x] Implement timed coordinated joint motion helpers
- [x] Implement pose-motion helpers
- [x] Write strong `_device_docs()` guidance for LLM use
- [ ] Verify local hardware startup/shutdown
- [ ] Verify remote proxy connectivity
- [ ] Verify camera snapshot/frame access
- [ ] Verify live joint-state access
- [ ] Verify a timed arm move
- [ ] Verify gripper open/close
- [ ] Verify FK on live state
- [ ] Verify IK on a reachable pose
