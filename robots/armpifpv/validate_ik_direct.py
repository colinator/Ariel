#!/usr/bin/env python3
"""Direct Cartesian IK validation helper for ArmPi-FPV.

This script performs a small Cartesian neighborhood test around the current
pose. Because the ArmPi-FPV is only 5-DOF plus gripper, it cannot hold an
arbitrary full 6-DOF pose exactly while translating in XYZ. So this test keeps
the current orientation as a soft preference, allows approximation, and checks
how the achieved pose actually changes.

For each axis, it executes:
- +s
- -2s
- +s

where s is the user-supplied step size in meters.
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np
import roboflex.hiwonder_bus_servo as rhs

from . import kinematics
from .config import SERVO_BAUD_RATE, SERVO_DEVICE_NAME, SERVO_RETRIES, SERVO_TIMEOUT_MS


ARM_SERVO_ID_BY_NAME = {
    "base_yaw": 6,
    "shoulder": 5,
    "elbow": 4,
    "wrist_pitch": 3,
    "wrist_roll": 2,
}


def _read_position_retry(controller: rhs.HiwonderBusServoController, servo_id: int, retries: int = 8, delay_s: float = 0.05):
    for _ in range(retries):
        pulse = controller.read_position(servo_id)
        if pulse is not None:
            return int(pulse)
        time.sleep(delay_s)
    return None


def _read_arm_pulses(controller: rhs.HiwonderBusServoController):
    return {
        name: _read_position_retry(controller, servo_id)
        for name, servo_id in ARM_SERVO_ID_BY_NAME.items()
    }


def _arm_joints_from_pulses(pulses):
    missing = [name for name, pulse in pulses.items() if pulse is None]
    if missing:
        raise RuntimeError(f"missing direct pulse reads for {missing}")
    return kinematics.arm_pulses_to_joints(pulses)


def _position_delta(a, b):
    return (np.asarray(b, dtype=float) - np.asarray(a, dtype=float)).tolist()


def _write_joints(controller: rhs.HiwonderBusServoController, joints: dict[str, float], move_time: float):
    pulses = kinematics.arm_joints_to_pulses(joints)
    controller.write_positions_ms(
        int(round(max(move_time, 0.1) * 1000.0)),
        {ARM_SERVO_ID_BY_NAME[name]: pulse for name, pulse in pulses.items()},
    )
    return pulses


def _read_pose(controller: rhs.HiwonderBusServoController):
    pulses = _read_arm_pulses(controller)
    joints = _arm_joints_from_pulses(pulses)
    pose = kinematics.fk_pose(joints)
    return pulses, joints, pose


def _run_step(controller, label: str, axis: str, delta_m: float, move_time: float, current_joints: dict[str, float], orientation_xyzw):
    current_pose = kinematics.fk_pose(current_joints)
    requested_position = list(current_pose["position"])
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    requested_position[axis_index] += delta_m

    result = kinematics.solve_ik(
        requested_position,
        orientation_xyzw,
        initial_joints=current_joints,
        prefer_current=True,
        allow_approx=True,
    )
    target_pulses = kinematics.arm_joints_to_pulses(result["joints"])

    print(f"{label}: requested position {requested_position}", flush=True)
    print(f"{label}: solver result {result}", flush=True)
    print(f"{label}: target pulses {target_pulses}", flush=True)

    _write_joints(controller, result["joints"], move_time)
    time.sleep(move_time + 0.4)

    final_pulses, final_joints, final_pose = _read_pose(controller)
    print(f"{label}: final pulses {final_pulses}", flush=True)
    print(f"{label}: final joints {final_joints}", flush=True)
    print(f"{label}: final pose {final_pose}", flush=True)
    print(
        f"{label}: achieved position delta "
        f"{_position_delta(current_pose['position'], final_pose['position'])}",
        flush=True,
    )
    actual_orientation_error = kinematics._orientation_distance_rad(orientation_xyzw, final_pose["orientation"])
    print(f"{label}: orientation drift rad {actual_orientation_error:.4f}", flush=True)
    return final_joints, final_pose


def main():
    parser = argparse.ArgumentParser(description="Run a direct Cartesian neighborhood IK test against the local controller.")
    parser.add_argument(
        "--step",
        "-s",
        type=float,
        default=0.01,
        help="Cartesian step size in meters for each axis sequence",
    )
    parser.add_argument(
        "--move-time",
        type=float,
        default=0.3,
        help="move time in seconds for each Cartesian step",
    )
    parser.add_argument(
        "--axes",
        nargs="*",
        choices=("x", "y", "z"),
        default=("z", "x", "y"),
        help="axis order to test",
    )
    args = parser.parse_args()

    controller = rhs.HiwonderBusServoController(
        SERVO_DEVICE_NAME,
        SERVO_BAUD_RATE,
        SERVO_TIMEOUT_MS,
        SERVO_RETRIES,
    )

    start_pulses, start_joints, start_pose = _read_pose(controller)
    preferred_orientation = start_pose["orientation"]

    print("IK validation: start pulses", start_pulses, flush=True)
    print("IK validation: start joints", start_joints, flush=True)
    print("IK validation: start pose", start_pose, flush=True)
    print(
        "IK validation: using current orientation as a soft preference; "
        "this 5-DOF arm cannot generally preserve full orientation exactly.",
        flush=True,
    )

    current_joints = dict(start_joints)

    try:
        for axis in args.axes:
            print(f"IK validation: axis {axis} sequence with step={args.step} m", flush=True)
            for label, delta in (
                (f"{axis} +s", +args.step),
                (f"{axis} -2s", -2.0 * args.step),
                (f"{axis} +s return", +args.step),
            ):
                current_joints, _ = _run_step(
                    controller,
                    label,
                    axis,
                    delta,
                    args.move_time,
                    current_joints,
                    preferred_orientation,
                )
    finally:
        controller.write_positions_ms(
            int(round(max(args.move_time, 0.4) * 1000.0)),
            {ARM_SERVO_ID_BY_NAME[name]: pulse for name, pulse in start_pulses.items()},
        )
        time.sleep(max(args.move_time, 0.4) + 0.4)
        restored = _read_arm_pulses(controller)
        print("IK validation: restored pulses", restored, flush=True)
        try:
            restored_joints = _arm_joints_from_pulses(restored)
            print("IK validation: restored pose", kinematics.fk_pose(restored_joints), flush=True)
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
