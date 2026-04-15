#!/usr/bin/env python3
"""Direct IK validation helper for ArmPi-FPV.

Default behavior is solver-only:
- read current arm pulses directly
- compute current FK pose
- apply a small requested Cartesian offset
- solve IK from the current joint state
- print requested vs achieved pose and solver errors

Optional `--execute` will command the returned joint target directly and then
measure the resulting pulses / FK pose.
"""

from __future__ import annotations

import argparse
import time

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
    pulses = {}
    for name, servo_id in ARM_SERVO_ID_BY_NAME.items():
        pulses[name] = _read_position_retry(controller, servo_id)
    return pulses


def _arm_joints_from_pulses(pulses):
    missing = [name for name, pulse in pulses.items() if pulse is None]
    if missing:
        raise RuntimeError(f"missing direct pulse reads for {missing}")
    return kinematics.arm_pulses_to_joints(pulses)


def _apply_offset(position, dx: float, dy: float, dz: float):
    return [position[0] + dx, position[1] + dy, position[2] + dz]


def main():
    parser = argparse.ArgumentParser(description="Validate IK directly against the local controller.")
    parser.add_argument("--dx", type=float, default=0.0, help="target Cartesian x offset in meters")
    parser.add_argument("--dy", type=float, default=0.0, help="target Cartesian y offset in meters")
    parser.add_argument("--dz", type=float, default=0.01, help="target Cartesian z offset in meters")
    parser.add_argument("--move-time", type=float, default=0.3, help="execution move time if --execute is used")
    parser.add_argument("--execute", action="store_true", help="actually command the solved joint target")
    parser.add_argument("--allow-approx", action="store_true", help="allow approximate IK results instead of failing")
    args = parser.parse_args()

    controller = rhs.HiwonderBusServoController(
        SERVO_DEVICE_NAME,
        SERVO_BAUD_RATE,
        SERVO_TIMEOUT_MS,
        SERVO_RETRIES,
    )

    start_pulses = _read_arm_pulses(controller)
    start_joints = _arm_joints_from_pulses(start_pulses)
    start_pose = kinematics.fk_pose(start_joints)
    requested_position = _apply_offset(start_pose["position"], args.dx, args.dy, args.dz)

    print("IK validation: current pulses", start_pulses, flush=True)
    print("IK validation: current joints", start_joints, flush=True)
    print("IK validation: current pose", start_pose, flush=True)
    print("IK validation: requested position", requested_position, flush=True)
    print("IK validation: requested orientation", start_pose["orientation"], flush=True)

    result = kinematics.solve_ik(
        requested_position,
        start_pose["orientation"],
        initial_joints=start_joints,
        prefer_current=True,
        allow_approx=args.allow_approx,
    )
    target_pulses = kinematics.arm_joints_to_pulses(result["joints"])

    print("IK validation: solver result", result, flush=True)
    print("IK validation: target pulses", target_pulses, flush=True)

    if not args.execute:
        return

    try:
        controller.write_positions_ms(
            int(round(max(args.move_time, 0.1) * 1000.0)),
            {ARM_SERVO_ID_BY_NAME[name]: pulse for name, pulse in target_pulses.items()},
        )
        time.sleep(max(args.move_time, 0.1) + 0.5)

        final_pulses = _read_arm_pulses(controller)
        final_joints = _arm_joints_from_pulses(final_pulses)
        final_pose = kinematics.fk_pose(final_joints)

        print("IK validation: final pulses", final_pulses, flush=True)
        print("IK validation: final joints", final_joints, flush=True)
        print("IK validation: final pose", final_pose, flush=True)
    finally:
        controller.write_positions_ms(
            int(round(max(args.move_time, 0.4) * 1000.0)),
            {ARM_SERVO_ID_BY_NAME[name]: pulse for name, pulse in start_pulses.items()},
        )
        time.sleep(max(args.move_time, 0.4) + 0.4)
        print("IK validation: restored pulses", _read_arm_pulses(controller), flush=True)


if __name__ == "__main__":
    main()
