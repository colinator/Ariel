#!/usr/bin/env python3
"""Direct FK validation helper for ArmPi-FPV.

Moves exactly one arm joint through a small pulse delta using the local
Hiwonder controller, then prints direct pulse snapshots and FK before/after.
This is meant to validate the kinematic model with as little extra machinery
as possible.
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


def _clamp_pulse(value: int) -> int:
    return int(max(0, min(1000, int(value))))


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


def _pose_delta(before, after):
    pb = before["position"]
    pa = after["position"]
    return [pa[i] - pb[i] for i in range(3)]


def main():
    parser = argparse.ArgumentParser(description="Validate FK directly against one joint motion.")
    parser.add_argument("--joint", required=True, choices=tuple(ARM_SERVO_ID_BY_NAME.keys()))
    parser.add_argument("--delta-deg", type=float, default=10.0)
    parser.add_argument("--move-time", type=float, default=0.5)
    args = parser.parse_args()

    controller = rhs.HiwonderBusServoController(
        SERVO_DEVICE_NAME,
        SERVO_BAUD_RATE,
        SERVO_TIMEOUT_MS,
        SERVO_RETRIES,
    )

    counts_per_deg = 1000.0 / 240.0
    delta_counts = int(round(args.delta_deg * counts_per_deg))
    servo_id = ARM_SERVO_ID_BY_NAME[args.joint]

    start_pulses = _read_arm_pulses(controller)
    start_joints = _arm_joints_from_pulses(start_pulses)
    start_pose = kinematics.fk_pose(start_joints)
    start_pulse = start_pulses[args.joint]
    target_pulse = _clamp_pulse(start_pulse + delta_counts)

    print(f"FK validation: joint={args.joint} servo_id={servo_id} start_pulse={start_pulse} target_pulse={target_pulse}")
    print("before pulses", start_pulses, flush=True)
    print("before joints", start_joints, flush=True)
    print("before pose", start_pose, flush=True)

    try:
        controller.write_positions_ms(int(round(args.move_time * 1000.0)), {servo_id: target_pulse})
        time.sleep(args.move_time + 0.5)

        moved_pulses = _read_arm_pulses(controller)
        moved_joints = _arm_joints_from_pulses(moved_pulses)
        moved_pose = kinematics.fk_pose(moved_joints)

        print("after pulses", moved_pulses, flush=True)
        print("after joints", moved_joints, flush=True)
        print("after pose", moved_pose, flush=True)
        print("position delta", _pose_delta(start_pose, moved_pose), flush=True)
    finally:
        controller.write_positions_ms(int(round(max(args.move_time, 0.4) * 1000.0)), {servo_id: start_pulse})
        time.sleep(max(args.move_time, 0.4) + 0.4)
        print("restored pulse", _read_position_retry(controller, servo_id), flush=True)


if __name__ == "__main__":
    main()
