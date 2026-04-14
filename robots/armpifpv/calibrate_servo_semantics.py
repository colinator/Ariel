#!/usr/bin/env python3
"""Direct servo calibration helper for ArmPi-FPV.

This bypasses the Ariel proxy/ZMQ path and talks straight to the local
HiwonderBusServoController. Use it to verify:

- servo id -> physical joint mapping
- +pulse / -pulse physical direction
- whether a joint reliably returns to its starting pulse
"""

from __future__ import annotations

import argparse
import time

import roboflex.hiwonder_bus_servo as rhs

from .config import SERVO_BAUD_RATE, SERVO_DEVICE_NAME, SERVO_RETRIES, SERVO_TIMEOUT_MS


SERVO_ID_BY_NAME = {
    "gripper": 1,
    "wrist_roll": 2,
    "wrist_pitch": 3,
    "elbow": 4,
    "shoulder": 5,
    "base_yaw": 6,
}


def _clamp_pulse(value: int) -> int:
    return int(max(0, min(1000, int(value))))


def _read_position_retry(
    controller: rhs.HiwonderBusServoController,
    servo_id: int,
    *,
    retries: int = 8,
    delay_s: float = 0.05,
) -> int | None:
    for _ in range(retries):
        pulse = controller.read_position(servo_id)
        if pulse is not None:
            return int(pulse)
        time.sleep(delay_s)
    return None


def _wait_for_pulse(
    controller: rhs.HiwonderBusServoController,
    servo_id: int,
    target_pulse: int,
    *,
    move_time: float,
    tolerance: int = 8,
    settle_window_s: float = 0.25,
    timeout_pad_s: float = 2.0,
    poll_s: float = 0.08,
) -> int | None:
    deadline = time.time() + move_time + timeout_pad_s
    settled_since = None
    last_pulse = None

    while time.time() < deadline:
        pulse = _read_position_retry(controller, servo_id, retries=2, delay_s=min(poll_s, 0.04))
        if pulse is None:
            time.sleep(poll_s)
            continue
        last_pulse = pulse
        if abs(pulse - target_pulse) <= tolerance:
            if settled_since is None:
                settled_since = time.time()
            elif time.time() - settled_since >= settle_window_s:
                return pulse
        else:
            settled_since = None
        time.sleep(poll_s)

    return last_pulse


def main():
    parser = argparse.ArgumentParser(description="Direct ArmPi-FPV servo calibration helper.")
    parser.add_argument(
        "--joint",
        required=True,
        choices=tuple(SERVO_ID_BY_NAME.keys()),
        help="semantic joint name to test",
    )
    parser.add_argument("--delta-deg", type=float, default=20.0, help="pulse delta expressed in nominal servo degrees")
    parser.add_argument("--move-time", type=float, default=0.4, help="seconds for the +delta and return legs")
    parser.add_argument("--middle-move-time", type=float, default=0.8, help="seconds for the -delta leg")
    parser.add_argument(
        "--target-pulses",
        type=int,
        nargs="+",
        default=None,
        help="explicit pulse sequence to command in order; final restore to the starting pulse still happens automatically",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=1,
        help="repeat the +delta / -delta / return sequence this many times",
    )
    args = parser.parse_args()

    servo_id = SERVO_ID_BY_NAME[args.joint]
    counts_per_deg = 1000.0 / 240.0
    delta_counts = int(round(args.delta_deg * counts_per_deg))

    controller = rhs.HiwonderBusServoController(
        SERVO_DEVICE_NAME,
        SERVO_BAUD_RATE,
        SERVO_TIMEOUT_MS,
        SERVO_RETRIES,
    )

    start_pulse = _read_position_retry(controller, servo_id)
    if start_pulse is None:
        raise RuntimeError(f"could not read starting pulse for {args.joint} (servo {servo_id})")

    pos_target = _clamp_pulse(start_pulse + delta_counts)
    neg_target = _clamp_pulse(start_pulse - delta_counts)

    print(f"Calibration: joint={args.joint} servo_id={servo_id}")
    print(f"  start_pulse={start_pulse}")
    if args.target_pulses:
        explicit_targets = [_clamp_pulse(value) for value in args.target_pulses]
        print(f"  explicit_targets={explicit_targets}")
    else:
        explicit_targets = None
        print(f"  pos_target={pos_target}  (+{args.delta_deg} deg nominal)")
        print(f"  neg_target={neg_target}  (-{args.delta_deg} deg nominal)")
    print("Observe which physical joint moves, and which direction +pulse corresponds to.", flush=True)

    try:
        if explicit_targets is not None:
            for index, target in enumerate(explicit_targets, start=1):
                move_time = args.middle_move_time if index == 2 and len(explicit_targets) >= 2 else args.move_time
                print(f"step {index}: target pulse {target}", flush=True)
                controller.write_positions_ms(int(round(move_time * 1000.0)), {servo_id: target})
                print("  measured pulse", _wait_for_pulse(controller, servo_id, target, move_time=move_time), flush=True)
        else:
            for cycle in range(args.cycles):
                print(f"cycle {cycle + 1}: +delta", flush=True)
                controller.write_positions_ms(int(round(args.move_time * 1000.0)), {servo_id: pos_target})
                print("  measured pulse", _wait_for_pulse(controller, servo_id, pos_target, move_time=args.move_time), flush=True)

                print(f"cycle {cycle + 1}: -2*delta (to start-delta)", flush=True)
                controller.write_positions_ms(int(round(args.middle_move_time * 1000.0)), {servo_id: neg_target})
                print("  measured pulse", _wait_for_pulse(controller, servo_id, neg_target, move_time=args.middle_move_time), flush=True)

                print(f"cycle {cycle + 1}: return to start", flush=True)
                controller.write_positions_ms(int(round(args.move_time * 1000.0)), {servo_id: start_pulse})
                print("  measured pulse", _wait_for_pulse(controller, servo_id, start_pulse, move_time=args.move_time), flush=True)
    finally:
        print("final restore to start", flush=True)
        controller.write_positions_ms(int(round(max(args.move_time, 0.3) * 1000.0)), {servo_id: start_pulse})
        time.sleep(max(args.move_time, 0.3) + 0.3)
        print("  final pulse", _read_position_retry(controller, servo_id), flush=True)


if __name__ == "__main__":
    main()
