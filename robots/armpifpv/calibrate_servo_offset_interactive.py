#!/usr/bin/env python3
"""Interactive direct helper for servo offset/deviation inspection.

This script does not try to auto-calibrate. Instead it:

- moves one servo to a chosen inspection pulse (default: 500)
- lets you nudge it from the terminal in small pulse increments
- records your estimate of whether the misalignment is minor enough for
  software deviation or large enough to require re-indexing the horn/spline

Use it when physically inspecting alignment against a straightedge, known
neutral pose, or vendor reference photo.
"""

from __future__ import annotations

import argparse
import json
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


def _move_and_wait(
    controller: rhs.HiwonderBusServoController,
    servo_id: int,
    target_pulse: int,
    *,
    move_time: float,
    settle_s: float = 0.35,
) -> int | None:
    controller.write_positions_ms(int(round(max(move_time, 0.05) * 1000.0)), {servo_id: target_pulse})
    time.sleep(move_time + settle_s)
    return _read_position_retry(controller, servo_id)


def _estimate_software_vs_mechanical(offset_pulse: int) -> str:
    # Vendor docs suggest roughly <=13 deg is software-fixable.
    # 1000 pulses / 240 deg ~= 4.1667 pulses/deg, so 13 deg ~= 54 pulses.
    if abs(offset_pulse) <= 54:
        return "likely software-fixable"
    return "likely mechanical re-indexing first"


def main():
    parser = argparse.ArgumentParser(description="Interactive servo offset/deviation inspection.")
    parser.add_argument(
        "--joint",
        required=True,
        choices=tuple(SERVO_ID_BY_NAME.keys()),
        help="semantic joint name to inspect",
    )
    parser.add_argument(
        "--inspect-pulse",
        type=int,
        default=500,
        help="starting pulse for visual inspection; 500 is the usual center",
    )
    parser.add_argument(
        "--move-time",
        type=float,
        default=0.5,
        help="seconds for inspection/nudge moves",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=5,
        help="default pulse increment for bare + / - commands",
    )
    parser.add_argument(
        "--save-json",
        default=None,
        help="optional path to save a JSON summary of the inspection",
    )
    args = parser.parse_args()

    servo_id = SERVO_ID_BY_NAME[args.joint]
    controller = rhs.HiwonderBusServoController(
        SERVO_DEVICE_NAME,
        SERVO_BAUD_RATE,
        SERVO_TIMEOUT_MS,
        SERVO_RETRIES,
    )

    start_pulse = _read_position_retry(controller, servo_id)
    if start_pulse is None:
        raise RuntimeError(f"could not read starting pulse for {args.joint} (servo {servo_id})")

    current_target = _clamp_pulse(args.inspect_pulse)
    print(f"Offset inspection: joint={args.joint} servo_id={servo_id}")
    print(f"  starting pulse on power-up/readback: {start_pulse}")
    print(f"  moving to inspection pulse: {current_target}")
    measured = _move_and_wait(controller, servo_id, current_target, move_time=args.move_time)
    print(f"  measured pulse after move: {measured}")
    print("")
    print("Inspect the physical alignment now.")
    print("Commands:")
    print("  +            nudge +step pulses")
    print("  -            nudge -step pulses")
    print("  +N / -N      nudge by N pulses, e.g. +12 or -20")
    print("  p            print current target/readback/offset recommendation")
    print("  p N          explicitly read servo id N now, e.g. `p 5`")
    print("  note TEXT    append a note")
    print("  done         finish and print summary")
    print("  abort        restore start pulse and exit")
    print("")

    notes: list[str] = []
    last_measured = measured

    try:
        while True:
            raw = input("> ").strip()
            if not raw:
                continue
            if raw == "done":
                break
            if raw == "abort":
                raise KeyboardInterrupt
            if raw == "p":
                offset = current_target - args.inspect_pulse
                print(f"target={current_target} measured={last_measured} offset_from_inspect={offset}")
                print(_estimate_software_vs_mechanical(offset))
                continue
            if raw.startswith("p "):
                suffix = raw[2:].strip()
                try:
                    requested_servo_id = int(suffix)
                except ValueError:
                    print("expected integer servo id after `p`")
                    continue
                pulse = _read_position_retry(controller, requested_servo_id)
                print(f"servo_id={requested_servo_id} direct_read_position={pulse}")
                continue
            if raw.startswith("note "):
                note = raw[5:].strip()
                if note:
                    notes.append(note)
                    print("noted")
                continue

            delta = None
            if raw == "+":
                delta = args.step
            elif raw == "-":
                delta = -args.step
            elif raw.startswith("+") or raw.startswith("-"):
                try:
                    delta = int(raw)
                except ValueError:
                    pass

            if delta is None:
                print("unrecognized command")
                continue

            current_target = _clamp_pulse(current_target + delta)
            last_measured = _move_and_wait(controller, servo_id, current_target, move_time=args.move_time)
            print(f"target={current_target} measured={last_measured}")

        estimated_offset = current_target - args.inspect_pulse
        summary = {
            "joint": args.joint,
            "servo_id": servo_id,
            "starting_pulse": start_pulse,
            "inspection_pulse": args.inspect_pulse,
            "best_visual_pulse": current_target,
            "measured_pulse": last_measured,
            "estimated_offset_pulse": estimated_offset,
            "estimated_offset_deg_nominal": estimated_offset * 240.0 / 1000.0,
            "recommendation": _estimate_software_vs_mechanical(estimated_offset),
            "notes": notes,
        }
        print("")
        print("Summary:")
        print(json.dumps(summary, indent=2))
        if args.save_json:
            with open(args.save_json, "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2)
            print(f"saved to {args.save_json}")
    finally:
        print("")
        print("Restoring starting pulse...")
        controller.write_positions_ms(int(round(max(args.move_time, 0.3) * 1000.0)), {servo_id: start_pulse})
        time.sleep(max(args.move_time, 0.3) + 0.3)
        print("final pulse", _read_position_retry(controller, servo_id))


if __name__ == "__main__":
    main()
