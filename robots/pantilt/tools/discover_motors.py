#!/usr/bin/env python3
"""Read current positions of Dynamixel motors. Nudge one to identify which is which.

Usage:
    python -m robots.pantilt.tools.discover_motors [--nudge ID]

Without --nudge: just reads and prints positions for IDs 5 and 6.
With --nudge 5: briefly moves motor 5 by +200 then back, so you can see which one it is.
"""

import sys
import time
import roboflex.dynamixel as rfd

DEVICE = "/dev/cu.usbserial-FT2KQC4K"
BAUD = 115_200
IDS = [5, 6]

controller = rfd.DynamixelGroupController.PositionController(
    device_name=DEVICE,
    baud_rate=BAUD,
    dxl_ids=IDS,
)

state = controller.read()
print("Current positions:")
for mid in IDS:
    pos = state.values[mid][rfd.DXLControlTable.PresentPosition]
    print(f"  Motor {mid}: position = {pos}")

if len(sys.argv) > 2 and sys.argv[1] == "--nudge":
    nudge_id = int(sys.argv[2])
    pos = state.values[nudge_id][rfd.DXLControlTable.PresentPosition]
    print(f"\nNudging motor {nudge_id} from {pos} to {pos + 200} and back...")

    iters = [0]
    def nudge_fn(state, command):
        iters[0] += 1
        command.set(nudge_id, rfd.DXLControlTable.GoalPosition, pos + 200)
        return iters[0] < 50

    controller.run_readwrite_loop(nudge_fn)
    time.sleep(0.5)

    iters[0] = 0
    def return_fn(state, command):
        iters[0] += 1
        command.set(nudge_id, rfd.DXLControlTable.GoalPosition, pos)
        return iters[0] < 50

    controller.run_readwrite_loop(return_fn)
    print("Done — motor should be back.")
else:
    print("\nRun with --nudge 5 or --nudge 6 to identify which motor is which.")
