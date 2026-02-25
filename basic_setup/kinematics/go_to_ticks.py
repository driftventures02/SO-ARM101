#!/usr/bin/env python3
"""Move from current position to target ticks with linear interpolation."""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from servo_utils import (
    ServoBus,
    REG_GOAL_POSITION,
    REG_PRESENT_POSITION,
    REG_TORQUE_ENABLE,
    to_u16,
)

MOTOR_IDS = [1, 2, 3, 4, 5, 6]


def read_positions(bus: ServoBus) -> dict[int, int]:
    out = {}
    for motor_id in MOTOR_IDS:
        p = bus.read_reg(motor_id, REG_PRESENT_POSITION, 2)
        if p and len(p) >= 2:
            out[motor_id] = to_u16(p[0], p[1])
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--target", nargs=4, type=int, required=True,
                        metavar=("PAN", "SHOULDER", "ELBOW", "WRIST"))
    parser.add_argument("--steps", type=int, default=50)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()

    target = {1: args.target[0], 2: args.target[1], 3: args.target[2], 4: args.target[3]}
    bus = ServoBus(args.port)
    time.sleep(0.2)

    try:
        # Try reading twice — first read can fail after fresh connect.
        cur = read_positions(bus)
        if not cur:
            time.sleep(0.1)
            cur = read_positions(bus)
        if not cur:
            print("ERROR: Can't read any motors. Check port/connection.")
            return
        print(f"Current: {cur}")
        print(f"Target:  {target}")

        # Sync goals then enable torque.
        for mid in MOTOR_IDS:
            bus.write_reg(mid, REG_GOAL_POSITION, cur.get(mid, 2048), 2)
        time.sleep(0.05)
        for mid in MOTOR_IDS:
            bus.write_reg(mid, REG_TORQUE_ENABLE, 1, 1)
        time.sleep(0.05)

        # Interpolate to target.
        dt = args.duration / args.steps
        for step in range(1, args.steps + 1):
            t = step / args.steps
            for mid in (1, 2, 3, 4):
                s = cur.get(mid, 2048)
                pos = int(s + (target[mid] - s) * t) % 4096
                bus.write_reg(mid, REG_GOAL_POSITION, pos, 2)
            time.sleep(dt)

        print(f"Done. Press Enter to return and release torque.")
        input()

        # Return to original.
        now = read_positions(bus)
        for step in range(1, args.steps + 1):
            t = step / args.steps
            for mid in (1, 2, 3, 4):
                s = now.get(mid, 2048)
                orig = cur.get(mid, 2048)
                pos = int(s + (orig - s) * t) % 4096
                bus.write_reg(mid, REG_GOAL_POSITION, pos, 2)
            time.sleep(dt)
    finally:
        for mid in MOTOR_IDS:
            bus.write_reg(mid, REG_TORQUE_ENABLE, 0, 1)
        print("Torque OFF.")
        bus.close()


if __name__ == "__main__":
    main()
