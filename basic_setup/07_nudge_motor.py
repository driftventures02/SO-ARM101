#!/usr/bin/env python3
"""Nudge one motor by a relative number of ticks (default: ID 1 by +2000)."""

import argparse
import time

from servo_utils import (
    ServoBus,
    REG_GOAL_POSITION,
    REG_PRESENT_POSITION,
    REG_TORQUE_ENABLE,
    to_u16,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/cu.usbmodemXXXXX")
    parser.add_argument("--id", type=int, default=1, help="Motor ID to move")
    parser.add_argument("--delta", type=int, default=2000, help="Relative tick move (can be negative)")
    parser.add_argument("--wait", type=float, default=0.4, help="Seconds to wait before reading back position")
    args = parser.parse_args()

    bus = ServoBus(args.port)
    try:
        p = bus.read_reg(args.id, REG_PRESENT_POSITION, 2)
        if not p:
            print(f"No response from motor ID {args.id}")
            return

        cur = to_u16(p[0], p[1])
        target = max(0, min(4095, cur + args.delta))
        print(f"ID {args.id}: current={cur}, target={target} (delta={args.delta})")

        # Power LED != torque enabled. Force torque on so the move command is applied.
        bus.write_reg(args.id, REG_TORQUE_ENABLE, 1, 1)
        time.sleep(0.05)
        bus.write_reg(args.id, REG_GOAL_POSITION, target, 2)
        time.sleep(max(0.0, args.wait))

        p2 = bus.read_reg(args.id, REG_PRESENT_POSITION, 2)
        if p2:
            new = to_u16(p2[0], p2[1])
            print(f"ID {args.id}: new_position={new}")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
