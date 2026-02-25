#!/usr/bin/env python3
"""Move from current position to target ticks with safe interpolation."""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from servo_utils import ServoBus
from kinematics.motion import read_positions, torque_on, torque_off, move


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--target", nargs=4, type=int, required=True,
                        metavar=("PAN", "SHOULDER", "ELBOW", "WRIST"))
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()

    target = {1: args.target[0], 2: args.target[1], 3: args.target[2], 4: args.target[3]}
    bus = ServoBus(args.port)
    time.sleep(0.2)

    try:
        cur = read_positions(bus)
        if not cur:
            print("ERROR: Can't read any motors. Check port/connection.")
            return
        print(f"Current: {cur}")
        print(f"Target:  {target}")

        torque_on(bus, cur)
        move(bus, cur, target, steps=args.steps, duration=args.duration)

        print("Done. Press Enter to return and release torque.")
        input()

        # Return to original position.
        now = read_positions(bus)
        move(bus, now, cur, steps=args.steps, duration=args.duration)

    finally:
        torque_off(bus)
        print("Torque OFF.")
        bus.close()


if __name__ == "__main__":
    main()
