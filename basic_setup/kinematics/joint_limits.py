#!/usr/bin/env python3
"""
Record safe joint limits interactively and save to JSON.

Usage:
    uv run python basic_setup/kinematics/joint_limits.py --port /dev/cu.usbmodemXXXXX

How it works:
  1. Disables torque so you can move each joint by hand.
  2. For each joint, prompts you to move it to the MIN safe position, then MAX.
  3. Reads the encoder tick at each extreme.
  4. Saves everything to a JSON file you can load from IK or teleop.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

# Allow running from basic_setup/ directory.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from servo_utils import ServoBus, REG_PRESENT_POSITION, to_u16

MOTOR_IDS = [1, 2, 3, 4, 5, 6]
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper"]

DEFAULT_OUT = str(Path(__file__).resolve().parent / "joint_limits.json")


def read_position(bus: ServoBus, motor_id: int) -> int | None:
    p = bus.read_reg(motor_id, REG_PRESENT_POSITION, 2)
    return to_u16(p[0], p[1]) if p else None


def load_limits(path: str | Path) -> dict:
    """Load existing limits JSON. Returns empty dict if file missing."""
    p = Path(path)
    if p.exists():
        with open(p) as f:
            return json.load(f)
    return {}


def main():
    parser = argparse.ArgumentParser(description="Record safe joint limits")
    parser.add_argument("--port", required=True)
    parser.add_argument("--out", default=DEFAULT_OUT,
                        help="Output JSON path (default: kinematics/joint_limits.json)")
    args = parser.parse_args()

    bus = ServoBus(args.port)

    # Disable torque so user can move arm freely.
    bus.set_torque_all(MOTOR_IDS, False)
    print("\nTorque OFF — arm is free to move by hand.\n")

    limits: dict[str, dict] = {}

    for motor_id, name in zip(MOTOR_IDS, NAMES):
        print(f"--- [{motor_id}] {name} ---")
        input(f"  Move to MIN safe position, then press Enter...")
        pos_min = read_position(bus, motor_id)
        if pos_min is None:
            print(f"  No response from motor {motor_id}, skipping.")
            continue
        print(f"  MIN recorded: {pos_min}")

        input(f"  Move to MAX safe position, then press Enter...")
        pos_max = read_position(bus, motor_id)
        if pos_max is None:
            print(f"  No response from motor {motor_id}, skipping.")
            continue
        print(f"  MAX recorded: {pos_max}")

        # Ensure min < max regardless of which direction user moved first.
        lo, hi = min(pos_min, pos_max), max(pos_min, pos_max)
        limits[name] = {"id": motor_id, "min": lo, "max": hi}
        print(f"  → saved: {lo} – {hi}  (range={hi - lo} ticks)\n")

    out_path = Path(args.out)
    with open(out_path, "w") as f:
        json.dump({"joint_limits": limits}, f, indent=2)
    print(f"Saved to {out_path}")
    bus.close()


if __name__ == "__main__":
    main()
