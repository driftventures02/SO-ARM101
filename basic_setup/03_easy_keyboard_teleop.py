#!/usr/bin/env python3
"""
Simple direct-key teleop (no limit logic).

Design goal: easy to read in ~5 minutes.

Jargon:
- Teleop: "teleoperation", meaning you control hardware remotely from input keys.
- Goal position: target position we ask the servo to move to.
- Torque: motor holding force (ON = holds/moves, OFF = free to move by hand).
"""

import argparse
import select
import sys
import termios
import time
import tty

from servo_utils import ServoBus, REG_GOAL_POSITION, REG_PRESENT_POSITION, to_u16

MOTOR_IDS = [1, 2, 3, 4, 5, 6]
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# key -> (motor_id, direction)
MOVE_KEYS = {
    "q": (1, +1), "a": (1, -1),
    "w": (2, +1), "s": (2, -1),
    "e": (3, +1), "d": (3, -1),
    "r": (4, +1), "f": (4, -1),
    "t": (5, +1), "g": (5, -1),
    "y": (6, +1), "h": (6, -1),
}


def get_key(timeout=0.02):
    # Read one key without requiring Enter (raw terminal mode).
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_positions(bus: ServoBus):
    # Read each joint's current encoder position.
    out = {}
    for mid in MOTOR_IDS:
        p = bus.read_reg(mid, REG_PRESENT_POSITION, 2)
        if p:
            out[mid] = to_u16(p[0], p[1])
    return out


def sync_goals_to_current(bus: ServoBus, goals: dict[int, int]):
    """Read current position and write it as goal so torque-on does not jump."""
    pos = read_positions(bus)
    for mid in MOTOR_IDS:
        cur = pos.get(mid, goals.get(mid, 2048))
        goals[mid] = cur
        bus.write_reg(mid, REG_GOAL_POSITION, cur, 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--step", type=int, default=20)
    parser.add_argument("--ui-hz", type=float, default=12.0)
    args = parser.parse_args()

    # Open serial connection to the servo bus adapter.
    bus = ServoBus(args.port)
    step = max(1, args.step)
    torque_on = True
    # Start by setting safe goals first, then enable torque.
    goals = read_positions(bus)
    for mid in MOTOR_IDS:
        goals.setdefault(mid, 2048)
    sync_goals_to_current(bus, goals)
    bus.set_torque_all(MOTOR_IDS, True)

    ui_period = 1.0 / max(1.0, args.ui_hz)
    last_ui = 0.0

    try:
        while True:
            now = time.time()
            if now - last_ui >= ui_period:
                pos = read_positions(bus)
                print("\033[H\033[J", end="")
                print(f"Easy Teleop  step={step}  torque={'ON' if torque_on else 'OFF'}  (x quit)")
                print("q/a pan  w/s lift  e/d elbow  r/f wrist_flex  t/g wrist_roll  y/h gripper")
                print("-/= step down/up  space torque  z home(2048)  p print positions\n")
                for i, mid in enumerate(MOTOR_IDS):
                    cur = pos.get(mid, goals[mid])
                    print(f"[{mid}] {NAMES[i]:<14} pos={cur:>4}  goal={goals[mid]:>4}")
                last_ui = now

            key = get_key()
            if not key:
                continue
            if key == "x":
                break
            if key == "-":
                step = max(1, step // 2)
                continue
            if key == "=":
                step = min(512, step * 2)
                continue
            if key == " ":
                # Toggle between active control and free-move mode.
                torque_on = not torque_on
                if torque_on:
                    # Before enabling torque, hold current pose to avoid snap-to-old-goal.
                    sync_goals_to_current(bus, goals)
                bus.set_torque_all(MOTOR_IDS, torque_on)
                continue
            if key == "z":
                for mid in MOTOR_IDS:
                    goals[mid] = 2048
                    if torque_on:
                        bus.write_reg(mid, REG_GOAL_POSITION, 2048, 2)
                continue
            if key == "p":
                print("\n", read_positions(bus))
                continue
            if key in MOVE_KEYS:
                mid, direction = MOVE_KEYS[key]
                # Use live position as the base so movement feels predictable.
                p = bus.read_reg(mid, REG_PRESENT_POSITION, 2)
                base = to_u16(p[0], p[1]) if p else goals[mid]
                goals[mid] = max(0, min(4095, base + direction * step))
                if torque_on:
                    bus.write_reg(mid, REG_GOAL_POSITION, goals[mid], 2)
    except KeyboardInterrupt:
        pass
    finally:
        bus.set_torque_all(MOTOR_IDS, False)
        bus.close()


if __name__ == "__main__":
    main()
