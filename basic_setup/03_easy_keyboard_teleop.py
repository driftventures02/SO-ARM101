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

from servo_utils import (
    ServoBus,
    REG_ACCELERATION,
    REG_GOAL_POSITION,
    REG_PRESENT_POSITION,
    to_u16,
)

MOTOR_IDS = [1, 2, 3, 4, 5, 6]
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# key -> (motor_id, direction)
MOVE_KEYS = {
    "q": (1, +1), "a": (1, -1),
    "w": (2, +1), "s": (2, -1),
    "d": (3, +1), "e": (3, -1),
    "f": (4, +1), "r": (4, -1),
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
    # Fast path: one SYNC_READ call for all motors.
    out = {}
    packets = bus.sync_read(MOTOR_IDS, REG_PRESENT_POSITION, 2)
    for mid in MOTOR_IDS:
        pkt = packets.get(mid)
        if pkt and len(pkt["params"]) >= 2:
            out[mid] = to_u16(pkt["params"][0], pkt["params"][1])
    return out


def sync_goals_to_current(bus: ServoBus, goals: dict[int, int]):
    """Read current position and write it as goal so torque-on does not jump."""
    pos = read_positions(bus)
    for motor_id in MOTOR_IDS:
        cur = pos.get(motor_id, goals.get(motor_id, 2048))
        goals[motor_id] = cur
        bus.write_reg(motor_id, REG_GOAL_POSITION, cur, 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--step", type=int, default=20)
    parser.add_argument("--ui-hz", type=float, default=12.0)
    parser.add_argument("--accel", type=int, default=20, help="Servo acceleration register (0..255). Higher is smoother.")
    parser.add_argument("--control-hz", type=float, default=60.0, help="Velocity update rate while key is held")
    parser.add_argument("--hold-ms", type=float, default=120.0, help="How long to treat a key as held after last repeat event")
    args = parser.parse_args()

    # Open serial connection to the servo bus adapter.
    bus = ServoBus(args.port)
    step = max(1, args.step)
    tap_step = max(1, step // 2)
    vel_step = max(1, step // 6)
    accel = max(0, min(255, args.accel))
    control_hz = max(10.0, args.control_hz)
    control_period = 1.0 / control_hz
    hold_s = max(0.02, args.hold_ms / 1000.0)
    torque_on = True
    # Start by setting safe goals first, then enable torque.
    goals = read_positions(bus)
    for mid in MOTOR_IDS:
        goals.setdefault(mid, 2048)
    # Hardware-side smoothing: non-zero acceleration softens start/stop.
    for mid in MOTOR_IDS:
        bus.write_reg(mid, REG_ACCELERATION, accel, 1)
        time.sleep(0.002)
    sync_goals_to_current(bus, goals)
    bus.set_torque_all(MOTOR_IDS, True)

    ui_period = 1.0 / max(1.0, args.ui_hz)
    last_ui = 0.0
    last_control = 0.0
    held_dir = {mid: 0 for mid in MOTOR_IDS}
    held_until = {mid: 0.0 for mid in MOTOR_IDS}

    try:
        while True:
            now = time.time()
            # Use a tiny blocking poll so terminal keypresses are captured reliably.
            key = get_key(timeout=0.005)
            if key == "x":
                return
            if key == "-":
                step = max(1, step // 2)
                tap_step = max(1, step // 2)
                vel_step = max(1, step // 6)
            elif key == "=":
                step = min(512, step * 2)
                tap_step = max(1, step // 2)
                vel_step = max(1, step // 6)
            elif key == " ":
                torque_on = not torque_on
                if torque_on:
                    sync_goals_to_current(bus, goals)
                bus.set_torque_all(MOTOR_IDS, torque_on)
            elif key == "z":
                # Safety behavior: "z" now means hold current pose,
                # not "force all joints to 2048".
                sync_goals_to_current(bus, goals)
                for mid in MOTOR_IDS:
                    held_dir[mid] = 0
            elif key == "p":
                print("\n", read_positions(bus))
            elif key in MOVE_KEYS:
                mid, direction = MOVE_KEYS[key]
                # Immediate tap response.
                goals[mid] = max(0, min(4095, goals[mid] + direction * tap_step))
                if torque_on:
                    bus.write_reg(mid, REG_GOAL_POSITION, goals[mid], 2)
                # Keep moving while key-repeat events continue.
                held_dir[mid] = direction
                held_until[mid] = now + hold_s

            # Velocity loop while a key is held.
            if now - last_control >= control_period:
                for mid in MOTOR_IDS:
                    if held_until[mid] >= now and held_dir[mid] != 0:
                        goals[mid] = max(0, min(4095, goals[mid] + held_dir[mid] * vel_step))
                        if torque_on:
                            bus.write_reg(mid, REG_GOAL_POSITION, goals[mid], 2)
                    else:
                        held_dir[mid] = 0
                last_control = now

            if now - last_ui >= ui_period:
                pos = read_positions(bus)
                print("\033[H\033[J", end="")
                print(
                    f"Easy Teleop  step={step}  accel={accel}  "
                    f"torque={'ON' if torque_on else 'OFF'}  (x quit)"
                )
                print("q/a pan  w/s lift  e/d elbow  r/f wrist_flex  t/g wrist_roll  y/h gripper")
                print("-/= step down/up  space torque  z hold-current  p print positions")
                print(f"velocity: control_hz={control_hz:.0f}  tap_step={tap_step}  vel_step={vel_step}\n")
                for i, mid in enumerate(MOTOR_IDS):
                    cur = pos.get(mid, goals[mid])
                    print(f"[{mid}] {NAMES[i]:<14} pos={cur:>4}  goal={goals[mid]:>4}")
                last_ui = now

            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        bus.set_torque_all(MOTOR_IDS, False)
        bus.close()


if __name__ == "__main__":
    main()
