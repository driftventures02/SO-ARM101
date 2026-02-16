#!/usr/bin/env python3
"""
STEP 3 (ALT): Easy keyboard teleoperation with direct key mapping.

No joint switching. Each joint always has fixed +/- keys:
  q/a : shoulder_pan   +/-
  w/s : shoulder_lift  +/-
  e/d : elbow_flex     +/-
  r/f : wrist_flex     +/-
  t/g : wrist_roll     +/-
  y/h : gripper        +/-

Other controls:
  - / = : decrease/increase step size
  space : torque off/on
  z     : home all joints to 2048
  p     : print current positions
  x     : quit

Usage:
  uv run python files/03_easy_keyboard_teleop.py --port /dev/cu.usbmodemXXXXX
"""

import argparse
import select
import serial
import sys
import termios
import time
import tty

# Colors
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"

# Protocol
INST_READ = 0x02
INST_WRITE = 0x03

REG_TORQUE_ENABLE = 40
REG_GOAL_POSITION = 42
REG_PRESENT_POSITION = 56

MOTOR_IDS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# key -> (motor_id, direction)
MOVE_KEYS = {
    "q": (1, +1), "a": (1, -1),
    "w": (2, +1), "s": (2, -1),
    "e": (3, +1), "d": (3, -1),
    "r": (4, +1), "f": (4, -1),
    "t": (5, +1), "g": (5, -1),
    "y": (6, +1), "h": (6, -1),
}


def build_packet(servo_id: int, instruction: int, params: list[int]) -> bytes:
    length = len(params) + 2
    checksum = ~(servo_id + length + instruction + sum(params)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, instruction] + params + [checksum])


def send_recv(ser: serial.Serial, packet: bytes, timeout: float = 0.01) -> bytes:
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    time.sleep(timeout)
    response = ser.read(ser.in_waiting or 64)
    if response and response[:len(packet)] == packet:
        response = response[len(packet):]
    return response


def send_no_wait(ser: serial.Serial, packet: bytes):
    """Fast path for WRITE commands: send immediately, don't wait for status packet."""
    ser.write(packet)
    ser.flush()


def parse_response(data: bytes):
    if len(data) < 6 or data[0] != 0xFF or data[1] != 0xFF:
        return None
    length = data[3]
    params = list(data[5:5 + length - 2]) if length > 2 else []
    return {"id": data[2], "error": data[4], "params": params}


def read_positions(ser: serial.Serial) -> dict[int, int]:
    positions = {}
    for mid in MOTOR_IDS:
        pkt = build_packet(mid, INST_READ, [REG_PRESENT_POSITION, 2])
        resp = send_recv(ser, pkt)
        parsed = parse_response(resp) if resp else None
        if parsed and len(parsed["params"]) >= 2:
            positions[mid] = parsed["params"][0] | (parsed["params"][1] << 8)
    return positions


def write_position(ser: serial.Serial, motor_id: int, pos: int):
    pos = max(0, min(4095, pos))
    pkt = build_packet(motor_id, INST_WRITE, [REG_GOAL_POSITION, pos & 0xFF, (pos >> 8) & 0xFF])
    send_no_wait(ser, pkt)


def set_torque(ser: serial.Serial, enable: bool):
    value = 1 if enable else 0
    for mid in MOTOR_IDS:
        pkt = build_packet(mid, INST_WRITE, [REG_TORQUE_ENABLE, value])
        send_no_wait(ser, pkt)


def draw_bar(pos: int, width: int = 34) -> str:
    marker = max(0, min(width - 1, int((pos / 4095.0) * (width - 1))))
    center = width // 2
    chars = list("─" * width)
    chars[center] = "┼"
    chars[marker] = "●"
    return "".join(chars)


def get_key(timeout: float = 0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    parser = argparse.ArgumentParser(description="Easy keyboard teleop for SO-ARM101")
    parser.add_argument("--port", required=True)
    parser.add_argument("--ui-hz", type=float, default=10.0, help="Display refresh rate (higher = smoother, lower = less overhead)")
    parser.add_argument("--poll-hz", type=float, default=5.0, help="How often to read actual motor positions")
    args = parser.parse_args()

    print(f"{BOLD}Connecting to {args.port}...{RESET}")
    try:
        ser = serial.Serial(port=args.port, baudrate=1_000_000, timeout=0.1)
    except serial.SerialException as e:
        print(f"{RED}Failed: {e}{RESET}")
        sys.exit(1)
    time.sleep(0.2)

    goal_positions = read_positions(ser)
    if len(goal_positions) < 6:
        print(f"{YELLOW}Warning: only {len(goal_positions)}/6 motors responded.{RESET}")
    for mid in MOTOR_IDS:
        goal_positions.setdefault(mid, 2048)

    torque_on = True
    step_size = 20
    set_torque(ser, True)
    current_positions = dict(goal_positions)
    ui_period = 1.0 / max(1.0, args.ui_hz)
    poll_period = 1.0 / max(0.5, args.poll_hz)
    last_ui = 0.0
    last_poll = 0.0

    try:
        while True:
            now = time.time()

            # Poll actual positions at a lower rate so keypress handling stays snappy.
            if now - last_poll >= poll_period:
                polled = read_positions(ser)
                if polled:
                    current_positions.update(polled)
                last_poll = now

            # Refresh UI separately from command handling.
            if now - last_ui >= ui_period:
                print("\033[H\033[J", end="")
                print(f"{BOLD}{'='*84}")
                print(f"  EASY TELEOP  |  Step: {step_size}  |  Torque: {'ON' if torque_on else 'OFF'}")
                print(f"{'='*84}{RESET}")
                print(f"  {CYAN}q/a{RESET} pan  {CYAN}w/s{RESET} lift  {CYAN}e/d{RESET} elbow  {CYAN}r/f{RESET} wrist_flex  {CYAN}t/g{RESET} wrist_roll  {CYAN}y/h{RESET} gripper")
                print(f"  {CYAN}- / ={RESET} step down/up   {CYAN}space{RESET} torque toggle   {CYAN}z{RESET} home   {CYAN}p{RESET} print pos   {CYAN}x{RESET} quit\n")

                for i, mid in enumerate(MOTOR_IDS):
                    pos = current_positions.get(mid, goal_positions[mid])
                    goal = goal_positions[mid]
                    print(
                        f"  [{mid}] {JOINT_NAMES[i]:<14} pos={pos:>4} ({(pos/4096)*360:>6.1f}°) "
                        f"goal={goal:>4}  {DIM}{draw_bar(pos)}{RESET}"
                    )
                last_ui = now

            key = get_key(timeout=0.01)
            if key is None:
                continue

            if key == "x":
                break
            if key in MOVE_KEYS:
                mid, direction = MOVE_KEYS[key]
                goal_positions[mid] = max(0, min(4095, goal_positions[mid] + direction * step_size))
                current_positions[mid] = goal_positions[mid]
                if torque_on:
                    write_position(ser, mid, goal_positions[mid])
                continue
            if key == "-":
                step_size = max(1, step_size // 2)
                continue
            if key == "=":
                step_size = min(512, step_size * 2)
                continue
            if key == " ":
                torque_on = not torque_on
                set_torque(ser, torque_on)
                if torque_on:
                    goal_positions = read_positions(ser)
                    for mid in MOTOR_IDS:
                        goal_positions.setdefault(mid, 2048)
                    current_positions.update(goal_positions)
                continue
            if key == "z":
                for mid in MOTOR_IDS:
                    goal_positions[mid] = 2048
                    current_positions[mid] = 2048
                    if torque_on:
                        write_position(ser, mid, 2048)
                continue
            if key == "p":
                current = read_positions(ser)
                pos_dict = {JOINT_NAMES[i]: current.get(mid, 0) for i, mid in enumerate(MOTOR_IDS)}
                print(f"\n{GREEN}positions = {pos_dict}{RESET}")
                input(f"{DIM}Press Enter to continue...{RESET}")

    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n{YELLOW}Disabling torque and closing...{RESET}")
        set_torque(ser, False)
        ser.close()
        print(f"{GREEN}Done.{RESET}")


if __name__ == "__main__":
    main()
