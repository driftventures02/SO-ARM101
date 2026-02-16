#!/usr/bin/env python3
"""
STEP 3: Keyboard teleoperation.

Control the arm from your laptop keyboard. No leader arm needed.

Controls:
  1-6     : Select joint (shown in header)
  ← →     : Move selected joint by small step
  ↑ ↓     : Move selected joint by large step
  a d     : Small move left/right (arrow-key fallback)
  w s     : Large move up/down (arrow-key fallback)
  h j k l : Vim fallback (h/l small, j/k large)
  [ ]     : Decrease/increase step size
  h       : Home all joints to center (2048)
  space   : Toggle torque on/off (when off, you can move arm by hand and read positions)
  p       : Print current positions as a Python dict (for copy-paste into your code)
  v       : Toggle verbose mode (show raw packets)
  q       : Quit

Usage:
    python 03_keyboard_teleop.py --port /dev/tty.usbmodemXXXXX
"""

import serial
import time
import sys
import argparse
import tty
import termios
import select

# ============================================================
# COLORS
# ============================================================
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
CYAN = "\033[96m"
MAGENTA = "\033[95m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"

# ============================================================
# PROTOCOL
# ============================================================
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83

REG_TORQUE_ENABLE = 40
REG_GOAL_POSITION = 42
REG_PRESENT_POSITION = 56

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", 
               "wrist_flex", "wrist_roll", "gripper"]


def build_packet(servo_id, instruction, params=[]):
    length = len(params) + 2
    checksum = ~(servo_id + length + instruction + sum(params)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, instruction] + params + [checksum])


def hex_str(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def send_recv(ser, packet, timeout=0.01, verbose=False):
    ser.reset_input_buffer()
    if verbose:
        print(f"\r  {RED}TX → {hex_str(packet)}{RESET}                    ")
    ser.write(packet)
    ser.flush()
    time.sleep(timeout)
    response = ser.read(ser.in_waiting or 64)
    if response:
        if response[:len(packet)] == packet:
            response = response[len(packet):]
        if response and verbose:
            print(f"\r  {GREEN}RX ← {hex_str(response)}{RESET}                    ")
    return response


def parse_response(data):
    if len(data) < 6 or data[0] != 0xFF or data[1] != 0xFF:
        return None
    servo_id = data[2]
    length = data[3]
    error = data[4]
    params = list(data[5:5 + length - 2]) if length > 2 else []
    return {"id": servo_id, "error": error, "params": params}


def read_positions(ser, motor_ids, verbose=False):
    """Read current position of all motors."""
    positions = {}
    for mid in motor_ids:
        pkt = build_packet(mid, INST_READ, [REG_PRESENT_POSITION, 2])
        resp = send_recv(ser, pkt, verbose=verbose)
        if resp:
            parsed = parse_response(resp)
            if parsed and len(parsed["params"]) >= 2:
                pos = parsed["params"][0] | (parsed["params"][1] << 8)
                positions[mid] = pos
    return positions


def write_position(ser, motor_id, position, verbose=False):
    """
    Write a goal position to a single motor.
    
    This writes to register 42 (Goal_Position), which is a 2-byte register.
    The servo's internal PID controller immediately starts driving toward this position.
    
    The WRITE packet contains:
      [0xFF][0xFF][ID][0x05][0x03][42][pos_lo][pos_hi][checksum]
    
    0x05 = length (instruction + addr + 2 data bytes + checksum = 5)
    0x03 = WRITE instruction
    42 = start address
    pos_lo = position & 0xFF  (little-endian byte order)
    pos_hi = (position >> 8) & 0xFF
    """
    position = max(0, min(4095, position))
    pkt = build_packet(motor_id, INST_WRITE, [
        REG_GOAL_POSITION,
        position & 0xFF,
        (position >> 8) & 0xFF,
    ])
    return send_recv(ser, pkt, verbose=verbose)


def set_torque(ser, motor_ids, enable, verbose=False):
    """
    Enable or disable torque.
    
    When torque is OFF:
      - Motor is free to rotate (backdrivable)
      - You can still READ position (encoder works without power to the motor)
      - You CANNOT write goal positions (they'll be ignored)
    
    When torque is ON:
      - Motor holds its current position (PID active)
      - You can write new goal positions
    
    This is actually the "Torque Switch" register at address 40.
    Writing 1 enables, 0 disables.
    """
    for mid in motor_ids:
        value = 1 if enable else 0
        pkt = build_packet(mid, INST_WRITE, [REG_TORQUE_ENABLE, value])
        send_recv(ser, pkt, verbose=verbose)


def write_all_positions(ser, motor_ids, positions, verbose=False):
    """
    Write goal positions to all motors using individual writes.
    (SYNC_WRITE would be more efficient but individual writes let us
    see each packet separately for learning purposes.)
    """
    for mid in motor_ids:
        if mid in positions:
            write_position(ser, mid, positions[mid], verbose=verbose)


def get_key():
    """Non-blocking key read on Mac/Linux using raw terminal mode."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            ch = sys.stdin.read(1)
            # Handle arrow keys (escape sequences)
            if ch == '\x1b':
                ch2 = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0.01)[0] else ''
                ch3 = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0.01)[0] else ''
                if ch2 == '[':
                    if ch3 == 'A': return 'UP'
                    if ch3 == 'B': return 'DOWN'
                    if ch3 == 'C': return 'RIGHT'
                    if ch3 == 'D': return 'LEFT'
                return 'ESC'
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def draw_joint_bar(pos, is_selected=False, width=40):
    """Draw a position bar with marker."""
    fraction = pos / 4096
    marker_pos = int(fraction * width)
    bar_chars = list("─" * width)
    
    # Mark center (2048)
    center = width // 2
    bar_chars[center] = "┼"
    
    # Mark current position
    marker_pos = max(0, min(width - 1, marker_pos))
    if is_selected:
        bar_chars[marker_pos] = "█"
    else:
        bar_chars[marker_pos] = "●"
    
    return "".join(bar_chars)


def main():
    parser = argparse.ArgumentParser(description="Keyboard teleop for SO-ARM101")
    parser.add_argument("--port", required=True)
    args = parser.parse_args()
    
    motor_ids = [1, 2, 3, 4, 5, 6]
    
    print(f"{BOLD}Connecting to {args.port}...{RESET}")
    try:
        ser = serial.Serial(port=args.port, baudrate=1_000_000, timeout=0.1)
    except serial.SerialException as e:
        print(f"{RED}Failed: {e}{RESET}")
        sys.exit(1)
    
    time.sleep(0.2)
    
    # Read initial positions
    goal_positions = read_positions(ser, motor_ids)
    if not goal_positions:
        print(f"{RED}No motors responding! Check connections.{RESET}")
        ser.close()
        sys.exit(1)
    
    # Fill in any missing
    for mid in motor_ids:
        if mid not in goal_positions:
            goal_positions[mid] = 2048
    
    print(f"{GREEN}Connected! Found {len(goal_positions)} motors.{RESET}")
    
    selected_joint = 0  # Index into JOINT_NAMES (0-5)
    step_size = 50      # Position steps per keypress
    torque_on = True
    verbose = False
    
    # Enable torque
    set_torque(ser, motor_ids, True, verbose=verbose)
    
    try:
        while True:
            # Read current positions
            current_positions = read_positions(ser, motor_ids, verbose=verbose)
            
            if not verbose:
                print("\033[H\033[J", end="")
            
            # Header
            print(f"{BOLD}{'='*75}")
            print(f"  KEYBOARD TELEOP  |  Step: {step_size}  |  Torque: {'ON' if torque_on else 'OFF'}  |  Verbose: {'ON' if verbose else 'OFF'}")
            print(f"{'='*75}{RESET}\n")
            
            # Joint display
            for i, (mid, name) in enumerate(zip(motor_ids, JOINT_NAMES)):
                is_sel = (i == selected_joint)
                pos = current_positions.get(mid, goal_positions.get(mid, 2048))
                goal = goal_positions.get(mid, 2048)
                deg = (pos / 4096) * 360
                
                prefix = f"  {BOLD}{CYAN}▶ " if is_sel else f"  {DIM}  "
                bar = draw_joint_bar(pos, is_selected=is_sel)
                
                sel_color = CYAN if is_sel else DIM
                
                print(f"{prefix}[{i+1}] {name:<15} pos={pos:>5} ({deg:>6.1f}°) "
                      f"goal={goal:>5}  {sel_color}{bar}{RESET}")
            
            # Controls
            print(f"""
  {DIM}{'─'*75}{RESET}
  {BOLD}Controls:{RESET}
    {CYAN}1-6{RESET} select joint   {CYAN}← →{RESET} small move   {CYAN}↑ ↓{RESET} big move   {CYAN}[ ]{RESET} step size
    {CYAN}h{RESET}   home (center)  {CYAN}space{RESET} torque on/off  {CYAN}p{RESET} print positions
    {CYAN}v{RESET}   verbose mode   {CYAN}q{RESET}   quit
""")
            
            if verbose:
                print(f"  {DIM}(Verbose ON — raw packets shown above){RESET}\n")
            
            # Get keypress
            key = get_key()
            if key is None:
                continue
            
            mid = motor_ids[selected_joint]
            
            if key == 'q':
                break
            elif key in '123456':
                selected_joint = int(key) - 1
            elif key in ('RIGHT', 'd', 'l'):
                goal_positions[mid] = min(4095, goal_positions[mid] + step_size)
                if torque_on:
                    write_position(ser, mid, goal_positions[mid], verbose=verbose)
            elif key in ('LEFT', 'a'):
                goal_positions[mid] = max(0, goal_positions[mid] - step_size)
                if torque_on:
                    write_position(ser, mid, goal_positions[mid], verbose=verbose)
            elif key in ('UP', 'w', 'k'):
                goal_positions[mid] = min(4095, goal_positions[mid] + step_size * 5)
                if torque_on:
                    write_position(ser, mid, goal_positions[mid], verbose=verbose)
            elif key in ('DOWN', 's', 'j'):
                goal_positions[mid] = max(0, goal_positions[mid] - step_size * 5)
                if torque_on:
                    write_position(ser, mid, goal_positions[mid], verbose=verbose)
            elif key == '[':
                step_size = max(1, step_size // 2)
            elif key == ']':
                step_size = min(500, step_size * 2)
            elif key == 'h':
                # Home all joints
                for mid in motor_ids:
                    goal_positions[mid] = 2048
                if torque_on:
                    write_all_positions(ser, motor_ids, goal_positions, verbose=verbose)
            elif key == ' ':
                torque_on = not torque_on
                set_torque(ser, motor_ids, torque_on, verbose=verbose)
                if torque_on:
                    # Re-read positions so goals match current physical state
                    goal_positions = read_positions(ser, motor_ids, verbose=verbose)
                    for mid in motor_ids:
                        if mid not in goal_positions:
                            goal_positions[mid] = 2048
            elif key == 'p':
                # Print positions as Python dict
                pos_dict = {JOINT_NAMES[i]: current_positions.get(motor_ids[i], 0) 
                           for i in range(6)}
                print(f"\n  {GREEN}positions = {pos_dict}{RESET}")
                input(f"  {DIM}(Press Enter to continue){RESET}")
            elif key == 'v':
                verbose = not verbose
    
    except KeyboardInterrupt:
        pass
    finally:
        # Disable torque before exit so arm can be moved freely
        print(f"\n{YELLOW}Disabling torque and closing...{RESET}")
        set_torque(ser, motor_ids, False)
        ser.close()
        print(f"{GREEN}Done.{RESET}")


if __name__ == "__main__":
    main()
