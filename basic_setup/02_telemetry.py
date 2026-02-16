#!/usr/bin/env python3
"""
STEP 2: Live telemetry dashboard.

After assembling and configuring all 6 motors, run this to see real-time
readings from all servos: position, speed, load, voltage, temperature.

Shows raw packets if --verbose flag is set.

Usage:
    python 02_telemetry.py --port /dev/tty.usbmodemXXXXX
    python 02_telemetry.py --port /dev/tty.usbmodemXXXXX --verbose  # show raw bytes
    python 02_telemetry.py --port /dev/tty.usbmodemXXXXX --hz 10     # update rate
"""

import serial
import time
import argparse
import sys
import os

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
INST_SYNC_READ = 0x82  # Sync read — read same register from multiple motors at once

JOINT_NAMES = {
    1: "shoulder_pan ",
    2: "shoulder_lift",
    3: "elbow_flex   ",
    4: "wrist_flex   ",
    5: "wrist_roll   ",
    6: "gripper      ",
}


def build_packet(servo_id, instruction, params=[]):
    length = len(params) + 2
    checksum = ~(servo_id + length + instruction + sum(params)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, instruction] + params + [checksum])


def build_sync_read(motor_ids, start_address, data_length):
    """
    Build a SYNC_READ packet.
    
    This is the efficient way to read the same register from multiple motors.
    Instead of 6 individual READ commands (6 TX + 6 RX = 12 packets),
    we send ONE sync_read command and get 6 responses.
    
    Packet: [0xFF][0xFF][0xFE][len][0x82][start_addr][data_len][id1][id2]...[checksum]
    
    0xFE = broadcast ID (all motors listen)
    0x82 = SYNC_READ instruction
    
    Each motor responds in order of its ID with a standard response packet.
    """
    params = [start_address, data_length] + list(motor_ids)
    return build_packet(0xFE, INST_SYNC_READ, params)


def hex_str(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def parse_response(data: bytes) -> dict | None:
    if len(data) < 6 or data[0] != 0xFF or data[1] != 0xFF:
        return None
    servo_id = data[2]
    length = data[3]
    error = data[4]
    params = list(data[5:5 + length - 2]) if length > 2 else []
    total_len = 4 + length  # header(2) + id(1) + length(1) + [length bytes]
    return {"id": servo_id, "error": error, "params": params, "total_len": total_len}


def read_all_telemetry(ser, motor_ids, verbose=False):
    """
    Read position (2 bytes), speed (2 bytes), load (2 bytes), voltage (1 byte), temp (1 byte)
    from all motors. That's registers 56-63 = 8 contiguous bytes per motor.
    
    We use SYNC_READ to get all of them in one bus transaction.
    """
    start_addr = 56  # Present_Position
    data_len = 8     # 8 bytes: pos(2) + speed(2) + load(2) + voltage(1) + temp(1)
    
    pkt = build_sync_read(motor_ids, start_addr, data_len)
    
    if verbose:
        print(f"  {RED}TX → {hex_str(pkt)}{RESET}")
    
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    
    # Each motor responds with a packet: [FF FF] [ID] [len] [error] [8 data bytes] [checksum]
    # That's 6 + 8 = 14 bytes per motor (header=2, id=1, len=1, error=1, data=8, checksum=1 → actually 4+len = 4+10 = 14... wait)
    # len = data_len + 2 = 10, so total per motor = 2 + 1 + 1 + 10 = 14 bytes? No:
    # [FF FF ID LEN ERR D0 D1 D2 D3 D4 D5 D6 D7 CKSM] = 14 bytes per motor
    # Actually LEN = 8+2 = 10, and total = 4 + LEN = 4 + 10 = 14. Yes, 14 bytes per motor.
    
    expected_bytes = 14 * len(motor_ids)
    time.sleep(0.01 + expected_bytes / 1_000_000 * 10)  # UART time + margin
    
    response = ser.read(ser.in_waiting or expected_bytes * 2)
    
    # Sometimes we get our own TX echoed back on half-duplex
    if response[:len(pkt)] == pkt:
        response = response[len(pkt):]
    
    if verbose and response:
        print(f"  {GREEN}RX ← {hex_str(response)}{RESET}")
    
    # Parse individual motor responses from the concatenated stream
    results = {}
    offset = 0
    
    for _ in motor_ids:
        # Find next response header
        while offset < len(response) - 1:
            if response[offset] == 0xFF and response[offset + 1] == 0xFF:
                break
            offset += 1
        
        if offset + 14 > len(response):
            break
        
        chunk = response[offset:offset + 14]
        parsed = parse_response(chunk)
        
        if parsed and len(parsed["params"]) >= 8:
            p = parsed["params"]
            motor_id = parsed["id"]
            
            position = p[0] | (p[1] << 8)
            speed_raw = p[2] | (p[3] << 8)
            load_raw = p[4] | (p[5] << 8)
            voltage = p[6] / 10.0
            temperature = p[7]
            
            # Speed is signed: bit 15 = direction, bits 0-14 = magnitude
            speed_sign = -1 if speed_raw & 0x8000 else 1
            speed = speed_sign * (speed_raw & 0x7FFF)
            
            # Load is signed: bit 10 = direction, bits 0-9 = magnitude (0-1023)
            load_sign = -1 if load_raw & 0x0400 else 1
            load = load_sign * (load_raw & 0x03FF)
            load_pct = (abs(load) / 1023.0) * 100
            
            results[motor_id] = {
                "position": position,
                "degrees": (position / 4096) * 360,
                "speed": speed,
                "load": load,
                "load_pct": load_pct,
                "voltage": voltage,
                "temperature": temperature,
                "error": parsed["error"],
            }
            
            offset += 14  # Move past this response
        else:
            offset += 1
    
    return results


def draw_position_bar(position, width=30):
    """Draw a visual bar showing position within 0-4096 range."""
    fraction = position / 4096
    filled = int(fraction * width)
    bar = "█" * filled + "░" * (width - filled)
    return bar


def draw_load_bar(load_pct, width=15):
    """Draw a centered load bar."""
    half = width // 2
    if load_pct == 0:
        return " " * half + "│" + " " * half
    chars = int((load_pct / 100) * half)
    chars = max(1, min(chars, half))
    bar = " " * (half - chars) + "█" * chars + "│" + "█" * chars + " " * (half - chars)
    return bar


def main():
    parser = argparse.ArgumentParser(description="Live telemetry from SO-ARM101 servos")
    parser.add_argument("--port", required=True)
    parser.add_argument("--verbose", action="store_true", help="Show raw packets")
    parser.add_argument("--hz", type=float, default=20, help="Update rate in Hz")
    args = parser.parse_args()

    motor_ids = [1, 2, 3, 4, 5, 6]
    
    print(f"{BOLD}Connecting to {args.port} at 1,000,000 baud...{RESET}")
    
    try:
        ser = serial.Serial(port=args.port, baudrate=1_000_000, timeout=0.1)
    except serial.SerialException as e:
        print(f"{RED}Failed: {e}{RESET}")
        sys.exit(1)
    
    time.sleep(0.2)
    print(f"{GREEN}Connected!{RESET}\n")
    
    period = 1.0 / args.hz
    loop_count = 0
    last_time = time.time()
    actual_hz = 0
    
    try:
        while True:
            t0 = time.time()
            
            results = read_all_telemetry(ser, motor_ids, verbose=args.verbose)
            
            # Calculate actual Hz
            loop_count += 1
            now = time.time()
            if now - last_time >= 1.0:
                actual_hz = loop_count / (now - last_time)
                loop_count = 0
                last_time = now
            
            if not args.verbose:
                # Clear screen and redraw
                print("\033[H\033[J", end="")  # ANSI: move cursor home, clear screen
            
            print(f"{BOLD}{'='*90}")
            print(f"  SO-ARM101 LIVE TELEMETRY  |  {actual_hz:.1f} Hz  |  Port: {args.port}  |  Ctrl+C to quit")
            print(f"{'='*90}{RESET}\n")
            
            # Header
            print(f"  {'Joint':<16} {'ID':>3} {'Pos':>5} {'Degrees':>8} {'Speed':>7} {'Load%':>7} {'Volts':>6} {'Temp':>5} {'Err':>4}  Position Bar")
            print(f"  {'─'*16} {'─'*3} {'─'*5} {'─'*8} {'─'*7} {'─'*7} {'─'*6} {'─'*5} {'─'*4}  {'─'*30}")
            
            for mid in motor_ids:
                if mid in results:
                    r = results[mid]
                    name = JOINT_NAMES.get(mid, f"motor_{mid}")
                    
                    # Color code: red if error, yellow if hot, green if ok
                    color = GREEN
                    if r["error"] != 0:
                        color = RED
                    elif r["temperature"] > 60:
                        color = YELLOW
                    elif r["voltage"] < 4.5:
                        color = YELLOW
                    
                    bar = draw_position_bar(r["position"])
                    
                    print(f"  {color}{name:<16} {mid:>3} {r['position']:>5} {r['degrees']:>7.1f}° "
                          f"{r['speed']:>+7d} {r['load_pct']:>6.1f}% "
                          f"{r['voltage']:>5.1f}V {r['temperature']:>4d}°C "
                          f"0x{r['error']:02X}  {CYAN}{bar}{RESET}")
                else:
                    name = JOINT_NAMES.get(mid, f"motor_{mid}")
                    print(f"  {RED}{name:<16} {mid:>3} {'---':>5} {'---':>8} "
                          f"{'---':>7} {'---':>7} {'---':>6} {'---':>5} {'---':>4}  "
                          f"{'NO RESPONSE':^30}{RESET}")
            
            # Summary stats
            if results:
                voltages = [r["voltage"] for r in results.values()]
                temps = [r["temperature"] for r in results.values()]
                print(f"\n  {DIM}Voltage range: {min(voltages):.1f}V - {max(voltages):.1f}V  |  "
                      f"Temp range: {min(temps)}°C - {max(temps)}°C  |  "
                      f"Motors responding: {len(results)}/6{RESET}")
            
            if args.verbose:
                print(f"\n  {DIM}{'─'*80}{RESET}\n")
            
            # Maintain loop rate
            elapsed = time.time() - t0
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print(f"\n\n{YELLOW}Telemetry stopped.{RESET}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
