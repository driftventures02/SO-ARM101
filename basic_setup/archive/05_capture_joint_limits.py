#!/usr/bin/env python3
"""
Capture per-joint safe min/max limits by hand and save to JSON.

Workflow:
1) Torque is disabled so you can move joints by hand.
2) For each joint, move to MIN safe position and press Enter.
3) Move to MAX safe position and press Enter.
4) Script writes a limits JSON file for teleop clamping.

Usage:
  uv run python basic_setup/05_capture_joint_limits.py --port /dev/cu.usbmodemXXXXX
"""

import argparse
import json
import serial
import time
from pathlib import Path

INST_READ = 0x02
INST_WRITE = 0x03

REG_TORQUE_ENABLE = 40
REG_PRESENT_POSITION = 56

MOTOR_IDS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def build_packet(servo_id: int, instruction: int, params: list[int]) -> bytes:
    length = len(params) + 2
    checksum = ~(servo_id + length + instruction + sum(params)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, instruction] + params + [checksum])


def send_recv(ser: serial.Serial, packet: bytes, timeout: float = 0.02) -> bytes:
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    time.sleep(timeout)
    response = ser.read(ser.in_waiting or 64)
    if response and response[:len(packet)] == packet:
        response = response[len(packet):]
    return response


def parse_response(data: bytes):
    if len(data) < 6 or data[0] != 0xFF or data[1] != 0xFF:
        return None
    length = data[3]
    params = list(data[5:5 + length - 2]) if length > 2 else []
    return {"id": data[2], "error": data[4], "params": params}


def read_position(ser: serial.Serial, motor_id: int) -> int | None:
    pkt = build_packet(motor_id, INST_READ, [REG_PRESENT_POSITION, 2])
    resp = send_recv(ser, pkt)
    parsed = parse_response(resp) if resp else None
    if parsed and len(parsed["params"]) >= 2:
        return parsed["params"][0] | (parsed["params"][1] << 8)
    return None


def set_torque_all(ser: serial.Serial, enable: bool):
    value = 1 if enable else 0
    for motor_id in MOTOR_IDS:
        pkt = build_packet(motor_id, INST_WRITE, [REG_TORQUE_ENABLE, value])
        send_recv(ser, pkt, timeout=0.01)


def main():
    parser = argparse.ArgumentParser(description="Capture safe per-joint limits")
    parser.add_argument("--port", required=True)
    parser.add_argument("--out", default=str(Path(__file__).with_name("joint_limits.json")))
    parser.add_argument("--margin", type=int, default=25, help="Ticks pulled inward from each measured limit")
    args = parser.parse_args()

    print(f"Connecting to {args.port}...")
    ser = serial.Serial(port=args.port, baudrate=1_000_000, timeout=0.1)
    time.sleep(0.2)

    try:
        print("\nDisabling torque so joints can be moved by hand...")
        set_torque_all(ser, False)
        time.sleep(0.1)

        limits = {}
        print("\nCapture each joint carefully. Stay away from hard stops.")
        print(f"Measured limits will be tightened by margin={args.margin} ticks.\n")

        for motor_id, name in zip(MOTOR_IDS, JOINT_NAMES):
            input(f"[{motor_id}] {name}: move to MIN safe position, then press Enter...")
            pmin = read_position(ser, motor_id)
            input(f"[{motor_id}] {name}: move to MAX safe position, then press Enter...")
            pmax = read_position(ser, motor_id)

            if pmin is None or pmax is None:
                print(f"  Warning: could not read {name}, using full range.")
                pmin, pmax = 0, 4095

            lo, hi = (pmin, pmax) if pmin <= pmax else (pmax, pmin)
            lo = max(0, lo + args.margin)
            hi = min(4095, hi - args.margin)
            if lo >= hi:
                lo, hi = 0, 4095

            limits[name] = {"id": motor_id, "min": lo, "max": hi}
            print(f"  saved {name}: min={lo}, max={hi}\n")

        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w", encoding="ascii") as f:
            json.dump({"joint_limits": limits}, f, indent=2)

        print(f"Saved limits file: {out_path}")
        print("You can now use this file in easy teleop with --limits-file.")

    finally:
        # Keep torque off on exit for safety.
        set_torque_all(ser, False)
        ser.close()


if __name__ == "__main__":
    main()
