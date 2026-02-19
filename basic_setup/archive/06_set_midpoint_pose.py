#!/usr/bin/env python3
"""
Set current physical pose as midpoint (2048) for all joints.

This uses STS3215 one-key midpoint calibration by writing 128 to register 40.

Typical workflow:
1) Assemble arm.
2) Run this script.
3) Move shoulder_pan to desired center and other joints to your neutral "straight up" pose.
4) Press Enter to set that pose as midpoint for all connected motors.
5) Verify each joint reads near 2048.

Usage:
  uv run python basic_setup/06_set_midpoint_pose.py --port /dev/cu.usbmodemXXXXX
"""

import argparse
import serial
import time

INST_READ = 0x02
INST_WRITE = 0x03

REG_CALIBRATION = 40       # write 128 => set current position as midpoint (2048)
REG_PRESENT_POSITION = 56

MOTOR_IDS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def build_packet(servo_id: int, instruction: int, params: list[int]) -> bytes:
    length = len(params) + 2
    checksum = ~(servo_id + length + instruction + sum(params)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, instruction] + params + [checksum])


def send_recv(ser: serial.Serial, packet: bytes, timeout: float = 0.04) -> bytes:
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    time.sleep(timeout)
    response = ser.read(ser.in_waiting or 64)
    if response and response[: len(packet)] == packet:
        response = response[len(packet):]
    return response


def parse_response(data: bytes):
    if len(data) < 6 or data[0] != 0xFF or data[1] != 0xFF:
        return None
    length = data[3]
    params = list(data[5:5 + length - 2]) if length > 2 else []
    return {"id": data[2], "error": data[4], "params": params}


def read_pos(ser: serial.Serial, motor_id: int):
    pkt = build_packet(motor_id, INST_READ, [REG_PRESENT_POSITION, 2])
    resp = send_recv(ser, pkt)
    parsed = parse_response(resp) if resp else None
    if parsed and len(parsed["params"]) >= 2:
        return parsed["params"][0] | (parsed["params"][1] << 8)
    return None


def set_midpoint_here(ser: serial.Serial, motor_id: int):
    pkt = build_packet(motor_id, INST_WRITE, [REG_CALIBRATION, 128])
    resp = send_recv(ser, pkt)
    parsed = parse_response(resp) if resp else None
    return parsed is not None and parsed["error"] == 0


def main():
    parser = argparse.ArgumentParser(description="Set current arm pose as midpoint (2048)")
    parser.add_argument("--port", required=True)
    parser.add_argument("--ids", default="1,2,3,4,5,6", help="Comma-separated motor IDs to calibrate")
    args = parser.parse_args()

    motor_ids = []
    for token in args.ids.split(","):
        token = token.strip()
        if not token:
            continue
        try:
            motor_ids.append(int(token))
        except ValueError:
            pass
    motor_ids = [motor_id for motor_id in motor_ids if 1 <= motor_id <= 253]
    if not motor_ids:
        print("No valid IDs provided.")
        return

    ser = serial.Serial(port=args.port, baudrate=1_000_000, timeout=0.1)
    time.sleep(0.2)

    try:
        print("Current positions:")
        for motor_id in motor_ids:
            pos = read_pos(ser, motor_id)
            name = JOINT_NAMES.get(motor_id, f"motor_{motor_id}")
            if pos is None:
                print(f"  ID {motor_id:>3} ({name:<14})  no response")
            else:
                print(f"  ID {motor_id:>3} ({name:<14})  pos={pos:>4} ({(pos/4096)*360:6.1f} deg)")

        print("\nMove the arm to your desired neutral pose now.")
        print("- shoulder_pan centered")
        print("- other joints in your preferred upright/neutral pose")
        input("\nPress Enter to set THIS pose as midpoint (2048) for selected IDs...")

        print("\nApplying midpoint calibration...")
        for motor_id in motor_ids:
            ok = set_midpoint_here(ser, motor_id)
            name = JOINT_NAMES.get(motor_id, f"motor_{motor_id}")
            print(f"  ID {motor_id:>3} ({name:<14})  {'OK' if ok else 'FAILED'}")
            time.sleep(0.08)

        time.sleep(0.3)
        print("\nVerification (should be near 2048):")
        for motor_id in motor_ids:
            pos = read_pos(ser, motor_id)
            name = JOINT_NAMES.get(motor_id, f"motor_{motor_id}")
            if pos is None:
                print(f"  ID {motor_id:>3} ({name:<14})  no response")
            else:
                delta = pos - 2048
                print(f"  ID {motor_id:>3} ({name:<14})  pos={pos:>4}  delta={delta:+5d}")

        print("\nDone. Next: run limit capture again if needed:")
        print(f"  uv run python basic_setup/05_capture_joint_limits.py --port {args.port}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
