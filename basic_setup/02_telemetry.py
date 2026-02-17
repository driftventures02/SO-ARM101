#!/usr/bin/env python3
"""
Simple live telemetry for all 6 joints.

What this script does:
1) Reads motor status repeatedly (position, speed, load, voltage, temperature).
2) Prints a live table so you can quickly spot problems.

Jargon:
- Telemetry: live measurements from hardware.
- Load: how hard the motor is pushing (rough effort estimate).
"""

import argparse
import time

from servo_utils import ServoBus, to_u16

MOTOR_IDS = [1, 2, 3, 4, 5, 6]
NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def decode_load(raw: int) -> float:
    # Load is encoded as direction bit + magnitude bits in the servo protocol.
    sign = -1 if raw & 0x0400 else 1
    return sign * (raw & 0x03FF) / 1023.0 * 100.0


def decode_speed(raw: int) -> int:
    # Speed uses a sign bit plus magnitude bits.
    sign = -1 if raw & 0x8000 else 1
    return sign * (raw & 0x7FFF)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--hz", type=float, default=15.0)
    args = parser.parse_args()

    # One shared bus object handles all serial packet details.
    bus = ServoBus(args.port)
    period = 1.0 / max(1.0, args.hz)

    try:
        while True:
            t0 = time.time()
            # Read 8 bytes starting at register 56 from each motor:
            # pos(2), speed(2), load(2), volt(1), temp(1)
            packets = bus.sync_read(MOTOR_IDS, addr=56, nbytes=8)
            print("\033[H\033[J", end="")
            print(f"Telemetry @ {args.hz:.1f}Hz  |  port={args.port}\n")
            print(" joint            id   pos   deg   speed   load%   volt   temp")
            print(" ---------------------------------------------------------------")

            for mid in MOTOR_IDS:
                pkt = packets.get(mid)
                if not pkt or len(pkt["params"]) < 8:
                    # No reply: usually wiring, power, or ID mismatch.
                    print(f" {NAMES[mid]:<15} {mid:>2}   ---   ---    ---    ---    ---    ---")
                    continue
                p = pkt["params"]
                pos = to_u16(p[0], p[1])
                speed = decode_speed(to_u16(p[2], p[3]))
                load = decode_load(to_u16(p[4], p[5]))
                volt = p[6] / 10.0
                temp = p[7]
                deg = pos / 4096.0 * 360.0
                print(f" {NAMES[mid]:<15} {mid:>2}  {pos:>4}  {deg:>5.1f}  {speed:>6}  {load:>6.1f}  {volt:>5.1f}V  {temp:>4}C")

            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()


if __name__ == "__main__":
    main()
