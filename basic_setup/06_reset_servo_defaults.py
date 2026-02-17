#!/usr/bin/env python3
"""
Reset core motor config to sane defaults for selected IDs.

This does NOT change IDs or baud.
It resets:
  - mode -> 0 (position mode)
  - min/max angle -> 0 / 4095
  - offset -> 0
  - goal -> current position (prevents jump on torque-on)
"""

import argparse
import time

from servo_utils import ServoBus, REG_GOAL_POSITION, REG_PRESENT_POSITION, to_u16

REG_LOCK = 55
REG_MIN = 9
REG_MAX = 11
REG_OFFSET = 31
REG_MODE = 33
REG_TORQUE = 40


def read_u16(bus: ServoBus, mid: int, reg: int):
    p = bus.read_reg(mid, reg, 2)
    return to_u16(p[0], p[1]) if p else None


def write_and_verify_1(bus: ServoBus, mid: int, reg: int, value: int, tries: int = 3) -> bool:
    for _ in range(tries):
        bus.write_reg(mid, reg, value, 1)
        time.sleep(0.03)
        got = bus.read_reg(mid, reg, 1)
        if got and got[0] == (value & 0xFF):
            return True
    return False


def write_and_verify_2(bus: ServoBus, mid: int, reg: int, value: int, tries: int = 3) -> bool:
    want = value & 0xFFFF
    for _ in range(tries):
        bus.write_reg(mid, reg, want, 2)
        time.sleep(0.03)
        got = read_u16(bus, mid, reg)
        if got == want:
            return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--ids", default="1,2,3,4,5,6")
    args = parser.parse_args()

    ids = [int(x.strip()) for x in args.ids.split(",") if x.strip()]
    bus = ServoBus(args.port)

    try:
        print("before:")
        for mid in ids:
            print(
                mid,
                {
                    "mode": (bus.read_reg(mid, REG_MODE, 1) or [None])[0],
                    "min": read_u16(bus, mid, REG_MIN),
                    "max": read_u16(bus, mid, REG_MAX),
                    "offset": read_u16(bus, mid, REG_OFFSET),
                    "present": read_u16(bus, mid, REG_PRESENT_POSITION),
                },
            )

        # Critical safety step: disable torque before changing mode/offset,
        # otherwise motors can jump while internal mapping changes.
        for mid in ids:
            bus.write_reg(mid, REG_TORQUE, 0, 1)
        time.sleep(0.08)

        for mid in ids:
            # unlock EEPROM
            write_and_verify_1(bus, mid, REG_LOCK, 0)
            # reset core settings
            write_and_verify_1(bus, mid, REG_MODE, 0)
            write_and_verify_2(bus, mid, REG_MIN, 0)
            write_and_verify_2(bus, mid, REG_MAX, 4095)
            write_and_verify_2(bus, mid, REG_OFFSET, 0)
            # Hold current pose as goal, so next torque ON doesn't snap.
            cur = read_u16(bus, mid, REG_PRESENT_POSITION) or 2048
            bus.write_reg(mid, REG_GOAL_POSITION, cur, 2)
            # lock EEPROM
            write_and_verify_1(bus, mid, REG_LOCK, 1)
            time.sleep(0.03)

        print("\nafter:")
        for mid in ids:
            print(
                mid,
                {
                    "mode": (bus.read_reg(mid, REG_MODE, 1) or [None])[0],
                    "min": read_u16(bus, mid, REG_MIN),
                    "max": read_u16(bus, mid, REG_MAX),
                    "offset": read_u16(bus, mid, REG_OFFSET),
                    "present": read_u16(bus, mid, REG_PRESENT_POSITION),
                },
            )
    finally:
        bus.close()


if __name__ == "__main__":
    main()
