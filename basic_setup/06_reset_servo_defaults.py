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


def read_u16(bus: ServoBus, motor_id: int, reg: int):
    p = bus.read_reg(motor_id, reg, 2)
    return to_u16(p[0], p[1]) if p else None


def write_and_verify_1(bus: ServoBus, motor_id: int, reg: int, value: int, tries: int = 3) -> bool:
    for _ in range(tries):
        bus.write_reg(motor_id, reg, value, 1)
        time.sleep(0.03)
        got = bus.read_reg(motor_id, reg, 1)
        if got and got[0] == (value & 0xFF):
            return True
    return False


def write_and_verify_2(bus: ServoBus, motor_id: int, reg: int, value: int, tries: int = 3) -> bool:
    want = value & 0xFFFF
    for _ in range(tries):
        bus.write_reg(motor_id, reg, want, 2)
        time.sleep(0.03)
        got = read_u16(bus, motor_id, reg)
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
        for motor_id in ids:
            print(
                motor_id,
                {
                    "mode": (bus.read_reg(motor_id, REG_MODE, 1) or [None])[0],
                    "min": read_u16(bus, motor_id, REG_MIN),
                    "max": read_u16(bus, motor_id, REG_MAX),
                    "offset": read_u16(bus, motor_id, REG_OFFSET),
                    "present": read_u16(bus, motor_id, REG_PRESENT_POSITION),
                },
            )

        # Critical safety step: disable torque before changing mode/offset,
        # otherwise motors can jump while internal mapping changes.
        for motor_id in ids:
            bus.write_reg(motor_id, REG_TORQUE, 0, 1)
        time.sleep(0.08)

        for motor_id in ids:
            # unlock EEPROM
            write_and_verify_1(bus, motor_id, REG_LOCK, 0)
            # reset core settings
            write_and_verify_1(bus, motor_id, REG_MODE, 0)
            write_and_verify_2(bus, motor_id, REG_MIN, 0)
            write_and_verify_2(bus, motor_id, REG_MAX, 4095)
            write_and_verify_2(bus, motor_id, REG_OFFSET, 0)
            # Hold current pose as goal, so next torque ON doesn't snap.
            cur = read_u16(bus, motor_id, REG_PRESENT_POSITION) or 2048
            bus.write_reg(motor_id, REG_GOAL_POSITION, cur, 2)
            # lock EEPROM
            write_and_verify_1(bus, motor_id, REG_LOCK, 1)
            time.sleep(0.03)

        print("\nafter:")
        for motor_id in ids:
            print(
                motor_id,
                {
                    "mode": (bus.read_reg(motor_id, REG_MODE, 1) or [None])[0],
                    "min": read_u16(bus, motor_id, REG_MIN),
                    "max": read_u16(bus, motor_id, REG_MAX),
                    "offset": read_u16(bus, motor_id, REG_OFFSET),
                    "present": read_u16(bus, motor_id, REG_PRESENT_POSITION),
                },
            )
    finally:
        bus.close()


if __name__ == "__main__":
    main()
