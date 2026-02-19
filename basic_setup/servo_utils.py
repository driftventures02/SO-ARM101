#!/usr/bin/env python3
"""
Minimal shared Feetech/STS3215 serial helpers.

Jargon (plain English):
- UART: serial communication over one wire, one byte at a time.
- Half-duplex: only one side talks at a time (host then servo replies).
- Register: small numbered memory slot inside each servo.
- Packet: one command/reply message made of bytes.
"""

from __future__ import annotations

import time
import serial

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_READ = 0x82

REG_TORQUE_ENABLE = 40
REG_ACCELERATION = 41
REG_GOAL_POSITION = 42
REG_PRESENT_POSITION = 56


def build_packet(servo_id: int, instruction: int, params: list[int]) -> bytes:
    # Servo protocol layout: FF FF ID LEN INST PARAMS... CHECKSUM
    length = len(params) + 2
    checksum = ~(servo_id + length + instruction + sum(params)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, instruction] + params + [checksum])


def parse_packet(data: bytes):
    # Basic validity checks: header and minimum size.
    if len(data) < 6 or data[0] != 0xFF or data[1] != 0xFF:
        return None
    length = data[3]
    if len(data) < 4 + length:
        return None
    params = list(data[5 : 5 + length - 2]) if length > 2 else []
    return {"id": data[2], "length": length, "error": data[4], "params": params}


def to_u16(lo: int, hi: int) -> int:
    # Servo sends 16-bit values as low-byte then high-byte (little-endian).
    return lo | (hi << 8)


class ServoBus:
    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 0.1):
        # Open OS serial device, e.g. /dev/cu.usbmodemXXXXX on macOS.
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        time.sleep(0.15)

    def close(self):
        self.ser.close()

    def _send_recv(self, packet: bytes, wait: float = 0.01) -> bytes:
        # Clear stale bytes, send one packet, then wait briefly for reply.
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        self.ser.flush()
        time.sleep(wait)
        resp = self.ser.read(self.ser.in_waiting or 64)
        # Some adapters echo TX bytes; strip echo if present.
        if resp and resp[: len(packet)] == packet:
            resp = resp[len(packet) :]
        return resp

    def read_reg(self, motor_id: int, addr: int, nbytes: int):
        # Read N bytes from one register address on one motor.
        pkt = build_packet(motor_id, INST_READ, [addr, nbytes])
        for _ in range(2):
            parsed = parse_packet(self._send_recv(pkt))
            # Important: only trust reply if it came from the expected ID.
            if parsed and parsed["id"] == motor_id and len(parsed["params"]) >= nbytes:
                return parsed["params"][:nbytes]
        return None

    def write_reg(self, motor_id: int, addr: int, value: int, nbytes: int = 1):
        # Write 1-byte or 2-byte value to one register.
        if nbytes == 1:
            params = [addr, value & 0xFF]
        else:
            params = [addr, value & 0xFF, (value >> 8) & 0xFF]
        pkt = build_packet(motor_id, INST_WRITE, params)
        self.ser.write(pkt)
        self.ser.flush()

    def set_torque_all(self, motor_ids: list[int], enabled: bool):
        # Torque ON = motor actively holds position. OFF = motor moves freely by hand.
        val = 1 if enabled else 0
        for mid in motor_ids:
            self.write_reg(mid, REG_TORQUE_ENABLE, val, 1)
            time.sleep(0.002)

    def sync_read(self, motor_ids: list[int], addr: int, nbytes: int):
        # SYNC_READ asks many motors for the same register in one command.
        # Faster and cleaner than sending 6 separate READ commands.
        pkt = build_packet(0xFE, INST_SYNC_READ, [addr, nbytes] + motor_ids)
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()
        # Rough wire-time estimate + margin.
        per_resp = nbytes + 6
        time.sleep(0.004 + (per_resp * len(motor_ids)) / 50000.0)
        raw = self.ser.read(self.ser.in_waiting or per_resp * len(motor_ids) * 2)
        if raw and raw[: len(pkt)] == pkt:
            raw = raw[len(pkt) :]

        out = {}
        i = 0
        while i < len(raw) - 5:
            if raw[i] != 0xFF or raw[i + 1] != 0xFF:
                i += 1
                continue
            ln = raw[i + 3]
            end = i + 4 + ln
            if end > len(raw):
                break
            parsed = parse_packet(raw[i:end])
            if parsed and len(parsed["params"]) >= nbytes:
                out[parsed["id"]] = parsed
            i = end
        return out
