#!/usr/bin/env python3
"""Shared servo/motion utilities for SO-ARM101.

Everything related to ticks, radians, joint limits, and moving the arm.

Usage:
    from kinematics.motion import (
        tick_to_rad, rad_to_tick, JointCalibration,
        clamp_tick, safe_interpolation_diff,
        read_positions, torque_on, torque_off, move,
    )
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# Lazy import — servo_utils requires pyserial which isn't needed for pure math.
def _servo():
    import servo_utils
    return servo_utils

# ── Constants ─────────────────────────────────────────────────────────

MOTOR_IDS = [1, 2, 3, 4, 5, 6]
FK_MOTOR_IDS = [1, 2, 3, 4]
TICKS_PER_REV = 4096.0

# ── Tick ↔ Radian conversion ─────────────────────────────────────────

@dataclass
class JointCalibration:
    """Per-joint tick↔radian mapping.

    center_tick = the servo tick when that joint is at 0 radians (arm straight).
    Measured by putting the arm fully straight and reading ticks.
    """
    center_tick_pan: float = 1977
    center_tick_shoulder: float = 2774
    center_tick_elbow: float = 397
    center_tick_wrist_flex: float = 3082
    invert_pan: bool = False
    invert_shoulder: bool = False
    invert_elbow: bool = False
    invert_wrist_flex: bool = False


def tick_to_rad(tick: int | float, center_tick: float, invert: bool = False) -> float:
    """Convert a servo tick to radians (0 rad = center_tick)."""
    angle = ((float(tick) - center_tick) / TICKS_PER_REV) * (2.0 * math.pi)
    return -angle if invert else angle


def rad_to_tick(rad: float, center_tick: float, invert: bool = False) -> float:
    """Convert radians back to a servo tick."""
    if invert:
        rad = -rad
    return center_tick + (rad / (2.0 * math.pi)) * TICKS_PER_REV


def _cal_params(cal: JointCalibration) -> list[tuple[float, bool]]:
    """Return [(center_tick, invert), ...] for motor IDs 1-4."""
    return [
        (cal.center_tick_pan, cal.invert_pan),
        (cal.center_tick_shoulder, cal.invert_shoulder),
        (cal.center_tick_elbow, cal.invert_elbow),
        (cal.center_tick_wrist_flex, cal.invert_wrist_flex),
    ]


def ticks_to_rads(
    ticks: dict[int, int], cal: JointCalibration | None = None,
) -> list[float]:
    """Convert {motor_id: tick} → [pan, shoulder, elbow, wrist] in radians."""
    params = _cal_params(cal or JointCalibration())
    return [tick_to_rad(ticks[mid], params[mid - 1][0], params[mid - 1][1])
            for mid in (1, 2, 3, 4)]


def rads_to_ticks(
    rads: list[float], cal: JointCalibration | None = None, clamp: bool = True,
) -> dict[int, int]:
    """Convert [pan, shoulder, elbow, wrist] radians → {motor_id: tick}.

    If clamp=True (default), ticks are clamped to joint limits.
    """
    params = _cal_params(cal or JointCalibration())
    result = {}
    for j, mid in enumerate((1, 2, 3, 4)):
        raw = rad_to_tick(rads[j], params[j][0], params[j][1])
        result[mid] = clamp_tick(mid, raw) if clamp else int(round(raw)) % 4096
    return result


# ── Joint limits ──────────────────────────────────────────────────────

_LIMITS_PATH = Path(__file__).resolve().parent / "joint_limits.json"
with open(_LIMITS_PATH) as _f:
    _jl = json.load(_f)["joint_limits"]

# {motor_id: (lo, hi, wraps)}
JOINT_LIMITS: dict[int, tuple[int, int, bool]] = {}
for _v in _jl.values():
    JOINT_LIMITS[_v["id"]] = (_v["min"], _v["max"], _v.get("wraps", False))


def clamp_tick(motor_id: int, tick: float) -> int:
    """Clamp a tick to the joint's safe range."""
    lo, hi, wraps = JOINT_LIMITS.get(motor_id, (0, 4095, False))
    t = int(round(tick)) % 4096
    if not wraps:
        return max(lo, min(hi, t))
    # Wrapping joint: safe is [lo..4095] ∪ [0..hi].
    if t >= lo or t <= hi:
        return t
    # In forbidden zone — snap to nearest safe boundary.
    return lo if (lo - t) <= (t - hi) else hi


def safe_interpolation_diff(start: int, goal: int, motor_id: int) -> int:
    """Compute tick diff from start→goal that avoids the forbidden zone.

    For non-wrapping joints, returns plain (goal - start).
    For wrapping joints, picks the direction that doesn't cross the gap.
    """
    lo, hi, wraps = JOINT_LIMITS.get(motor_id, (0, 4095, False))
    diff = goal - start
    if not wraps or lo <= hi:
        return diff
    # Wrapping joint: safe = [lo..4095] ∪ [0..hi], gap = [hi+1..lo-1].
    d_pos = (goal - start) % 4096
    if d_pos == 0:
        return 0
    d_neg = d_pos - 4096
    gap_mid = (hi + 1 + lo - 1) // 2
    # If positive direction crosses the gap midpoint, go negative.
    if (gap_mid - start) % 4096 <= d_pos:
        return d_neg
    return d_pos


# ── Bus helpers ───────────────────────────────────────────────────────

def read_positions(bus, motor_ids: list[int] | None = None) -> dict[int, int]:
    """Read current tick positions. Retries once on failure."""
    su = _servo()
    ids = motor_ids or MOTOR_IDS
    out: dict[int, int] = {}
    for mid in ids:
        p = bus.read_reg(mid, su.REG_PRESENT_POSITION, 2)
        if p and len(p) >= 2:
            out[mid] = su.to_u16(p[0], p[1])
    if not all(mid in out for mid in ids):
        time.sleep(0.1)
        for mid in ids:
            if mid not in out:
                p = bus.read_reg(mid, su.REG_PRESENT_POSITION, 2)
                if p and len(p) >= 2:
                    out[mid] = su.to_u16(p[0], p[1])
    return out


def torque_on(bus, current: dict[int, int], motor_ids: list[int] | None = None):
    """Sync goal positions to current, set acceleration, then enable torque."""
    su = _servo()
    ids = motor_ids or MOTOR_IDS
    for mid in ids:
        bus.write_reg(mid, su.REG_ACCELERATION, 20, 1)
    time.sleep(0.02)
    for mid in ids:
        bus.write_reg(mid, su.REG_GOAL_POSITION, current.get(mid, 2048), 2)
    time.sleep(0.05)
    for mid in ids:
        bus.write_reg(mid, su.REG_TORQUE_ENABLE, 1, 1)
    time.sleep(0.05)


def torque_off(bus, motor_ids: list[int] | None = None):
    """Disable torque on all motors (retries twice to make sure)."""
    su = _servo()
    ids = motor_ids or MOTOR_IDS
    for _ in range(2):
        for mid in ids:
            bus.write_reg(mid, su.REG_TORQUE_ENABLE, 0, 1)
            time.sleep(0.02)
        time.sleep(0.1)


def move(
    bus,
    start: dict[int, int],
    goal: dict[int, int],
    motor_ids: list[int] | None = None,
    steps: int = 100,
    duration: float = 2.0,
):
    """Interpolate from start to goal ticks, using safe wrapping paths.

    Only moves the joints specified in `goal`. Other motors are untouched.
    """
    su = _servo()
    ids = motor_ids or list(goal.keys())
    diffs = {}
    for mid in ids:
        s = start.get(mid, 2048)
        g = goal.get(mid, s)
        diffs[mid] = safe_interpolation_diff(s, g, mid)

    dt = duration / steps
    for step in range(1, steps + 1):
        t = step / steps
        for mid in ids:
            s = start.get(mid, 2048)
            pos = int(s + diffs[mid] * t) % 4096
            bus.write_reg(mid, su.REG_GOAL_POSITION, pos, 2)
        time.sleep(dt)
