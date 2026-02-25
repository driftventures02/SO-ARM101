#!/usr/bin/env python3
"""
Numerical inverse kinematics for SO-ARM101.

Damped least-squares (pseudo-inverse) method — reuses FK, handles wrapping joints.

Usage:
    uv run python basic_setup/kinematics/ik.py --port /dev/cu.usbmodemXXXXX --x 0.324 --y -0.026 --z 0.122
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from kinematics.so_arm101_fk import (
    SoArm101Geometry,
    JointCalibration,
    tick_to_rad,
    fk_xyz_m,
    fk_from_ticks_m,
    TICKS_PER_REV,
)


# ── Joint limits ──────────────────────────────────────────────────────

_LIMITS_PATH = Path(__file__).resolve().parent / "joint_limits.json"
with open(_LIMITS_PATH) as _f:
    _jl = json.load(_f)["joint_limits"]

JOINT_LIMITS = {}
for _v in _jl.values():
    JOINT_LIMITS[_v["id"]] = (_v["min"], _v["max"], _v.get("wraps", False))


def clamp_tick(motor_id: int, tick: float) -> int:
    lo, hi, wraps = JOINT_LIMITS.get(motor_id, (0, 4095, False))
    t = int(round(tick)) % 4096
    if not wraps:
        return max(lo, min(hi, t))
    # Wrapping: safe is [lo..4095] ∪ [0..hi].
    if t >= lo or t <= hi:
        return t
    return lo if (lo - t) <= (t - hi) else hi


def rad_to_tick(rad: float, center_tick: float, invert: bool = False) -> float:
    if invert:
        rad = -rad
    return center_tick + (rad / (2.0 * math.pi)) * TICKS_PER_REV


# ── IK solver ─────────────────────────────────────────────────────────

def _finite_diff_jacobian(angles: list[float], geo: SoArm101Geometry, eps: float = 1e-5):
    """3×4 Jacobian via finite differences."""
    base = fk_xyz_m(*angles, geo)
    jac = []
    for axis in range(3):
        row = []
        for j in range(4):
            perturbed = list(angles)
            perturbed[j] += eps
            fwd = fk_xyz_m(*perturbed, geo)
            row.append((fwd[axis] - base[axis]) / eps)
        jac.append(row)
    return jac


def _inv3x3(m):
    """Invert a 3×3 matrix (list of lists). Returns None if singular."""
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-20:
        return None
    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


def _dls_step(jac, err, damping=0.001):
    """Damped least-squares: dq = J^T (J J^T + λI)^{-1} e.
    
    Much faster convergence than pure Jacobian-transpose.
    """
    # JJT = J * J^T  (3×3)
    JJT = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            for k in range(4):
                JJT[i][j] += jac[i][k] * jac[j][k]
            if i == j:
                JJT[i][j] += damping

    inv = _inv3x3(JJT)
    if inv is None:
        # Fallback to JT if singular.
        dq = [0.0] * 4
        for j in range(4):
            for ax in range(3):
                dq[j] += 0.1 * jac[ax][j] * err[ax]
        return dq

    # v = inv(JJT) * err  (3×1)
    v = [sum(inv[i][j] * err[j] for j in range(3)) for i in range(3)]
    # dq = J^T * v  (4×1)
    dq = [sum(jac[ax][j] * v[ax] for ax in range(3)) for j in range(4)]
    return dq


class IKSolver:
    def __init__(
        self,
        geo: SoArm101Geometry | None = None,
        cal: JointCalibration | None = None,
        max_iter: int = 200,
        tolerance_m: float = 0.002,
        damping: float = 0.001,
    ):
        self.geo = geo or SoArm101Geometry()
        self.cal = cal or JointCalibration()
        self.max_iter = max_iter
        self.tolerance_m = tolerance_m
        self.damping = damping

    def _centers(self):
        c = self.cal
        return [c.center_tick_pan, c.center_tick_shoulder,
                c.center_tick_elbow, c.center_tick_wrist_flex]

    def _inverts(self):
        c = self.cal
        return [c.invert_pan, c.invert_shoulder,
                c.invert_elbow, c.invert_wrist_flex]

    def _ticks_to_angles(self, ticks: dict[int, int]) -> list[float]:
        centers = self._centers()
        inverts = self._inverts()
        return [tick_to_rad(ticks[mid], centers[mid - 1], inverts[mid - 1])
                for mid in (1, 2, 3, 4)]

    def _angles_to_ticks(self, angles: list[float]) -> dict[int, int]:
        centers = self._centers()
        inverts = self._inverts()
        result = {}
        for j, mid in enumerate((1, 2, 3, 4)):
            raw = rad_to_tick(angles[j], centers[j], inverts[j])
            result[mid] = clamp_tick(mid, raw)
        return result

    def _clamp_angles(self, angles: list[float]) -> list[float]:
        """Round-trip through ticks to enforce joint limits."""
        ticks = self._angles_to_ticks(angles)
        return self._ticks_to_angles(ticks)

    def _solve_from_seed(
        self,
        target_xyz: tuple[float, float, float],
        seed_angles: list[float],
    ) -> tuple[list[float], float] | None:
        """Run DLS from a seed. Returns (angles, error_m) or None."""
        angles = list(seed_angles)

        dist = float("inf")
        for iteration in range(self.max_iter):
            pos = fk_xyz_m(*angles, self.geo)
            err = (target_xyz[0] - pos[0],
                   target_xyz[1] - pos[1],
                   target_xyz[2] - pos[2])
            dist = math.sqrt(err[0] ** 2 + err[1] ** 2 + err[2] ** 2)
            if dist < self.tolerance_m:
                return angles, dist

            jac = _finite_diff_jacobian(angles, self.geo)
            dq = _dls_step(jac, err, self.damping)
            angles = [angles[j] + dq[j] for j in range(4)]
            # Clamp every 5 iters to stay near valid zone.
            if iteration % 5 == 0:
                angles = self._clamp_angles(angles)

        # Didn't converge — final clamp and check.
        angles = self._clamp_angles(angles)
        pos = fk_xyz_m(*angles, self.geo)
        err = (target_xyz[0] - pos[0], target_xyz[1] - pos[1], target_xyz[2] - pos[2])
        dist = math.sqrt(sum(e ** 2 for e in err))
        if dist < self.tolerance_m:
            return angles, dist
        return None

    def solve(
        self,
        target_xyz: tuple[float, float, float],
        current_ticks: dict[int, int],
    ) -> dict[int, int] | None:
        """
        Solve IK with multiple seeds. Returns {motor_id: tick} or None.
        Tries current position + several spread-out seeds to avoid local minima.
        """
        cal = self.cal
        # Build seed angles: current position.
        seed0 = self._ticks_to_angles(current_ticks)

        # Additional seeds at joint-limit midpoints and quarter-points.
        def _mid_tick(mid):
            lo, hi, wraps = JOINT_LIMITS.get(mid, (0, 4095, False))
            if not wraps:
                return (lo + hi) // 2
            return lo  # start from a known-safe edge for wrapping joints

        mid_ticks = {mid: _mid_tick(mid) for mid in (1, 2, 3, 4)}
        seed_mid = self._ticks_to_angles(mid_ticks)

        # Spread seeds across the range.
        seeds = [seed0, seed_mid]
        for frac in (0.25, 0.75):
            s = []
            for j, mid in enumerate((1, 2, 3, 4)):
                lo, hi, wraps = JOINT_LIMITS.get(mid, (0, 4095, False))
                if not wraps:
                    t = int(lo + (hi - lo) * frac)
                else:
                    t = lo
                centers = self._centers()
                inverts = self._inverts()
                s.append(tick_to_rad(t, centers[j], inverts[j]))
            seeds.append(s)

        best = None
        best_dist = float("inf")
        for i, seed in enumerate(seeds):
            result = self._solve_from_seed(target_xyz, seed)
            if result is not None:
                angles, dist = result
                # Check that the clamped ticks are still close.
                ticks = self._angles_to_ticks(angles)
                check_angles = self._ticks_to_angles(ticks)
                check_pos = fk_xyz_m(*check_angles, self.geo)
                check_dist = math.sqrt(sum((target_xyz[a] - check_pos[a]) ** 2 for a in range(3)))
                if check_dist < best_dist:
                    best = ticks
                    best_dist = check_dist

        if best is None or best_dist > self.tolerance_m * 5:
            print(f"IK: no seed converged within limits. Best error = {best_dist * 1000:.1f} mm")
            return None

        print(f"IK converged, error after clamp = {best_dist * 1000:.1f} mm")
        return best


# ── CLI ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="IK test — solve for target XYZ")
    parser.add_argument("--port", required=True)
    parser.add_argument("--x", type=float, required=True, help="meters")
    parser.add_argument("--y", type=float, required=True, help="meters")
    parser.add_argument("--z", type=float, required=True, help="meters")
    parser.add_argument("--move", action="store_true", help="Actually move the arm (otherwise just solve)")
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()

    from servo_utils import ServoBus, REG_GOAL_POSITION, REG_PRESENT_POSITION, REG_TORQUE_ENABLE, REG_ACCELERATION, to_u16

    bus = ServoBus(args.port)
    time.sleep(0.2)

    try:
        # Read current ticks.
        current = {}
        for mid in (1, 2, 3, 4, 5, 6):
            p = bus.read_reg(mid, REG_PRESENT_POSITION, 2)
            if p and len(p) >= 2:
                current[mid] = to_u16(p[0], p[1])
        if not all(mid in current for mid in (1, 2, 3, 4)):
            time.sleep(0.1)
            for mid in (1, 2, 3, 4):
                if mid not in current:
                    p = bus.read_reg(mid, REG_PRESENT_POSITION, 2)
                    if p and len(p) >= 2:
                        current[mid] = to_u16(p[0], p[1])
        if not all(mid in current for mid in (1, 2, 3, 4)):
            print("ERROR: Can't read all motors.")
            return

        # Show current FK position.
        cur_xyz = fk_from_ticks_m(current, SoArm101Geometry())
        print(f"Current ticks: { {k: current[k] for k in (1,2,3,4)} }")
        if cur_xyz:
            print(f"Current FK:    x={cur_xyz[0]*1000:+.1f}  y={cur_xyz[1]*1000:+.1f}  z={cur_xyz[2]*1000:+.1f} mm")
        print(f"Target:        x={args.x*1000:+.1f}  y={args.y*1000:+.1f}  z={args.z*1000:+.1f} mm")

        # Solve IK.
        solver = IKSolver()
        goal = solver.solve((args.x, args.y, args.z), current)
        if goal is None:
            return

        print(f"IK ticks:      {goal}")
        # Verify with FK.
        check_xyz = fk_from_ticks_m(goal, SoArm101Geometry())
        if check_xyz:
            print(f"IK FK check:   x={check_xyz[0]*1000:+.1f}  y={check_xyz[1]*1000:+.1f}  z={check_xyz[2]*1000:+.1f} mm")

        if not args.move:
            print("(Dry run — pass --move to actually move the arm)")
            return

        # Move to IK solution.
        for mid in (1, 2, 3, 4, 5, 6):
            bus.write_reg(mid, REG_ACCELERATION, 20, 1)
        time.sleep(0.02)
        for mid in (1, 2, 3, 4, 5, 6):
            bus.write_reg(mid, REG_GOAL_POSITION, current.get(mid, 2048), 2)
        time.sleep(0.05)
        for mid in (1, 2, 3, 4, 5, 6):
            bus.write_reg(mid, REG_TORQUE_ENABLE, 1, 1)
        time.sleep(0.05)

        dt = args.duration / args.steps
        for step in range(1, args.steps + 1):
            t = step / args.steps
            for mid in (1, 2, 3, 4):
                s = current[mid]
                diff = goal[mid] - s
                # Short path for wrapping joints.
                _, _, wraps = JOINT_LIMITS.get(mid, (0, 4095, False))
                if wraps:
                    if diff > 2048: diff -= 4096
                    elif diff < -2048: diff += 4096
                pos = int(s + diff * t) % 4096
                bus.write_reg(mid, REG_GOAL_POSITION, pos, 2)
            time.sleep(dt)

        print("Done. Press Enter to release torque.")
        input()

    finally:
        for _ in range(2):
            for mid in (1, 2, 3, 4, 5, 6):
                bus.write_reg(mid, REG_TORQUE_ENABLE, 0, 1)
                time.sleep(0.02)
            time.sleep(0.1)
        print("Torque OFF.")
        bus.close()


if __name__ == "__main__":
    main()
