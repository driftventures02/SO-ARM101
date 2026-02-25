#!/usr/bin/env python3
"""
Numerical inverse kinematics for SO-ARM101.

Damped least-squares (pseudo-inverse) method with multi-seed solving.

Usage:
    uv run python basic_setup/kinematics/inverse_kin.py --port /dev/cu.usbmodemXXXXX --x 0.324 --y -0.026 --z 0.122
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from kinematics.forward_kin import SoArm101Geometry, fk_xyz_m, fk_from_ticks_m
from kinematics.motion import (
    JointCalibration,
    JOINT_LIMITS,
    tick_to_rad,
    ticks_to_rads,
    rads_to_ticks,
    read_positions,
    torque_on,
    torque_off,
    move,
)


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
    """Invert a 3×3 matrix. Returns None if singular."""
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
    """Damped least-squares: dq = J^T (J J^T + λI)^{-1} e."""
    JJT = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            for k in range(4):
                JJT[i][j] += jac[i][k] * jac[j][k]
            if i == j:
                JJT[i][j] += damping

    inv = _inv3x3(JJT)
    if inv is None:
        dq = [0.0] * 4
        for j in range(4):
            for ax in range(3):
                dq[j] += 0.1 * jac[ax][j] * err[ax]
        return dq

    v = [sum(inv[i][j] * err[j] for j in range(3)) for i in range(3)]
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

    def _clamp_rads(self, rads: list[float]) -> list[float]:
        """Clamp radians to joint limits via tick round-trip.
        Unwraps to prevent 360° jumps during solver iteration.
        """
        ticks = rads_to_ticks(rads, self.cal)
        clamped = ticks_to_rads(ticks, self.cal)
        for j in range(4):
            while clamped[j] - rads[j] > math.pi:
                clamped[j] -= 2 * math.pi
            while clamped[j] - rads[j] < -math.pi:
                clamped[j] += 2 * math.pi
        return clamped

    def _solve_from_seed(self, target_xyz, seed_rads):
        """Run DLS from a seed. Returns (rads, error_m) or None."""
        rads = list(seed_rads)
        dist = float("inf")
        for iteration in range(self.max_iter):
            pos = fk_xyz_m(*rads, self.geo)
            err = (target_xyz[0] - pos[0], target_xyz[1] - pos[1], target_xyz[2] - pos[2])
            dist = math.sqrt(sum(e ** 2 for e in err))
            if dist < self.tolerance_m:
                return rads, dist
            jac = _finite_diff_jacobian(rads, self.geo)
            dq = _dls_step(jac, err, self.damping)
            rads = [rads[j] + dq[j] for j in range(4)]
            if iteration % 5 == 0:
                rads = self._clamp_rads(rads)

        rads = self._clamp_rads(rads)
        pos = fk_xyz_m(*rads, self.geo)
        dist = math.sqrt(sum((target_xyz[a] - pos[a]) ** 2 for a in range(3)))
        return (rads, dist) if dist < self.tolerance_m else None

    def solve(self, target_xyz, current_ticks: dict[int, int]) -> dict[int, int] | None:
        """Solve IK with multiple seeds. Returns {motor_id: tick} or None."""
        cal = self.cal
        seed0 = ticks_to_rads(current_ticks, cal)

        def _mid_tick(mid):
            lo, hi, wraps = JOINT_LIMITS.get(mid, (0, 4095, False))
            return lo if wraps else (lo + hi) // 2

        seeds = [seed0, ticks_to_rads({m: _mid_tick(m) for m in (1, 2, 3, 4)}, cal)]
        for frac in (0.25, 0.75):
            seed_ticks = {}
            for mid in (1, 2, 3, 4):
                lo, hi, wraps = JOINT_LIMITS.get(mid, (0, 4095, False))
                seed_ticks[mid] = lo if wraps else int(lo + (hi - lo) * frac)
            seeds.append(ticks_to_rads(seed_ticks, cal))

        best, best_dist = None, float("inf")
        for seed in seeds:
            result = self._solve_from_seed(target_xyz, seed)
            if result is None:
                continue
            rads, _ = result
            ticks = rads_to_ticks(rads, cal)
            check_pos = fk_xyz_m(*ticks_to_rads(ticks, cal), self.geo)
            check_dist = math.sqrt(sum((target_xyz[a] - check_pos[a]) ** 2 for a in range(3)))
            if check_dist < best_dist:
                best, best_dist = ticks, check_dist

        if best is None or best_dist > self.tolerance_m * 5:
            print(f"IK: no seed converged. Best error = {best_dist * 1000:.1f} mm")
            return None

        print(f"IK converged, error = {best_dist * 1000:.1f} mm")
        return best


# ── CLI ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="IK — solve and optionally move to XYZ")
    parser.add_argument("--port", required=True)
    parser.add_argument("--x", type=float, required=True, help="meters")
    parser.add_argument("--y", type=float, required=True, help="meters")
    parser.add_argument("--z", type=float, required=True, help="meters")
    parser.add_argument("--move", action="store_true", help="Actually move the arm")
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()

    from servo_utils import ServoBus

    bus = ServoBus(args.port)
    time.sleep(0.2)

    try:
        current = read_positions(bus)
        if not all(mid in current for mid in (1, 2, 3, 4)):
            print("ERROR: Can't read all motors.")
            return

        cur_xyz = fk_from_ticks_m(current, SoArm101Geometry())
        print(f"Current ticks: { {k: current[k] for k in (1,2,3,4)} }")
        if cur_xyz:
            print(f"Current FK:    x={cur_xyz[0]*1000:+.1f}  y={cur_xyz[1]*1000:+.1f}  z={cur_xyz[2]*1000:+.1f} mm")
        print(f"Target:        x={args.x*1000:+.1f}  y={args.y*1000:+.1f}  z={args.z*1000:+.1f} mm")

        goal = IKSolver().solve((args.x, args.y, args.z), current)
        if goal is None:
            return

        print(f"IK ticks:      {goal}")
        check_xyz = fk_from_ticks_m(goal, SoArm101Geometry())
        if check_xyz:
            print(f"IK FK check:   x={check_xyz[0]*1000:+.1f}  y={check_xyz[1]*1000:+.1f}  z={check_xyz[2]*1000:+.1f} mm")

        if not args.move:
            print("(Dry run — pass --move to actually move)")
            return

        torque_on(bus, current)
        move(bus, current, goal, steps=args.steps, duration=args.duration)

        print("Done. Press Enter to release torque.")
        input()

    finally:
        torque_off(bus)
        print("Torque OFF.")
        bus.close()


if __name__ == "__main__":
    main()
