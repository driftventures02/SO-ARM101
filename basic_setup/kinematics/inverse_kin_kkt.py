#!/usr/bin/env python3
"""
Inverse kinematics using scipy L-BFGS-B (constrained optimization).

Joint limits are enforced via proper box constraints — no clamping hacks.
L-BFGS-B uses KKT conditions internally to respect bounds.

Usage:
    uv run python basic_setup/kinematics/inverse_kin_scipy.py --port /dev/cu.usbmodemXXXXX --x 0.324 --y -0.026 --z 0.122
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from scipy.optimize import minimize

from kinematics.forward_kin import SoArm101Geometry, fk_xyz_m, fk_from_ticks_m
from kinematics.motion import (
    JointCalibration,
    JOINT_LIMITS,
    _cal_params,
    tick_to_rad,
    ticks_to_rads,
    rads_to_ticks,
    read_positions,
    torque_on,
    torque_off,
    move,
)


# ── Radian bounds from tick limits ────────────────────────────────────

def _rad_bounds(cal: JointCalibration | None = None) -> list[tuple[float, float]]:
    """Compute (lo_rad, hi_rad) bounds for each FK joint."""
    params = _cal_params(cal or JointCalibration())
    bounds = []
    for j, mid in enumerate((1, 2, 3, 4)):
        center, invert = params[j]
        lo_tick, hi_tick, _ = JOINT_LIMITS.get(mid, (0, 4095, False))
        lo_rad = tick_to_rad(lo_tick, center, invert)
        hi_rad = tick_to_rad(hi_tick, center, invert)
        bounds.append((min(lo_rad, hi_rad), max(lo_rad, hi_rad)))
    return bounds


# ── Solver ────────────────────────────────────────────────────────────

class IKSolver:
    def __init__(
        self,
        geo: SoArm101Geometry | None = None,
        cal: JointCalibration | None = None,
        tolerance_m: float = 0.002,
    ):
        self.geo = geo or SoArm101Geometry()
        self.cal = cal or JointCalibration()
        self.tolerance_m = tolerance_m
        self.bounds = _rad_bounds(self.cal)

    def _cost(self, rads, target_xyz, start_rads):
        """Squared distance from target + regularization to stay near current pose."""
        pos = fk_xyz_m(*rads, self.geo)
        pos_err = sum((target_xyz[i] - pos[i]) ** 2 for i in range(3))
        # Small penalty for moving joints far from starting position.
        # Prevents the redundant DOF (wrist) from drifting to extremes.
        reg = 1e-4 * sum((rads[j] - start_rads[j]) ** 2 for j in range(4))
        return pos_err + reg

    def solve(self, target_xyz, current_ticks: dict[int, int]) -> dict[int, int] | None:
        """Solve IK. Returns {motor_id: tick} or None."""
        cal = self.cal
        seed0 = ticks_to_rads(current_ticks, cal)

        # Build seeds: current position + mid/quarter/three-quarter of range.
        def _seed_ticks(frac):
            t = {}
            for mid in (1, 2, 3, 4):
                lo, hi, _ = JOINT_LIMITS.get(mid, (0, 4095, False))
                t[mid] = int(lo + (hi - lo) * frac)
            return t

        seeds = [
            seed0,
            ticks_to_rads(_seed_ticks(0.5), cal),
            ticks_to_rads(_seed_ticks(0.25), cal),
            ticks_to_rads(_seed_ticks(0.75), cal),
        ]

        best, best_dist = None, float("inf")
        for seed in seeds:
            result = minimize(
                self._cost,
                x0=seed,
                args=(target_xyz, seed0),
                method="L-BFGS-B",
                bounds=self.bounds,
                options={"maxiter": 200, "ftol": 1e-12},
            )
            pos = fk_xyz_m(*result.x, self.geo)
            dist = math.sqrt(sum((target_xyz[i] - pos[i]) ** 2 for i in range(3)))
            if dist < best_dist:
                best_rads = list(result.x)
                best_dist = dist

        if best_dist > self.tolerance_m * 5:
            print(f"IK: no seed converged. Best error = {best_dist * 1000:.1f} mm")
            return None

        ticks = rads_to_ticks(best_rads, cal)
        # Verify after tick quantization.
        check_pos = fk_xyz_m(*ticks_to_rads(ticks, cal), self.geo)
        check_dist = math.sqrt(sum((target_xyz[i] - check_pos[i]) ** 2 for i in range(3)))
        print(f"IK converged, error = {check_dist * 1000:.1f} mm")
        return ticks


# ── CLI ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="IK (scipy) — solve and optionally move to XYZ")
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
