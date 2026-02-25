#!/usr/bin/env python3
"""
Numerical inverse kinematics for SO-ARM101 with joint-limit safety.

Approach:
  - Damped least-squares (Jacobian transpose) — simplest stable numerical IK.
  - Hard clamps every joint to its recorded safe range every iteration.
  - When commanding motors, moves each joint via the shortest safe path
    (never crosses through the forbidden zone outside [min, max]).

Usage as a library:
    from kinematics.ik import IKSolver
    solver = IKSolver.from_limits_file("kinematics/joint_limits.json")
    ticks = solver.solve(target_xyz=(0.15, 0.0, 0.20), current_ticks={1: ..., 2: ..., 3: ..., 4: ...})

Usage standalone (demo):
    uv run python basic_setup/kinematics/ik.py --port /dev/cu.usbmodemXXXXX --x 0.15 --y 0.0 --z 0.20
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from kinematics.so_arm101_fk import (
    SoArm101Geometry,
    JointCalibration,
    tick_to_rad,
    fk_xyz_m,
    TICKS_PER_REV,
    DEFAULT_CENTER_TICK,
)


# ── Joint limit helpers ─────────────────────────────────────────────

@dataclass
class JointLimit:
    """Safe tick range for one joint."""
    min_tick: int = 0
    max_tick: int = 4095


@dataclass
class JointLimits:
    """Safe ranges for the 4 FK-relevant joints (IDs 1-4)."""
    pan: JointLimit = field(default_factory=JointLimit)
    shoulder: JointLimit = field(default_factory=JointLimit)
    elbow: JointLimit = field(default_factory=JointLimit)
    wrist_flex: JointLimit = field(default_factory=JointLimit)

    @staticmethod
    def from_file(path: str | Path) -> JointLimits:
        with open(path) as f:
            data = json.load(f)
        jl = data.get("joint_limits", data)
        limits = JointLimits()
        mapping = {
            "shoulder_pan": "pan",
            "shoulder_lift": "shoulder",
            "elbow_flex": "elbow",
            "wrist_flex": "wrist_flex",
        }
        for json_name, attr_name in mapping.items():
            if json_name in jl:
                entry = jl[json_name]
                setattr(limits, attr_name, JointLimit(entry["min"], entry["max"]))
        return limits


def clamp_tick(tick: float, limit: JointLimit) -> int:
    """Clamp a tick value to the safe range."""
    return max(limit.min_tick, min(limit.max_tick, int(round(tick))))


def rad_to_tick(rad: float, center_tick: float = DEFAULT_CENTER_TICK,
                invert: bool = False) -> float:
    """Inverse of tick_to_rad."""
    if invert:
        rad = -rad
    return center_tick + (rad / (2.0 * math.pi)) * TICKS_PER_REV


# ── Numerical IK solver ─────────────────────────────────────────────

def _finite_diff_jacobian(
    angles: list[float],
    geometry: SoArm101Geometry,
    eps: float = 1e-5,
) -> list[list[float]]:
    """3×4 Jacobian via finite differences (simple, no analytic derivation)."""
    base = fk_xyz_m(*angles, geometry)
    jac = []
    for axis in range(3):  # x, y, z
        row = []
        for j in range(4):  # 4 joints
            perturbed = list(angles)
            perturbed[j] += eps
            fwd = fk_xyz_m(*perturbed, geometry)
            row.append((fwd[axis] - base[axis]) / eps)
        jac.append(row)
    return jac


def _jt_step(
    jac: list[list[float]],
    error: tuple[float, float, float],
    alpha: float,
) -> list[float]:
    """Jacobian-transpose step: dq = alpha * J^T * e."""
    dq = [0.0] * 4
    for j in range(4):
        for axis in range(3):
            dq[j] += alpha * jac[axis][j] * error[axis]
    return dq


class IKSolver:
    """Simple damped Jacobian-transpose IK with hard joint limits."""

    def __init__(
        self,
        geometry: SoArm101Geometry | None = None,
        calibration: JointCalibration | None = None,
        limits: JointLimits | None = None,
        max_iter: int = 500,
        tolerance_m: float = 0.002,
        alpha: float = 0.3,
    ):
        self.geometry = geometry or SoArm101Geometry()
        self.calibration = calibration or JointCalibration()
        self.limits = limits or JointLimits()
        self.max_iter = max_iter
        self.tolerance_m = tolerance_m
        self.alpha = alpha

    @staticmethod
    def from_limits_file(
        path: str | Path,
        geometry: SoArm101Geometry | None = None,
    ) -> IKSolver:
        return IKSolver(geometry=geometry, limits=JointLimits.from_file(path))

    # ── Limit helpers ────────────────────────────────────────────

    def _limit_for(self, joint_idx: int) -> JointLimit:
        return [self.limits.pan, self.limits.shoulder,
                self.limits.elbow, self.limits.wrist_flex][joint_idx]

    def _clamp_angles(self, angles: list[float]) -> list[float]:
        """Clamp angles so their corresponding ticks stay inside safe range."""
        cal = self.calibration
        centers = [cal.center_tick_pan, cal.center_tick_shoulder,
                   cal.center_tick_elbow, cal.center_tick_wrist_flex]
        inverts = [cal.invert_pan, cal.invert_shoulder,
                   cal.invert_elbow, cal.invert_wrist_flex]
        out = []
        for j in range(4):
            tick = rad_to_tick(angles[j], centers[j], inverts[j])
            clamped = clamp_tick(tick, self._limit_for(j))
            out.append(tick_to_rad(clamped, centers[j], inverts[j]))
        return out

    # ── Core solve ───────────────────────────────────────────────

    def solve(
        self,
        target_xyz: tuple[float, float, float],
        current_ticks: dict[int, int],
    ) -> dict[int, int] | None:
        """
        Solve IK for a target (x, y, z) in meters.

        Args:
            target_xyz: desired end-effector position.
            current_ticks: {motor_id: tick} for IDs 1-4 (starting pose).

        Returns:
            {motor_id: tick} for IDs 1-4 with safe, clamped goal ticks,
            or None if IK did not converge.
        """
        cal = self.calibration
        # Seed angles from current ticks (clamp first so we start safe).
        angles = [
            tick_to_rad(current_ticks[1], cal.center_tick_pan, cal.invert_pan),
            tick_to_rad(current_ticks[2], cal.center_tick_shoulder, cal.invert_shoulder),
            tick_to_rad(current_ticks[3], cal.center_tick_elbow, cal.invert_elbow),
            tick_to_rad(current_ticks[4], cal.center_tick_wrist_flex, cal.invert_wrist_flex),
        ]
        angles = self._clamp_angles(angles)

        dist = float("inf")
        for _ in range(self.max_iter):
            pos = fk_xyz_m(*angles, self.geometry)
            err = (target_xyz[0] - pos[0],
                   target_xyz[1] - pos[1],
                   target_xyz[2] - pos[2])
            dist = math.sqrt(err[0]**2 + err[1]**2 + err[2]**2)
            if dist < self.tolerance_m:
                break

            jac = _finite_diff_jacobian(angles, self.geometry)
            dq = _jt_step(jac, err, self.alpha)

            angles = [angles[j] + dq[j] for j in range(4)]
            # Hard clamp every iteration so we never leave the safe zone.
            angles = self._clamp_angles(angles)
        else:
            # Did not converge — report remaining error.
            print(f"IK: {self.max_iter} iters, remaining error = {dist*1000:.1f} mm")
            return None

        # Convert final angles → ticks (already clamped).
        centers = [cal.center_tick_pan, cal.center_tick_shoulder,
                   cal.center_tick_elbow, cal.center_tick_wrist_flex]
        inverts = [cal.invert_pan, cal.invert_shoulder,
                   cal.invert_elbow, cal.invert_wrist_flex]
        result = {}
        for j, motor_id in enumerate([1, 2, 3, 4]):
            tick = rad_to_tick(angles[j], centers[j], inverts[j])
            result[motor_id] = clamp_tick(tick, self._limit_for(j))
        return result

    # ── Safe motion planning ─────────────────────────────────────

    def plan_safe_path(
        self,
        current_ticks: dict[int, int],
        goal_ticks: dict[int, int],
        steps: int = 20,
    ) -> list[dict[int, int]]:
        """
        Linear interpolation from current to goal in tick space,
        clamped at every step so no joint ever leaves its safe range.

        Returns a list of waypoint dicts [{motor_id: tick}, ...].
        """
        waypoints = []
        for s in range(1, steps + 1):
            t = s / steps
            wp = {}
            for motor_id in (1, 2, 3, 4):
                cur = current_ticks[motor_id]
                goal = goal_ticks[motor_id]
                interp = int(round(cur + (goal - cur) * t))
                limit = self._limit_for(motor_id - 1)
                wp[motor_id] = clamp_tick(interp, limit)
            waypoints.append(wp)
        return waypoints


# ── CLI demo ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="IK demo — move arm to XYZ")
    parser.add_argument("--port", required=True)
    parser.add_argument("--x", type=float, required=True, help="Target X (meters)")
    parser.add_argument("--y", type=float, required=True, help="Target Y (meters)")
    parser.add_argument("--z", type=float, required=True, help="Target Z (meters)")
    parser.add_argument("--limits", default=str(Path(__file__).resolve().parent / "joint_limits.json"))
    parser.add_argument("--steps", type=int, default=30, help="Interpolation steps for safe motion")
    parser.add_argument("--step-delay", type=float, default=0.02, help="Delay between waypoints (s)")
    args = parser.parse_args()

    from servo_utils import ServoBus, REG_GOAL_POSITION, REG_PRESENT_POSITION, to_u16

    bus = ServoBus(args.port)

    # Read current ticks for joints 1-4.
    current = {}
    for motor_id in (1, 2, 3, 4):
        p = bus.read_reg(motor_id, REG_PRESENT_POSITION, 2)
        if not p:
            print(f"Motor {motor_id} not responding!")
            bus.close()
            return
        current[motor_id] = to_u16(p[0], p[1])

    print(f"Current ticks: {current}")
    print(f"Target: ({args.x}, {args.y}, {args.z}) m")

    solver = IKSolver.from_limits_file(args.limits)
    goal = solver.solve((args.x, args.y, args.z), current)

    if goal is None:
        print("IK did not converge — target may be out of reach.")
        bus.close()
        return

    print(f"IK solution ticks: {goal}")

    # Plan safe interpolated path.
    path = solver.plan_safe_path(current, goal, steps=args.steps)

    # Execute.
    bus.set_torque_all([1, 2, 3, 4], True)
    for wp in path:
        for motor_id in (1, 2, 3, 4):
            bus.write_reg(motor_id, REG_GOAL_POSITION, wp[motor_id], 2)
        time.sleep(args.step_delay)

    # Read back final position.
    time.sleep(0.3)
    final = {}
    for motor_id in (1, 2, 3, 4):
        p = bus.read_reg(motor_id, REG_PRESENT_POSITION, 2)
        if p:
            final[motor_id] = to_u16(p[0], p[1])
    print(f"Final ticks: {final}")

    from kinematics.so_arm101_fk import fk_from_ticks_m, SoArm101Geometry
    xyz = fk_from_ticks_m(final, SoArm101Geometry())
    if xyz:
        print(f"Final FK position: x={xyz[0]:.4f} y={xyz[1]:.4f} z={xyz[2]:.4f} m")

    bus.close()


if __name__ == "__main__":
    main()
