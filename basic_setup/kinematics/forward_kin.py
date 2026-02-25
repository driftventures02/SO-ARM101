#!/usr/bin/env python3
"""
Forward kinematics helpers for SO-ARM101.

Model assumptions (simple and practical for live teleop feedback):
- Joint 1 (shoulder_pan) is base yaw about +Z.
- Joints 2/3/4 (shoulder_lift/elbow_flex/wrist_flex) form a planar chain.
- Wrist roll and gripper do not change XYZ position (orientation/grip only).

All dimensions are in meters.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
import math

from kinematics.motion import (
    JointCalibration,
    tick_to_rad,
    ticks_to_rads,
)


@dataclass
class SoArm101Geometry:
    """Geometric parameters for position-only FK model.

    Origin = center of shoulder pan rotation axis, at table level.

    Physical measurements from the actual SO-ARM101:
      Pan axis → shoulder link:   50 mm  (horizontal, rotates with pan)
      Table → shoulder axis:     120 mm  (60mm servo + 60mm material)
      Shoulder → elbow:          120 mm
      Elbow → wrist flex:        140 mm
      Wrist flex → wrist roll:    90 mm
      Wrist roll → gripper tip:  80 mm
      Total reach:               550 mm
    """

    pan_to_shoulder_m: float = -0.050     # shoulder is 50mm in the arm's forward direction
    base_height_m: float = 0.120          # table to shoulder rotation axis
    shoulder_to_elbow_m: float = 0.120    # upper arm
    elbow_to_wrist_m: float = 0.140       # lower arm
    wrist_to_tool_m: float = 0.090        # wrist flex → wrist roll
    tool_tip_m: float = 0.080             # wrist roll → gripper tip


def fk_xyz_m(
    pan_rad: float,
    shoulder_rad: float,
    elbow_rad: float,
    wrist_flex_rad: float,
    geometry: SoArm101Geometry,
) -> tuple[float, float, float]:
    """
    Position-only FK for SO-ARM101.

    Returns:
        (x, y, z) in meters in a base frame where +Z is up and +X is forward.
    """
    q1 = pan_rad
    # At 0° the arm points straight UP, so add π/2 to convert to
    # the trig frame where 0° = horizontal, 90° = up.
    q2 = shoulder_rad + math.pi / 2
    q23 = q2 + elbow_rad
    q234 = q23 + wrist_flex_rad

    l1 = geometry.shoulder_to_elbow_m
    l2 = geometry.elbow_to_wrist_m
    l3 = geometry.wrist_to_tool_m + geometry.tool_tip_m

    # Planar radius from pan axis and vertical height.
    # pan_to_shoulder is a fixed horizontal offset that rotates with pan.
    r = (geometry.pan_to_shoulder_m
         + l1 * math.cos(q2) + l2 * math.cos(q23) + l3 * math.cos(q234))
    z = (geometry.base_height_m
         + l1 * math.sin(q2) + l2 * math.sin(q23) + l3 * math.sin(q234))

    # The arm reaches in the -r direction in the trig frame,
    # so negate to make forward = +x.
    x = -r * math.cos(q1)
    y = -r * math.sin(q1)
    return x, y, z

def fk_from_ticks_m(
    positions_by_id: dict[int, int],
    geometry: SoArm101Geometry,
    calibration: JointCalibration | None = None,
) -> tuple[float, float, float] | None:
    """Compute FK directly from tick dict {motor_id: tick} → (x, y, z) meters."""
    if any(mid not in positions_by_id for mid in (1, 2, 3, 4)):
        return None
    rads = ticks_to_rads(positions_by_id, calibration)
    return fk_xyz_m(*rads, geometry)


# ── CLI diagnostic ───────────────────────────────────────────────────

def main():
    """Read live motor ticks (or accept manual values) and print FK XYZ.

    Usage:
        # Live from hardware:
        uv run python basic_setup/kinematics/forward_kin.py --port /dev/cu.usbmodemXXXXX

        # Manual tick values (no hardware needed):
        uv run python basic_setup/kinematics/forward_kin.py --ticks 2048 2048 2048 2048

        # Sweep one joint to see how FK responds:
        uv run python basic_setup/kinematics/forward_kin.py --sweep
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

    parser = argparse.ArgumentParser(description="FK diagnostic — see XYZ from joint ticks")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--port", help="Read ticks once from hardware")
    group.add_argument("--live", help="Continuous live display (move arm by hand, Ctrl+C to stop). Pass port.")
    group.add_argument("--ticks", nargs=4, type=int, metavar=("PAN", "SHOULDER", "ELBOW", "WRIST"),
                       help="Manual tick values for joints 1-4")
    args = parser.parse_args()

    import time
    geo = SoArm101Geometry()
    cal = JointCalibration()

    if args.live:
        from servo_utils import ServoBus, REG_PRESENT_POSITION, REG_TORQUE_ENABLE, to_u16
        bus = ServoBus(args.live)
        # Torque off so you can move by hand.
        for motor_id in (1, 2, 3, 4, 5, 6):
            bus.write_reg(motor_id, REG_TORQUE_ENABLE, 0, 1)
        print("Torque OFF — move arm by hand. Ctrl+C to stop.\n")
        names = {1: "pan", 2: "shldr", 3: "elbow", 4: "wrist"}
        try:
            while True:
                ticks = {}
                packets = bus.sync_read([1, 2, 3, 4], REG_PRESENT_POSITION, 2)
                for motor_id in (1, 2, 3, 4):
                    pkt = packets.get(motor_id)
                    if pkt and len(pkt["params"]) >= 2:
                        ticks[motor_id] = to_u16(pkt["params"][0], pkt["params"][1])
                xyz = fk_from_ticks_m(ticks, geo, cal)
                if xyz:
                    parts = []
                    for motor_id in (1, 2, 3, 4):
                        parts.append(f"{names[motor_id]}={ticks.get(motor_id, '?'):>5}")
                    pos_str = "  ".join(parts)
                    print(f"\r  {pos_str}  │  x={xyz[0]*1000:>+7.1f}  y={xyz[1]*1000:>+7.1f}  z={xyz[2]*1000:>+7.1f} mm", end="", flush=True)
                time.sleep(0.08)
        except KeyboardInterrupt:
            print("\nStopped.")
        bus.close()
        return

    if args.ticks:
        ticks = {1: args.ticks[0], 2: args.ticks[1], 3: args.ticks[2], 4: args.ticks[3]}
        print(f"Manual ticks: {ticks}")
    else:
        from servo_utils import ServoBus, REG_PRESENT_POSITION, to_u16
        bus = ServoBus(args.port)
        ticks = {}
        for motor_id in (1, 2, 3, 4, 5, 6):
            p = bus.read_reg(motor_id, REG_PRESENT_POSITION, 2)
            if p:
                ticks[motor_id] = to_u16(p[0], p[1])
        bus.close()
        print(f"Live ticks: {ticks}")

    # Show per-joint angle breakdown.
    centers = {1: cal.center_tick_pan, 2: cal.center_tick_shoulder,
               3: cal.center_tick_elbow, 4: cal.center_tick_wrist_flex}
    print(f"\nJoint angles (0° = arm straight up):")
    names_long = {1: "pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex"}
    for motor_id in (1, 2, 3, 4):
        if motor_id in ticks:
            rad = tick_to_rad(ticks[motor_id], centers[motor_id])
            deg = math.degrees(rad)
            print(f"  [{motor_id}] {names_long[motor_id]:<15} pos={ticks[motor_id]:>5}  center={centers[motor_id]:.0f}  → {deg:>+7.1f}°")

    # Compute FK.
    xyz = fk_from_ticks_m(ticks, geo, cal)
    if xyz:
        print(f"\nFK end-effector position:")
        print(f"  x = {xyz[0]*1000:>+8.1f} mm  ({xyz[0]:>+.4f} m)")
        print(f"  y = {xyz[1]*1000:>+8.1f} mm  ({xyz[1]:>+.4f} m)")
        print(f"  z = {xyz[2]*1000:>+8.1f} mm  ({xyz[2]:>+.4f} m)")
        dist = math.sqrt(xyz[0]**2 + xyz[1]**2)
        print(f"  horizontal reach = {dist*1000:.1f} mm")
        print(f"  height above table = {xyz[2]*1000:.1f} mm")
    else:
        print("Missing joint data for FK (need IDs 1-4).")


if __name__ == "__main__":
    main()
