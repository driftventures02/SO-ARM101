#!/usr/bin/env python3
"""
Vision agent for SO-ARM101.

Captures webcam frames, sends them to GPT-4.1-nano with the arm's current
FK state, and executes tool calls (move_to_xyz, gripper, wrist_roll) in a
loop until the task is done.

Usage:
    uv run python basic_setup/08_vision_agent.py \
        --port /dev/cu.usbmodemXXXXX \
        --task "pick up the red block"
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import sys
import time
from pathlib import Path

import cv2
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv(Path(__file__).resolve().parent.parent.parent / ".env")

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from servo_utils import ServoBus
from kinematics.forward_kin import SoArm101Geometry, fk_from_ticks_m
from kinematics.inverse_kin import IKSolver
from kinematics.motion import (
    MOTOR_IDS,
    JointCalibration,
    read_positions,
    torque_on,
    torque_off,
    move,
    clamp_tick,
)

# ── Constants ────────────────────────────────────────────────────────

GRIPPER_OPEN = 2965
GRIPPER_CLOSE = 2089
WRIST_ROLL_ID = 5
GRIPPER_ID = 6

TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "move_to_xyz",
            "description": (
                "Move the robot arm end-effector to an XYZ position in meters. "
                "X is forward/backward, Y is left/right, Z is up/down. "
                "Origin is the base of the arm at table level. "
                "Typical reach is ~0.1 to 0.4 m forward, ±0.3 m sideways, 0 to 0.5 m high."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "Forward distance in meters"},
                    "y": {"type": "number", "description": "Left/right distance in meters (positive = left)"},
                    "z": {"type": "number", "description": "Height in meters (0 = table level)"},
                },
                "required": ["x", "y", "z"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "open_gripper",
            "description": "Open the gripper to release an object.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "close_gripper",
            "description": "Close the gripper to grab an object.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "rotate_wrist",
            "description": (
                "Rotate the wrist roll joint. Angle is in degrees, "
                "0 is neutral, positive is clockwise when viewed from above. "
                "Range is roughly -180 to +180 degrees."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "angle_degrees": {"type": "number", "description": "Wrist roll angle in degrees"},
                },
                "required": ["angle_degrees"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "done",
            "description": "Call this when the task is complete.",
            "parameters": {
                "type": "object",
                "properties": {
                    "message": {"type": "string", "description": "Summary of what was accomplished"},
                },
                "required": ["message"],
            },
        },
    },
]

SYSTEM_PROMPT = """\
You are controlling a SO-ARM101 robot arm via tool calls.
You see a camera feed and the arm's current state each turn.

Coordinate system (meters):
- X: forward (away from base), typically 0.1–0.4 m reachable
- Y: left/right (positive = left), typically ±0.3 m
- Z: height above table, typically 0–0.5 m

You have these tools: move_to_xyz, open_gripper, close_gripper, rotate_wrist, done.

Strategy:
- Look at the image to understand the scene
- Move carefully in small increments
- Open gripper before approaching an object to pick up
- Close gripper once positioned around the object
- Call done() when finished

Be concise in your reasoning. Execute one action at a time.\
"""


# ── Camera ───────────────────────────────────────────────────────────

def capture_frame(cap: cv2.VideoCapture) -> str:
    """Capture a frame, flip 180°, return as base64 JPEG string."""
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Failed to capture frame from webcam")
    frame = cv2.flip(frame, -1)  # 180° rotation
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return base64.b64encode(buf).decode("utf-8")


# ── Arm state ────────────────────────────────────────────────────────

def get_arm_state(bus, geo, cal) -> str:
    """Return a human-readable string of the arm's current state."""
    ticks = read_positions(bus)
    xyz = fk_from_ticks_m(ticks, geo, cal)
    lines = []
    if xyz:
        lines.append(f"End-effector: x={xyz[0]:.3f} y={xyz[1]:.3f} z={xyz[2]:.3f} m")
    gripper_tick = ticks.get(GRIPPER_ID, 0)
    openness = (gripper_tick - GRIPPER_CLOSE) / (GRIPPER_OPEN - GRIPPER_CLOSE)
    lines.append(f"Gripper: {'open' if openness > 0.5 else 'closed'} ({openness:.0%})")
    wrist_tick = ticks.get(WRIST_ROLL_ID, 2048)
    lines.append(f"Wrist roll tick: {wrist_tick}")
    return "\n".join(lines)


# ── Tool execution ───────────────────────────────────────────────────

def execute_tool(name: str, args: dict, bus, ik: IKSolver, geo, cal) -> str:
    """Execute a tool call and return the result string."""
    ticks = read_positions(bus)

    if name == "move_to_xyz":
        target = (args["x"], args["y"], args["z"])
        print(f"  → move_to_xyz({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})")
        goal = ik.solve(target, ticks)
        if goal is None:
            return "IK failed — target may be out of reach. Try a different position."
        # Keep wrist roll and gripper as-is
        goal[WRIST_ROLL_ID] = ticks.get(WRIST_ROLL_ID, 2048)
        goal[GRIPPER_ID] = ticks.get(GRIPPER_ID, GRIPPER_OPEN)
        move(bus, ticks, goal, steps=80, duration=1.5)
        new_ticks = read_positions(bus)
        new_xyz = fk_from_ticks_m(new_ticks, geo, cal)
        if new_xyz:
            return f"Moved. Now at x={new_xyz[0]:.3f} y={new_xyz[1]:.3f} z={new_xyz[2]:.3f} m"
        return "Moved."

    elif name == "open_gripper":
        print("  → open_gripper")
        goal = {GRIPPER_ID: GRIPPER_OPEN}
        move(bus, ticks, goal, motor_ids=[GRIPPER_ID], steps=40, duration=0.5)
        return "Gripper opened."

    elif name == "close_gripper":
        print("  → close_gripper")
        goal = {GRIPPER_ID: GRIPPER_CLOSE}
        move(bus, ticks, goal, motor_ids=[GRIPPER_ID], steps=40, duration=0.5)
        return "Gripper closed."

    elif name == "rotate_wrist":
        angle = args["angle_degrees"]
        print(f"  → rotate_wrist({angle:.1f}°)")
        # Convert degrees to ticks (4096 ticks per 360°)
        target_tick = 2048 + int(angle / 360.0 * 4096)
        target_tick = clamp_tick(WRIST_ROLL_ID, target_tick)
        goal = {WRIST_ROLL_ID: target_tick}
        move(bus, ticks, goal, motor_ids=[WRIST_ROLL_ID], steps=40, duration=0.5)
        return f"Wrist rotated to {angle:.1f}°."

    elif name == "done":
        print(f"  → done: {args.get('message', '')}")
        return "DONE"

    return f"Unknown tool: {name}"


# ── Main loop ────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Vision agent for SO-ARM101")
    parser.add_argument("--port", required=True, help="Serial port for servo bus")
    parser.add_argument("--task", required=True, help="Task description for the agent")
    parser.add_argument("--camera", type=int, default=1, help="Camera device index (default: 1)")
    parser.add_argument("--list-cameras", action="store_true", help="List available cameras and exit")
    parser.add_argument("--max-turns", type=int, default=20, help="Max agent turns")
    parser.add_argument("--model", default="gpt-4.1-nano", help="OpenAI model")
    args = parser.parse_args()

    # List cameras mode
    if args.list_cameras:
        print("Probing camera indices 0-4...")
        for i in range(5):
            c = cv2.VideoCapture(i)
            if c.isOpened():
                w = int(c.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
                ret, _ = c.read()
                print(f"  [{i}] {'OK' if ret else 'open but no frame'}  {w}x{h}")
                c.release()
            else:
                print(f"  [{i}] not available")
        return

    # Init camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera {args.camera}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Init arm
    bus = ServoBus(args.port)
    time.sleep(0.2)
    geo = SoArm101Geometry()
    cal = JointCalibration()
    ik = IKSolver(geo=geo, cal=cal)

    client = OpenAI()  # uses OPENAI_API_KEY env var

    try:
        current = read_positions(bus)
        torque_on(bus, current)

        messages = [{"role": "system", "content": SYSTEM_PROMPT}]

        for turn in range(1, args.max_turns + 1):
            print(f"\n{'='*60}")
            print(f"Turn {turn}/{args.max_turns}")
            print(f"{'='*60}")

            # Capture frame + arm state
            frame_b64 = capture_frame(cap)
            arm_state = get_arm_state(bus, geo, cal)
            print(f"Arm state:\n{arm_state}")

            # Build user message with image + state + task
            user_content = [
                {"type": "text", "text": f"Task: {args.task}\n\nCurrent arm state:\n{arm_state}"},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{frame_b64}", "detail": "low"}},
            ]
            messages.append({"role": "user", "content": user_content})

            # Call the model
            response = client.chat.completions.create(
                model=args.model,
                messages=messages,
                tools=TOOLS,
                tool_choice="auto",
                max_tokens=300,
            )
            msg = response.choices[0].message
            messages.append(msg)

            if msg.content:
                print(f"\nAgent: {msg.content}")

            # No tool call = model is done
            if not msg.tool_calls:
                print("\n✓ Agent finished (no more actions).")
                return

            for tc in msg.tool_calls:
                fn_name = tc.function.name
                fn_args = json.loads(tc.function.arguments)
                result = execute_tool(fn_name, fn_args, bus, ik, geo, cal)
                print(f"  Result: {result}")

                messages.append({
                    "role": "tool",
                    "tool_call_id": tc.id,
                    "content": result,
                })

                if result == "DONE":
                    print("\n✓ Task complete!")
                    return

            time.sleep(0.5)  # brief pause between turns

        print(f"\nReached max turns ({args.max_turns}).")

    finally:
        torque_off(bus)
        cap.release()
        bus.close()
        print("Torque OFF. Camera released.")


if __name__ == "__main__":
    main()
