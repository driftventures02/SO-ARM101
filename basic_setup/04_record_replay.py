#!/usr/bin/env python3
"""
STEP 4: Record and Replay.

This is the simplest possible "teach and repeat" — no neural networks needed.

1. RECORD mode: Torque off, move arm by hand, positions are logged at N Hz
2. REPLAY mode: Torque on, play back the recorded trajectory

This is how industrial robots have been programmed since the 1970s.
It's the precursor to imitation learning — except instead of a neural network
generalizing from demonstrations, we're just replaying the exact trajectory.

Usage:
    python 04_record_replay.py --port /dev/tty.usbmodemXXXXX --mode record --file trajectory.json
    python 04_record_replay.py --port /dev/tty.usbmodemXXXXX --mode replay --file trajectory.json
"""

import serial
import time
import json
import sys
import argparse

# ============================================================
# COLORS
# ============================================================
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"

INST_READ = 0x02
INST_WRITE = 0x03

REG_TORQUE_ENABLE = 40
REG_GOAL_POSITION = 42
REG_PRESENT_POSITION = 56

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper"]


def build_packet(servo_id, instruction, params=[]):
    length = len(params) + 2
    checksum = ~(servo_id + length + instruction + sum(params)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, instruction] + params + [checksum])


def send_recv(ser, packet, timeout=0.008):
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    time.sleep(timeout)
    response = ser.read(ser.in_waiting or 64)
    if response and response[:len(packet)] == packet:
        response = response[len(packet):]
    return response


def parse_response(data):
    if len(data) < 6 or data[0] != 0xFF or data[1] != 0xFF:
        return None
    return {"id": data[2], "error": data[4],
            "params": list(data[5:5 + data[3] - 2]) if data[3] > 2 else []}


def read_positions(ser, motor_ids):
    positions = {}
    for mid in motor_ids:
        pkt = build_packet(mid, INST_READ, [REG_PRESENT_POSITION, 2])
        resp = send_recv(ser, pkt)
        if resp:
            p = parse_response(resp)
            if p and len(p["params"]) >= 2:
                positions[mid] = p["params"][0] | (p["params"][1] << 8)
    return positions


def write_position(ser, motor_id, position):
    position = max(0, min(4095, position))
    pkt = build_packet(motor_id, INST_WRITE, [
        REG_GOAL_POSITION, position & 0xFF, (position >> 8) & 0xFF
    ])
    send_recv(ser, pkt)


def set_torque(ser, motor_ids, enable):
    for mid in motor_ids:
        pkt = build_packet(mid, INST_WRITE, [REG_TORQUE_ENABLE, 1 if enable else 0])
        send_recv(ser, pkt)


def record_trajectory(ser, motor_ids, hz=20):
    """
    Record a trajectory by reading joint positions while human moves the arm.
    
    This is "kinesthetic teaching" — the simplest form of robot programming.
    The arm is in zero-torque mode so you can freely move it. We sample
    the encoder positions at a fixed rate and store them.
    
    The output is a list of "waypoints" — each waypoint is a dict of
    {joint_name: position} along with a timestamp.
    """
    print(f"\n{BOLD}RECORDING MODE{RESET}")
    print(f"{DIM}Torque is OFF. Move the arm freely.{RESET}")
    print(f"{DIM}Recording at {hz} Hz.{RESET}")
    print(f"{YELLOW}Press Enter to START recording, then Enter again to STOP.{RESET}\n")
    
    # Disable torque
    set_torque(ser, motor_ids, False)
    time.sleep(0.1)
    
    input(f"  {GREEN}[Press Enter to start recording]{RESET}")
    
    trajectory = []
    period = 1.0 / hz
    start_time = time.time()
    frame = 0
    
    print(f"\n  {RED}● RECORDING...{RESET} (Press Ctrl+C to stop)\n")
    
    try:
        while True:
            t0 = time.time()
            
            positions = read_positions(ser, motor_ids)
            timestamp = time.time() - start_time
            
            waypoint = {
                "timestamp": round(timestamp, 4),
                "frame": frame,
                "positions": {JOINT_NAMES[i]: positions.get(motor_ids[i], 2048)
                             for i in range(6)},
            }
            trajectory.append(waypoint)
            frame += 1
            
            # Live display
            pos_str = "  ".join(f"{JOINT_NAMES[i][:4]}={positions.get(motor_ids[i], 0):>5}"
                               for i in range(6))
            print(f"\r  t={timestamp:>6.2f}s  frame={frame:>5}  {pos_str}", end="", flush=True)
            
            elapsed = time.time() - t0
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        pass
    
    duration = time.time() - start_time
    actual_hz = len(trajectory) / duration if duration > 0 else 0
    
    print(f"\n\n  {GREEN}■ Recording stopped.{RESET}")
    print(f"  Duration: {duration:.1f}s")
    print(f"  Frames: {len(trajectory)}")
    print(f"  Actual Hz: {actual_hz:.1f}")
    
    return trajectory


def replay_trajectory(ser, motor_ids, trajectory, speed=1.0):
    """
    Replay a recorded trajectory.
    
    This is pure open-loop playback — we don't check if the arm actually
    reached each waypoint, we just blast through the positions at the
    original timing. This is the fundamental limitation vs. a learned policy:
    if anything is different (object moved, arm bumped), replay fails.
    
    A learned policy (like ACT or Diffusion Policy) would use camera feedback
    to adjust — that's the whole point of imitation learning vs. teach-and-repeat.
    """
    if not trajectory:
        print(f"{RED}Empty trajectory!{RESET}")
        return
    
    print(f"\n{BOLD}REPLAY MODE{RESET}")
    print(f"  Frames: {len(trajectory)}")
    print(f"  Duration: {trajectory[-1]['timestamp']:.1f}s (at {speed}x speed)")
    print(f"\n{YELLOW}  ⚠ ARM WILL MOVE! Clear the workspace.{RESET}")
    
    # First, move to start position slowly
    start_positions = trajectory[0]["positions"]
    print(f"\n  Moving to start position...")
    
    # Enable torque
    set_torque(ser, motor_ids, True)
    time.sleep(0.1)
    
    # Read current position and interpolate to start
    current = read_positions(ser, motor_ids)
    steps = 50
    for s in range(steps):
        alpha = (s + 1) / steps
        for i, mid in enumerate(motor_ids):
            name = JOINT_NAMES[i]
            cur = current.get(mid, 2048)
            target = start_positions.get(name, 2048)
            interp = int(cur + alpha * (target - cur))
            write_position(ser, mid, interp)
        time.sleep(0.02)
    
    time.sleep(0.5)
    
    input(f"\n  {GREEN}[Press Enter to replay]{RESET}")
    
    print(f"\n  {CYAN}▶ REPLAYING...{RESET}\n")
    
    start_time = time.time()
    frame_idx = 0
    
    try:
        while frame_idx < len(trajectory):
            elapsed = (time.time() - start_time) * speed
            
            # Find the frame closest to current time
            while frame_idx < len(trajectory) - 1 and trajectory[frame_idx + 1]["timestamp"] <= elapsed:
                frame_idx += 1
            
            wp = trajectory[frame_idx]
            
            # Write positions
            for i, mid in enumerate(motor_ids):
                name = JOINT_NAMES[i]
                if name in wp["positions"]:
                    write_position(ser, mid, wp["positions"][name])
            
            # Display
            pos_str = "  ".join(f"{JOINT_NAMES[i][:4]}={wp['positions'].get(JOINT_NAMES[i], 0):>5}"
                               for i in range(6))
            progress = (frame_idx + 1) / len(trajectory) * 100
            print(f"\r  [{progress:>5.1f}%] frame={frame_idx:>5}/{len(trajectory)}  {pos_str}", 
                  end="", flush=True)
            
            frame_idx += 1
            
            # Maintain timing
            target_time = wp["timestamp"] / speed
            wait = target_time - (time.time() - start_time)
            if wait > 0:
                time.sleep(wait)
    
    except KeyboardInterrupt:
        print(f"\n\n  {YELLOW}Replay interrupted.{RESET}")
    
    print(f"\n\n  {GREEN}■ Replay complete.{RESET}")


def main():
    parser = argparse.ArgumentParser(description="Record and replay arm trajectories")
    parser.add_argument("--port", required=True)
    parser.add_argument("--mode", required=True, choices=["record", "replay"])
    parser.add_argument("--file", required=True, help="Trajectory JSON file")
    parser.add_argument("--hz", type=float, default=20, help="Recording rate (Hz)")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier")
    args = parser.parse_args()
    
    motor_ids = [1, 2, 3, 4, 5, 6]
    
    print(f"{BOLD}Connecting to {args.port}...{RESET}")
    ser = serial.Serial(port=args.port, baudrate=1_000_000, timeout=0.1)
    time.sleep(0.2)
    
    # Verify motors
    positions = read_positions(ser, motor_ids)
    found = len(positions)
    print(f"{GREEN}Found {found}/6 motors.{RESET}")
    
    if found < 6:
        print(f"{YELLOW}Warning: Only {found} motors responding. "
              f"Missing: {[m for m in motor_ids if m not in positions]}{RESET}")
    
    try:
        if args.mode == "record":
            trajectory = record_trajectory(ser, motor_ids, hz=args.hz)
            
            # Save
            with open(args.file, "w") as f:
                json.dump({
                    "metadata": {
                        "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                        "hz": args.hz,
                        "n_frames": len(trajectory),
                        "duration_s": trajectory[-1]["timestamp"] if trajectory else 0,
                        "joint_names": JOINT_NAMES,
                    },
                    "trajectory": trajectory,
                }, f, indent=2)
            
            print(f"\n  {GREEN}Saved to {args.file}{RESET}")
            print(f"\n  To replay: python 04_record_replay.py --port {args.port} --mode replay --file {args.file}")
        
        elif args.mode == "replay":
            with open(args.file, "r") as f:
                data = json.load(f)
            
            trajectory = data["trajectory"]
            meta = data.get("metadata", {})
            
            print(f"\n  Loaded trajectory: {meta.get('n_frames', len(trajectory))} frames, "
                  f"{meta.get('duration_s', 0):.1f}s, recorded at {meta.get('hz', '?')} Hz")
            
            replay_trajectory(ser, motor_ids, trajectory, speed=args.speed)
    
    finally:
        print(f"\n{YELLOW}Disabling torque...{RESET}")
        set_torque(ser, motor_ids, False)
        ser.close()
        print(f"{GREEN}Done.{RESET}")


if __name__ == "__main__":
    main()
