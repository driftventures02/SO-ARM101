# SO-ARM101 Raw Tools — Zero Abstractions

These scripts talk to your STS3215 servos using **raw serial bytes**.
No LeRobot. No Feetech SDK. Just Python + pyserial.

Every packet sent/received is visible so you can see the actual protocol.

## Quick file map (1 line each)

- `00_find_and_scan.py` — Finds serial port and scans bus for motor IDs/baud/models/telemetry.
- `01_configure_ids.py` — Configures one motor at a time (persistent ID + baud + center position).
- `02_telemetry.py` — Live dashboard for all joints (position/speed/load/voltage/temp/errors).
- `03_keyboard_teleop.py` — Keyboard teleop with joint selection and per-joint movement keys.
- `03_easy_keyboard_teleop.py` — Direct key mapping teleop (no joint switching, faster to use).
- `04_record_replay.py` — Records hand-guided joint trajectories and replays them.

## Known bug we hit (and fix)

- If IDs reset after power cycle, `01_configure_ids.py` must use the STS3215 EEPROM lock register at address `55` (`0x37`), not `21`.
- Correct behavior: write lock `0` (unlock) before ID/baud writes, then write lock `1` after writes finish.
- Keep only one motor connected while assigning IDs, and close telemetry before running teleop (serial port is single-owner).

## Prerequisites

```bash
pip install pyserial
```

That's it. One dependency.

## The Order

### Step 0: Find your hardware
```bash
python 00_find_and_scan.py
```
Discovers your bus adapter, scans all baud rates and IDs, reads model number, 
position, voltage, temp from any motor it finds. **Run this first** to verify 
your hardware works.

### Step 1: Configure motor IDs (one at a time, in this order!)
```bash
# LeRobot order: gripper first (ID 6), shoulder_pan last (ID 1)
# This matches the daisy-chain wiring during assembly — you start from
# the end-effector and work toward the base.

python 01_configure_ids.py --port /dev/tty.usbmodemXXXXX --id 6   # gripper
# unplug from bus adapter, leave cable on motor, plug next motor into adapter
python 01_configure_ids.py --port /dev/tty.usbmodemXXXXX --id 5   # wrist_roll
python 01_configure_ids.py --port /dev/tty.usbmodemXXXXX --id 4   # wrist_flex
python 01_configure_ids.py --port /dev/tty.usbmodemXXXXX --id 3   # elbow_flex
python 01_configure_ids.py --port /dev/tty.usbmodemXXXXX --id 2   # shoulder_lift
python 01_configure_ids.py --port /dev/tty.usbmodemXXXXX --id 1   # shoulder_pan
```
Sets unique IDs, baud rate to 1Mbps, centers position to 2048.
**Connect only ONE motor at a time** — they all ship with the same default ID.

### Step 2: Live telemetry (after assembly)
```bash
python 02_telemetry.py --port /dev/tty.usbmodemXXXXX
python 02_telemetry.py --port /dev/tty.usbmodemXXXXX --verbose  # see raw bytes
```
Real-time dashboard: position, speed, load, voltage, temperature for all 6 joints.

### Step 3: Keyboard control
```bash
python 03_keyboard_teleop.py --port /dev/tty.usbmodemXXXXX
```
Move the arm from your keyboard. Toggle torque off to move by hand.
Press `v` for verbose mode to see every packet.

### Step 3 (simpler): Easy keyboard control
```bash
python 03_easy_keyboard_teleop.py --port /dev/tty.usbmodemXXXXX
```
Direct fixed keys per joint. No limits layer; minimal code path.

### Optional calibration tools
```bash
python 06_set_midpoint_pose.py --port /dev/tty.usbmodemXXXXX
python 05_capture_joint_limits.py --port /dev/tty.usbmodemXXXXX
```
Use only if you want midpoint or custom limit calibration workflows.

### Step 4: Record and replay
```bash
# Record: move arm by hand while positions are logged
python 04_record_replay.py --port /dev/tty.usbmodemXXXXX --mode record --file demo.json

# Replay: arm follows the recorded trajectory
python 04_record_replay.py --port /dev/tty.usbmodemXXXXX --mode replay --file demo.json
```

## What's Next (not in these scripts yet)

After you've done all of the above and understand the raw protocol:

1. **Forward Kinematics** — given joint angles, compute end-effector position in 3D space (DH parameters)
2. **Inverse Kinematics** — given a desired 3D position, compute joint angles (analytical or numerical)
3. **MuJoCo simulation** — load the URDF, simulate the arm, test control before running on hardware
4. **LLM-driven control** — LLM outputs target (x,y,z) → IK solves joint angles → servo commands
5. **Visual servoing** — camera feedback loop for closed-loop control

## Protocol Quick Reference

```
Packet: [0xFF] [0xFF] [ID] [LENGTH] [INSTRUCTION] [PARAMS...] [CHECKSUM]

Instructions:
  0x01 = PING       (no params)
  0x02 = READ       (start_addr, num_bytes)
  0x03 = WRITE      (start_addr, data_bytes...)
  0x82 = SYNC_READ  (start_addr, num_bytes, id1, id2, ...)
  0x83 = SYNC_WRITE (start_addr, data_len, id1, d1, d2, ..., id2, d1, d2, ...)

Key Registers:
  3-4   Model Number (2B)    → 777 = STS3215
  5     ID (1B)              → 0-253
  6     Baud Rate (1B)       → 0=1M, 1=500K, 2=250K, ...
  40    Torque Enable (1B)   → 0=off, 1=on
  42-43 Goal Position (2B)   → 0-4095
  56-57 Present Position (2B)
  58-59 Present Speed (2B)
  60-61 Present Load (2B)
  62    Present Voltage (1B) → value/10 = volts
  63    Present Temp (1B)    → degrees C

Checksum: ~(ID + LENGTH + INSTRUCTION + sum(PARAMS)) & 0xFF
```
