#!/usr/bin/env python3
"""
STEP 1: Configure motor IDs, one at a time.

Connect ONE motor to the bus adapter. Run this script.
It finds the motor, shows you its current state, and lets you
assign a new ID.

⚠️  ORDER MATTERS — configure in this exact sequence (matches LeRobot):
  python 01_configure_ids.py --port PORT --id 6   # gripper
  python 01_configure_ids.py --port PORT --id 5   # wrist_roll
  python 01_configure_ids.py --port PORT --id 4   # wrist_flex
  python 01_configure_ids.py --port PORT --id 3   # elbow_flex
  python 01_configure_ids.py --port PORT --id 2   # shoulder_lift
  python 01_configure_ids.py --port PORT --id 1   # shoulder_pan

WHY THIS ORDER? The daisy-chain cabling goes from the bus adapter → 
motor 1 (base) → motor 2 → ... → motor 6 (gripper). By configuring
from the tip backward, each motor you finish already has its cable
plugged in on one side, ready for the next motor in the chain.
After all 6 are done, you connect motor 1 to the bus adapter and
the whole chain is wired.

Usage:
    python 01_configure_ids.py --port /dev/tty.usbmodemXXXXX --id 6
"""

import serial
import time
import sys
import argparse

# ============================================================
# COLORS
# ============================================================
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
CYAN = "\033[96m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"

# ============================================================
# PROTOCOL CONSTANTS
# ============================================================
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

REG_MODEL_NUMBER = 3
REG_ID = 5
REG_BAUD_RATE = 6
REG_MIN_ANGLE = 9
REG_MAX_ANGLE = 11
REG_LOCK = 55              # STS3215 EEPROM lock flag (0=unlocked, 1=locked)
REG_GOAL_POSITION = 42
REG_PRESENT_POSITION = 56
REG_PRESENT_VOLTAGE = 62
REG_PRESENT_TEMP = 63
REG_TORQUE_ENABLE = 40     # Torque switch: 0=off, 1=on (128 triggers midpoint calibration)
REG_OFFSET = 31            # Homing offset

BAUD_RATES = {0: 1_000_000, 1: 500_000, 2: 250_000, 3: 128_000,
              4: 115_200, 5: 57_600, 6: 38_400, 7: 19_200}
TARGET_BAUD_INDEX = 0  # 1,000,000 — what LeRobot expects


def hex_dump(data: bytes, label: str = "", color: str = "") -> str:
    hex_str = " ".join(f"{b:02X}" for b in data)
    return f"{color}{label}{hex_str}{RESET}"


def build_packet(servo_id: int, instruction: int, params: list[int] = []) -> bytes:
    length = len(params) + 2
    checksum = ~(servo_id + length + instruction + sum(params)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, instruction] + params + [checksum])


def send_recv(ser: serial.Serial, packet: bytes, timeout: float = 0.05, verbose: bool = True) -> bytes:
    ser.reset_input_buffer()
    if verbose:
        print(hex_dump(packet, "    TX → ", RED))
    ser.write(packet)
    ser.flush()
    time.sleep(timeout)
    response = ser.read(ser.in_waiting or 64)
    if response:
        if response[:len(packet)] == packet:
            response = response[len(packet):]
        if response and verbose:
            print(hex_dump(response, "    RX ← ", GREEN))
    return response


def parse_response(data: bytes) -> dict | None:
    if len(data) < 6 or data[0] != 0xFF or data[1] != 0xFF:
        return None
    servo_id = data[2]
    length = data[3]
    error = data[4]
    params = list(data[5:5 + length - 2]) if length > 2 else []
    return {"id": servo_id, "error": error, "params": params}


def read_register(ser, motor_id, address, num_bytes, verbose=True):
    """Read a register and return the raw value."""
    pkt = build_packet(motor_id, INST_READ, [address, num_bytes])
    resp = send_recv(ser, pkt, verbose=verbose)
    parsed = parse_response(resp) if resp else None
    if parsed and len(parsed["params"]) >= num_bytes:
        if num_bytes == 1:
            return parsed["params"][0]
        elif num_bytes == 2:
            return parsed["params"][0] | (parsed["params"][1] << 8)
    return None


def write_register(ser, motor_id, address, value, num_bytes=1, verbose=True):
    """Write to a register."""
    if num_bytes == 1:
        params = [address, value & 0xFF]
    elif num_bytes == 2:
        params = [address, value & 0xFF, (value >> 8) & 0xFF]
    pkt = build_packet(motor_id, INST_WRITE, params)
    resp = send_recv(ser, pkt, verbose=verbose)
    return parse_response(resp) if resp else None


def find_motor(port: str) -> tuple[serial.Serial, int, int] | None:
    """Scan all baud rates to find a single connected motor."""
    for baud_idx, baud_rate in BAUD_RATES.items():
        try:
            ser = serial.Serial(port=port, baudrate=baud_rate, timeout=0.1)
        except:
            continue
        
        time.sleep(0.05)
        
        for motor_id in range(0, 20):
            pkt = build_packet(motor_id, INST_PING)
            ser.reset_input_buffer()
            ser.write(pkt)
            ser.flush()
            time.sleep(0.02)
            resp = ser.read(ser.in_waiting or 64)
            if resp:
                if resp[:len(pkt)] == pkt:
                    resp = resp[len(pkt):]
                if resp and len(resp) >= 6:
                    parsed = parse_response(resp)
                    if parsed:
                        return ser, motor_id, baud_rate
        
        ser.close()
    
    return None


def main():
    parser = argparse.ArgumentParser(description="Configure STS3215 motor ID")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/tty.usbmodemXXXXX)")
    parser.add_argument("--id", type=int, required=True, choices=range(1, 7),
                        help="Target ID to assign (1=shoulder_pan, 2=shoulder_lift, 3=elbow, 4=wrist_flex, 5=wrist_roll, 6=gripper)")
    args = parser.parse_args()

    joint_names = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
                   4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}

    # LeRobot configures in reverse order: 6(gripper) → 5 → 4 → 3 → 2 → 1(shoulder_pan)
    # This matches the daisy-chain cabling during assembly — you start at the
    # end-effector and work toward the base. Each motor you configure gets its
    # 3-pin cable left connected on one side, ready to chain to the next motor.
    LEROBOT_ORDER = [6, 5, 4, 3, 2, 1]

    print(f"""
{BOLD}{'='*60}
  MOTOR ID CONFIGURATOR — Raw Protocol
{'='*60}{RESET}

  Target: Motor ID {args.id} ({joint_names[args.id]})
  Port:   {args.port}

{DIM}  Make sure ONLY ONE motor is connected to the bus adapter!{RESET}
""")

    # Find the motor
    print(f"{BOLD}Scanning for motor...{RESET}")
    result = find_motor(args.port)
    
    if result is None:
        print(f"\n{RED}No motor found! Check connections and power supply.{RESET}")
        sys.exit(1)
    
    ser, current_id, current_baud = result
    print(f"\n{GREEN}Found motor: ID={current_id} at {current_baud:,} baud{RESET}\n")

    # Read current state
    print(f"{BOLD}Reading current motor state:{RESET}\n")
    
    print(f"  {CYAN}[Reading model number]{RESET}")
    model = read_register(ser, current_id, REG_MODEL_NUMBER, 2)
    model_name = "STS3215" if model == 777 else f"Unknown({model})"
    print(f"  → Model: {model} ({model_name})\n")
    
    print(f"  {CYAN}[Reading current ID]{RESET}")
    stored_id = read_register(ser, current_id, REG_ID, 1)
    print(f"  → Stored ID: {stored_id}\n")
    
    print(f"  {CYAN}[Reading baud rate index]{RESET}")
    baud_idx = read_register(ser, current_id, REG_BAUD_RATE, 1)
    print(f"  → Baud index: {baud_idx} ({BAUD_RATES.get(baud_idx, '???'):,} baud)\n")
    
    print(f"  {CYAN}[Reading position]{RESET}")
    pos = read_register(ser, current_id, REG_PRESENT_POSITION, 2)
    if pos is not None:
        print(f"  → Position: {pos}/4096 ({(pos/4096)*360:.1f}°)\n")
    
    print(f"  {CYAN}[Reading voltage & temperature]{RESET}")
    voltage = read_register(ser, current_id, REG_PRESENT_VOLTAGE, 1)
    temp = read_register(ser, current_id, REG_PRESENT_TEMP, 1)
    if voltage is not None:
        print(f"  → Voltage: {voltage/10.0}V, Temperature: {temp}°C\n")

    # Configure
    print(f"\n{BOLD}{'='*60}")
    print(f"  CONFIGURING MOTOR")
    print(f"{'='*60}{RESET}\n")
    
    changes = []
    if stored_id != args.id:
        changes.append(f"ID: {stored_id} → {args.id}")
    if baud_idx != TARGET_BAUD_INDEX:
        changes.append(f"Baud: {BAUD_RATES.get(baud_idx, '???'):,} → {BAUD_RATES[TARGET_BAUD_INDEX]:,}")
    changes.append(f"Position: {pos} → 2048 (centered)")
    
    print(f"  Planned changes:")
    for c in changes:
        print(f"    • {c}")
    
    confirm = input(f"\n  {YELLOW}Proceed? (y/n): {RESET}")
    if confirm.lower() != 'y':
        print("  Aborted.")
        ser.close()
        sys.exit(0)
    
    # Step 1: Unlock EEPROM
    print(f"\n  {CYAN}[1/5] Unlocking EEPROM (register {REG_LOCK} = 0){RESET}")
    print(f"  {DIM}The servo has two memory regions: RAM (volatile, lost on power-off) and")
    print(f"  EEPROM (non-volatile, persists). ID and baud rate are in EEPROM,")
    print(f"  which is locked by default to prevent accidental writes.{RESET}")
    write_register(ser, current_id, REG_LOCK, 0)
    time.sleep(0.15)
    
    # Step 2: Set baud rate (must do before ID change, since we're still at old baud)
    if baud_idx != TARGET_BAUD_INDEX:
        print(f"\n  {CYAN}[2/5] Setting baud rate to 1,000,000 (index 0){RESET}")
        print(f"  {DIM}Writing index 0 to register {REG_BAUD_RATE}. The servo will immediately")
        print(f"  switch to the new baud rate, so we need to reconnect.{RESET}")
        write_register(ser, current_id, REG_BAUD_RATE, TARGET_BAUD_INDEX)
        time.sleep(0.3)
        ser.close()
        ser = serial.Serial(port=args.port, baudrate=1_000_000, timeout=0.1)
        time.sleep(0.2)
        # After reconnect, explicitly unlock EEPROM again before ID write.
        write_register(ser, current_id, REG_LOCK, 0)
        time.sleep(0.15)
        print(f"  {GREEN}  Reconnected at 1,000,000 baud{RESET}")
    else:
        print(f"\n  {CYAN}[2/5] Baud rate already correct, skipping{RESET}")
    
    # Step 3: Set ID
    if current_id != args.id:
        print(f"\n  {CYAN}[3/5] Setting ID to {args.id}{RESET}")
        print(f"  {DIM}Writing {args.id} to register {REG_ID}. After this, the motor will only")
        print(f"  respond to packets addressed to ID {args.id}.{RESET}")
        write_register(ser, current_id, REG_ID, args.id)
        time.sleep(0.2)
        current_id = args.id  # Update for subsequent commands
        print(f"  {GREEN}  Motor now responds as ID {args.id}{RESET}")
    else:
        print(f"\n  {CYAN}[3/5] ID already correct, skipping{RESET}")
    
    # Step 4: Clear homing offset
    print(f"\n  {CYAN}[4/5] Clearing homing offset (register {REG_OFFSET} = 0){RESET}")
    write_register(ser, current_id, REG_OFFSET, 0, num_bytes=2)
    time.sleep(0.05)
    
    # Step 5: Move to center position (2048)
    print(f"\n  {CYAN}[5/5] Moving to center position (2048){RESET}")
    write_register(ser, current_id, REG_TORQUE_ENABLE, 1)
    time.sleep(0.05)
    print(f"  {DIM}Writing 2048 to Goal Position register ({REG_GOAL_POSITION}).{RESET}")
    print(f"  {DIM}The servo's internal PID controller will drive the motor to this position.{RESET}")
    print(f"  {YELLOW}  ⚠ MOTOR WILL MOVE! Make sure it's free to rotate.{RESET}")
    write_register(ser, current_id, REG_GOAL_POSITION, 2048, num_bytes=2)
    time.sleep(1.0)
    
    # Verify
    print(f"\n  {CYAN}[Verifying...]{RESET}")
    new_pos = read_register(ser, current_id, REG_PRESENT_POSITION, 2)
    new_id = read_register(ser, current_id, REG_ID, 1)
    new_baud = read_register(ser, current_id, REG_BAUD_RATE, 1)
    
    # Give EEPROM writes time to commit, then lock again.
    time.sleep(0.2)
    write_register(ser, current_id, REG_LOCK, 1, verbose=False)
    
    print(f"""
{GREEN}{BOLD}{'='*60}
  CONFIGURATION COMPLETE
{'='*60}{RESET}

  Motor ID:     {GREEN}{new_id}{RESET} (was {stored_id})
  Baud rate:    {GREEN}{BAUD_RATES.get(new_baud, '???'):,}{RESET}
  Position:     {GREEN}{new_pos}/4096 ({(new_pos/4096)*360:.1f}°){RESET}
  Joint:        {GREEN}{joint_names[args.id]}{RESET}

{YELLOW}  Now disconnect this motor and connect the next one.
  Label this motor as {BOLD}{joint_names[args.id]} (ID {args.id}){RESET}{YELLOW} with tape/marker.
  
  {"DONE! All 6 motors configured. Now assemble the arm." if args.id == 1 else f"Next: python 01_configure_ids.py --port {args.port} --id {LEROBOT_ORDER[LEROBOT_ORDER.index(args.id) + 1]}"}{RESET}
""")
    
    ser.close()


if __name__ == "__main__":
    main()
