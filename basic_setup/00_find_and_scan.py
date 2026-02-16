#!/usr/bin/env python3
"""
STEP 0: Find your bus adapter and scan for motors.
Run this FIRST before anything else.

This script does NOT use LeRobot or the Feetech SDK.
It speaks raw serial bytes so you can see exactly what's happening on the wire.

Usage:
    python 00_find_and_scan.py
"""

import serial
import serial.tools.list_ports
import time
import sys

# ============================================================
# COLOR OUTPUT (so packets are visually distinguishable)
# ============================================================
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
CYAN = "\033[96m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"


def hex_dump(data: bytes, label: str = "", color: str = "") -> str:
    """Pretty-print bytes as hex with ASCII interpretation."""
    hex_str = " ".join(f"{b:02X}" for b in data)
    ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
    return f"{color}{label}{hex_str}  |{ascii_str}|{RESET}"


# ============================================================
# FEETECH PROTOCOL - RAW PACKET BUILDING
# ============================================================
# Packet format: [0xFF] [0xFF] [ID] [LENGTH] [INSTRUCTION] [PARAMS...] [CHECKSUM]
#
# The 0xFF 0xFF header is how the servo's UART receiver synchronizes —
# it watches the incoming byte stream for two consecutive 0xFF bytes,
# then knows the next byte is the ID field.
#
# LENGTH = number of bytes after length field = len(params) + 2 (instruction + checksum)
# CHECKSUM = ~(ID + LENGTH + INSTRUCTION + sum(PARAMS)) & 0xFF
#   This is a ones-complement checksum. If any byte gets corrupted in transit,
#   the checksum won't match and the servo ignores the packet.

INST_PING = 0x01       # "Are you there?" — servo responds with status
INST_READ = 0x02       # Read N bytes starting at address
INST_WRITE = 0x03      # Write bytes starting at address

# Servo register addresses (from the STS3215 datasheet)
REG_MODEL_NUMBER = 3       # 2 bytes — should be 777 (0x0309) for STS3215
REG_ID = 5                 # 1 byte — servo ID (0-253)
REG_BAUD_RATE = 6          # 1 byte — baud rate index
REG_PRESENT_POSITION = 56  # 2 bytes — current encoder value (0-4095)
REG_PRESENT_SPEED = 58     # 2 bytes
REG_PRESENT_LOAD = 60      # 2 bytes — current torque load
REG_PRESENT_VOLTAGE = 62   # 1 byte — voltage * 10
REG_PRESENT_TEMP = 63      # 1 byte — temperature in °C

# Baud rate lookup: index → actual baud rate
BAUD_RATES = {
    0: 1_000_000,
    1: 500_000,
    2: 250_000,
    3: 128_000,
    4: 115_200,
    5: 57_600,
    6: 38_400,
    7: 19_200,
}


def build_packet(servo_id: int, instruction: int, params: list[int] = []) -> bytes:
    """
    Build a raw Feetech/SCServo protocol packet.
    
    This is THE fundamental operation. Every single interaction with the servo
    — reading position, writing a target, changing ID, everything —
    goes through this packet format.
    """
    length = len(params) + 2  # params + instruction + checksum
    checksum = ~(servo_id + length + instruction + sum(params)) & 0xFF
    packet = bytes([0xFF, 0xFF, servo_id, length, instruction] + params + [checksum])
    return packet


def build_ping(servo_id: int) -> bytes:
    """PING packet — asks servo to respond with its status."""
    return build_packet(servo_id, INST_PING)


def build_read(servo_id: int, address: int, num_bytes: int) -> bytes:
    """
    READ packet — asks servo to return `num_bytes` starting at `address`.
    
    The servo's internal memory is a flat array of registers (like a tiny database).
    Each register has an address. We're just saying "give me bytes at address X".
    """
    return build_packet(servo_id, INST_READ, [address, num_bytes])


def parse_response(data: bytes) -> dict | None:
    """
    Parse a servo response packet.
    
    Response format: [0xFF] [0xFF] [ID] [LENGTH] [ERROR] [PARAMS...] [CHECKSUM]
    
    The ERROR byte is a bitfield:
      bit 0: Input Voltage Error
      bit 1: Angle Limit Error  
      bit 2: Overheating Error
      bit 3: Range Error
      bit 4: Checksum Error
      bit 5: Overload Error
      bit 6: Instruction Error
    """
    if len(data) < 6:
        return None
    if data[0] != 0xFF or data[1] != 0xFF:
        return None
    
    servo_id = data[2]
    length = data[3]
    error = data[4]
    params = list(data[5:5 + length - 2]) if length > 2 else []
    
    # Verify checksum
    expected_checksum = ~(servo_id + length + error + sum(params)) & 0xFF
    actual_checksum = data[5 + length - 2] if len(data) > 5 + length - 2 else -1
    
    error_flags = []
    if error & 0x01: error_flags.append("INPUT_VOLTAGE")
    if error & 0x02: error_flags.append("ANGLE_LIMIT")
    if error & 0x04: error_flags.append("OVERHEATING")
    if error & 0x08: error_flags.append("RANGE")
    if error & 0x10: error_flags.append("CHECKSUM")
    if error & 0x20: error_flags.append("OVERLOAD")
    if error & 0x40: error_flags.append("INSTRUCTION")
    
    return {
        "id": servo_id,
        "error": error,
        "error_flags": error_flags,
        "params": params,
        "checksum_ok": expected_checksum == actual_checksum,
    }


def send_and_receive(ser: serial.Serial, packet: bytes, timeout: float = 0.05) -> bytes:
    """
    Send a packet and read the response. Prints raw bytes in both directions.
    
    This is where the half-duplex dance happens:
    1. We write bytes to the serial port (TX)
    2. The USB-UART chip drives the data line
    3. We immediately switch to reading (RX)
    4. The servo responds on the same data line
    5. The USB-UART chip forwards it back to us over USB
    """
    # Flush any stale data in the buffer
    ser.reset_input_buffer()
    
    # TRANSMIT
    print(hex_dump(packet, "  TX → ", RED))
    ser.write(packet)
    ser.flush()  # Make sure all bytes are sent before we start listening
    
    # RECEIVE
    time.sleep(timeout)  # Wait for servo to process and respond
    response = ser.read(ser.in_waiting or 64)
    
    if response:
        # On half-duplex, we often see our own TX echoed back.
        # The Waveshare board's tri-state buffer should prevent this,
        # but some boards echo. Strip our own packet if present.
        if response[:len(packet)] == packet:
            response = response[len(packet):]
        
        if response:
            print(hex_dump(response, "  RX ← ", GREEN))
    
    return response


# ============================================================
# MAIN: FIND PORTS AND SCAN
# ============================================================

def find_serial_ports():
    """List all serial ports. On Mac, we're looking for /dev/tty.usbmodem* or /dev/cu.usbmodem*"""
    ports = serial.tools.list_ports.comports()
    usable = []
    for p in ports:
        # Filter to likely USB serial adapters
        if "usbmodem" in p.device or "usbserial" in p.device or "ttyACM" in p.device or "ttyUSB" in p.device or "COM" in p.device:
            usable.append(p)
    return usable


def scan_for_motors(port: str):
    """
    Try every baud rate and every ID (1-10) to find connected motors.
    Prints every packet sent and received.
    """
    found_motors = []
    
    for baud_idx, baud_rate in BAUD_RATES.items():
        print(f"\n{YELLOW}{'='*60}")
        print(f"  Trying baud rate: {baud_rate:,} (index {baud_idx})")
        print(f"{'='*60}{RESET}")
        
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
        except serial.SerialException as e:
            print(f"  {RED}Failed to open port at {baud_rate}: {e}{RESET}")
            continue
        
        time.sleep(0.1)  # Let the port settle after opening
        
        # Scan IDs 0-10 (factory default is usually 1, but could be 0 or others)
        for motor_id in range(0, 11):
            ping = build_ping(motor_id)
            print(f"\n  {CYAN}PING motor ID={motor_id}{RESET}")
            
            response = send_and_receive(ser, ping, timeout=0.02)
            
            if response and len(response) >= 6:
                parsed = parse_response(response)
                if parsed and parsed["checksum_ok"]:
                    print(f"  {GREEN}{BOLD}✓ FOUND MOTOR! ID={parsed['id']}, "
                          f"error=0x{parsed['error']:02X} {parsed['error_flags']}{RESET}")
                    
                    # Read model number
                    print(f"\n  {CYAN}READ model number (register {REG_MODEL_NUMBER}, 2 bytes){RESET}")
                    read_pkt = build_read(motor_id, REG_MODEL_NUMBER, 2)
                    resp = send_and_receive(ser, read_pkt)
                    if resp:
                        parsed_read = parse_response(resp)
                        if parsed_read and len(parsed_read["params"]) >= 2:
                            model = parsed_read["params"][0] | (parsed_read["params"][1] << 8)
                            model_name = "STS3215" if model == 777 else f"Unknown({model})"
                            print(f"  {GREEN}  Model: {model} → {model_name}{RESET}")
                    
                    # Read current position
                    print(f"\n  {CYAN}READ present position (register {REG_PRESENT_POSITION}, 2 bytes){RESET}")
                    read_pkt = build_read(motor_id, REG_PRESENT_POSITION, 2)
                    resp = send_and_receive(ser, read_pkt)
                    if resp:
                        parsed_read = parse_response(resp)
                        if parsed_read and len(parsed_read["params"]) >= 2:
                            pos = parsed_read["params"][0] | (parsed_read["params"][1] << 8)
                            degrees = (pos / 4096) * 360
                            print(f"  {GREEN}  Position: {pos}/4096 ({degrees:.1f}°){RESET}")
                    
                    # Read voltage and temperature
                    print(f"\n  {CYAN}READ voltage & temp (register {REG_PRESENT_VOLTAGE}, 2 bytes){RESET}")
                    read_pkt = build_read(motor_id, REG_PRESENT_VOLTAGE, 2)
                    resp = send_and_receive(ser, read_pkt)
                    if resp:
                        parsed_read = parse_response(resp)
                        if parsed_read and len(parsed_read["params"]) >= 2:
                            voltage = parsed_read["params"][0] / 10.0
                            temp = parsed_read["params"][1]
                            print(f"  {GREEN}  Voltage: {voltage}V, Temperature: {temp}°C{RESET}")
                    
                    found_motors.append({
                        "id": motor_id,
                        "baud_rate": baud_rate,
                        "baud_index": baud_idx,
                        "port": port,
                    })
            else:
                print(f"  {DIM}  (no response){RESET}")
        
        ser.close()
    
    return found_motors


def main():
    print(f"""
{BOLD}{'='*60}
  SO-ARM101 RAW SERIAL SCANNER
  No SDK. No abstractions. Just bytes on a wire.
{'='*60}{RESET}

{DIM}This script manually builds Feetech protocol packets and sends
them over your serial port. Every byte transmitted (TX) and 
received (RX) is printed so you can see the actual protocol.{RESET}
""")
    
    # Step 1: Find serial ports
    print(f"{BOLD}STEP 1: Finding serial ports...{RESET}\n")
    ports = find_serial_ports()
    
    if not ports:
        print(f"""
{RED}No USB serial devices found!{RESET}

Checklist:
  1. Is the Waveshare bus adapter plugged into your Mac via USB-C?
  2. Is the power supply connected to the bus adapter? (5V for 7.4V motors)
  3. Is at least one motor connected to the bus adapter via 3-pin cable?
  4. Are the jumpers on the adapter set to 'B' (USB) channel?

On Mac, you might need to install a CH340 driver if using a CH340-based adapter:
  brew install --cask wch-ch34x-usb-serial-driver

After connecting, the port should appear as /dev/tty.usbmodem* or /dev/cu.usbmodem*
""")
        
        print(f"\n{YELLOW}All serial ports on this system (including non-USB):{RESET}")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device} — {p.description} [{p.hwid}]")
        
        sys.exit(1)
    
    print(f"  Found {len(ports)} USB serial device(s):\n")
    for i, p in enumerate(ports):
        print(f"  [{i}] {BOLD}{p.device}{RESET}")
        print(f"      Description: {p.description}")
        print(f"      Hardware ID: {p.hwid}")
        print(f"      Manufacturer: {p.manufacturer}")
        print()
    
    if len(ports) == 1:
        port = ports[0].device
        print(f"  Using: {BOLD}{port}{RESET}")
    else:
        idx = int(input(f"  Select port [0-{len(ports)-1}]: "))
        port = ports[idx].device
    
    # Step 2: Scan for motors
    print(f"\n{BOLD}STEP 2: Scanning for motors on {port}...{RESET}")
    print(f"{DIM}(Trying all standard baud rates and IDs 0-10){RESET}")
    print(f"{DIM}Watch the TX/RX bytes — this is the raw protocol!{RESET}\n")
    
    found = scan_for_motors(port)
    
    # Summary
    print(f"\n\n{BOLD}{'='*60}")
    print(f"  SCAN COMPLETE")
    print(f"{'='*60}{RESET}\n")
    
    if found:
        print(f"  {GREEN}Found {len(found)} motor(s):{RESET}\n")
        for m in found:
            print(f"    Motor ID={m['id']} at {m['baud_rate']:,} baud on {m['port']}")
        print(f"""
{YELLOW}
IMPORTANT: If all motors have the same ID (probably 1), that's expected
for factory-fresh motors. You need to configure them ONE AT A TIME.

Next step: Run 01_configure_ids.py to assign unique IDs to each motor.
{RESET}""")
    else:
        print(f"""  {RED}No motors found!{RESET}
        
Possible issues:
  • No power supply connected (motors need 5V/12V external power, not just USB)
  • Cable not seated properly in the bus adapter or motor
  • Wrong jumper setting on the Waveshare board (need 'B' for USB)
  • Motor is defective
  • Try the other 3-pin connector on the motor
""")


if __name__ == "__main__":
    main()
