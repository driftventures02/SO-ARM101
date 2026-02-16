# SO-ARM101 Kit: Deep Technical Setup Guide

## What You're Actually Building (The Big Picture)

You have a **leader-follower teleoperation system**. Two identical 6-DOF (degree of freedom) robot arms:

- **Leader arm**: You move this by hand. It has no power driving the joints — it's purely a sensor. Your hand movements get read as encoder positions.
- **Follower arm**: This one has powered motors. It mirrors whatever the leader arm does in real-time.

The entire point of this setup is **imitation learning data collection**. You teleoperate (move the leader), the follower copies you, cameras record everything, and you get `(observation, action)` pairs. You then train a neural network policy (like ACT, Diffusion Policy, or a VLA) on that data so the follower can eventually act autonomously.

The software stack is **LeRobot** (by Hugging Face) — a PyTorch-based framework that handles motor control, data recording, training, and policy deployment.

---

## Part 1: Understanding Your Hardware

### 1.1 The STS3215 Servo Motor — What's Inside

Each joint uses a **Feetech STS3215** serial bus servo. This is not a dumb hobby servo — it's a "smart servo" with an onboard microcontroller. Let's crack one open conceptually:

**Internal components:**

1. **Coreless DC Motor**: The actual electromagnetic motor. "Coreless" means the rotor doesn't have an iron core — instead, the copper windings are formed into a thin cylindrical shell. Why? Lower inertia (starts/stops faster), no cogging torque (smoother at low speeds), and higher power-to-weight ratio. The tradeoff is lower thermal mass, so they can overheat faster under sustained load.

2. **Metal Gear Train**: A reduction gearbox (the gear ratio differs by joint in the SO-101 — 1/147, 1/191, or 1/345 for the leader). This trades speed for torque. A 1/345 ratio means the motor spins 345 times for the output shaft to rotate once. Higher ratio = more torque but slower. The follower uses all-identical gear ratios (standard STS3215).

3. **12-bit Magnetic Encoder**: This is the key innovation over cheap hobby servos. Instead of a potentiometer (which has physical contact wear and limited range), the STS3215 uses a **radial magnet** on the output shaft and a **Hall effect sensor** on the PCB. The sensor reads the magnetic field orientation and converts it to a 12-bit digital value. That gives you **4096 discrete positions** over 360°, which means **0.088° resolution**. There's zero physical contact between the magnet and sensor, so it essentially never wears out.

4. **Onboard MCU (Microcontroller)**: A small processor running firmware that handles:
   - PID position control (reading encoder, computing error, driving motor via H-bridge)
   - Serial communication (parsing commands, sending back telemetry)
   - Protection logic (overcurrent >2A for 2s, overvoltage, overtemperature >70°C)
   - Multiple operating modes (position servo, closed-loop speed, open-loop speed, stepping)

5. **H-Bridge Driver**: A circuit that lets the MCU drive the DC motor in both directions (forward/reverse) by switching which polarity is applied.

6. **Two Serial Ports (3-pin connectors)**: Both are electrically identical and connected in parallel internally. This is what enables **daisy-chaining** — you plug one servo into the next, forming a bus.

**Why this servo and not alternatives?**

| Alternative | Why Not |
|---|---|
| **Hobby PWM servos** (SG90, MG996R) | No feedback — you send a pulse width and pray it got there. No position reading. Can't daisy-chain (each needs a dedicated PWM pin). Limited to ~270° range. |
| **Dynamixel servos** (Robotis) | Industry standard, excellent quality, great software. But **5-10x the price** ($50-150 each vs ~$15). The SO-ARM100 originally considered these but cost was prohibitive for a democratized platform. |
| **Stepper motors** | Precise open-loop positioning, but no built-in feedback unless you add an encoder. Need separate driver boards per motor. Heavy, bulky. No daisy-chain bus. |
| **Brushless DC (BLDC) + encoder** | High performance, but requires external motor controllers (FOC drivers), encoder interfaces, and much more complex wiring/software. Used in high-end arms like those from Boston Dynamics. |
| **Direct drive (no gears)** | Used in research arms like KUKA iiwa. Amazing backdrivability and force sensing, but motors are huge, expensive, and need custom control. |

The STS3215 hits a sweet spot: $15/motor, built-in encoder + MCU + communication, daisy-chainable, 360° range, good enough torque for a desktop arm.

### 1.2 The Communication Bus — UART, TTL, and Half-Duplex

#### What is UART?

**UART** = Universal Asynchronous Receiver/Transmitter. It's one of the simplest serial communication protocols:

- **Serial**: Data is sent one bit at a time over a wire (as opposed to parallel, where 8+ wires carry 8+ bits simultaneously).
- **Asynchronous**: There's no shared clock signal. Both sides agree on a **baud rate** (bits per second) beforehand. The STS3215 defaults to 1,000,000 baud (1 Mbps).
- **Frame format**: Each byte is sent as: 1 start bit (low) → 8 data bits → 1 stop bit (high). No parity by default.

#### What does "TTL level" mean?

TTL = Transistor-Transistor Logic. It means the voltage levels are:
- Logic HIGH = ~3.3V or 5V (depending on the MCU)
- Logic LOW = ~0V

This is as opposed to RS-232 (which uses ±12V) or RS-485 (differential signaling). TTL is simple but only works over short distances (<1 meter reliably) because it's single-ended (susceptible to noise).

#### What is "half-duplex"?

A communication channel can be:
- **Full-duplex**: Can send and receive simultaneously (two separate wires — TX and RX). Like a phone call.
- **Half-duplex**: Can only send OR receive at any given moment (one shared data wire). Like a walkie-talkie.
- **Simplex**: One direction only (like a broadcast radio).

The STS3215 uses **half-duplex UART on a single data wire**. This means:
1. The host (your computer, via the bus adapter) sends a command packet
2. The host then **switches to listening mode**
3. The addressed servo responds with a status packet
4. The line goes idle until the next command

This is why the bus adapter board has a **direction control** mechanism — a small circuit (usually a tri-state buffer) that switches between transmit and receive mode.

#### The Bus Architecture

All 6 servos on one arm share a single data wire. Each servo has a unique **ID** (1-6). When the host sends a packet, it includes the target servo's ID. All servos hear the packet, but only the addressed one responds. This is the same principle as I²C or RS-485 multi-drop.

**Packet format** (Feetech protocol):
```
[0xFF] [0xFF] [ID] [Length] [Instruction] [Param1] [Param2] ... [Checksum]
```

- `0xFF 0xFF` — Header bytes (sync markers so the receiver knows a packet is starting)
- `ID` — Target servo (0-253, or 0xFE for broadcast to all)
- `Length` — Number of bytes following (parameters + 1 for checksum)
- `Instruction` — What to do (PING, READ, WRITE, SYNC_READ, SYNC_WRITE, etc.)
- `Parameters` — Data (e.g., register address, values to write)
- `Checksum` — Simple sum-based error detection: `~(ID + Length + Instruction + Params) & 0xFF`

This protocol is nearly identical to Dynamixel Protocol 1.0 (Feetech essentially cloned it), which is why the LeRobot codebase uses the SCServo SDK that mirrors the Dynamixel SDK structure.

#### Alternatives to this bus setup

| Bus Type | Description | Trade-offs |
|---|---|---|
| **UART half-duplex (what we use)** | Single data wire, shared bus | Simple, cheap, but slow for many servos (must poll sequentially) |
| **RS-485** | Differential signaling, same protocol | Much better noise immunity, longer distances (up to 1200m). Used in industrial settings. Some Feetech servos support this. |
| **CAN bus** | Used in automotive/industrial robotics | True multi-master, hardware collision avoidance, very robust. Used in UR robots, many industrial arms. Requires CAN transceivers. |
| **EtherCAT** | Ethernet-based real-time fieldbus | Microsecond-level deterministic timing. Used in high-end industrial arms. Expensive, complex. |
| **SPI** | Synchronous, full-duplex, master-slave | Very fast, but point-to-point (can't easily daisy-chain). Good for IMUs, ADCs. |
| **I²C** | Two-wire, multi-device | Slower (100-400 kHz typical), shared bus. Good for sensors, not great for high-bandwidth motor control. |

### 1.3 The Bus Servo Adapter Board (Waveshare)

This board does three things:

1. **USB-to-UART conversion**: Contains a USB-to-serial chip (like CH340, CP2102, or FTDI). Your computer sees it as a virtual serial port (`/dev/ttyACM0` on Linux, `/dev/tty.usbmodemXXX` on Mac, `COM3` on Windows). The chip converts USB packets to/from UART signals.

2. **Half-duplex direction control**: Since the STS3215 uses one data wire for both TX and RX, the board has logic (usually a tri-state buffer like 74HC126) that switches between driving the line (transmitting) and high-impedance (listening for responses).

3. **Power distribution**: The board takes your external power supply (5V or 12V) and routes it to the servo bus. The servos draw power from the bus — the data line is separate from the power rails. The board has jumpers labeled "A" and "B" — **B (USB)** means the data comes from the USB-to-serial chip. "A" is for direct microcontroller connection (like an ESP32 or Arduino).

---

## Part 2: Software Setup — Step by Step

### 2.1 Install LeRobot

```bash
# Create a conda environment (isolated Python environment)
conda create -y -n lerobot python=3.10
conda activate lerobot

# Install ffmpeg (needed for video encoding during data collection)
conda install ffmpeg -c conda-forge

# Install LeRobot with Feetech motor support
pip install 'lerobot[feetech]'
```

**What's happening under the hood:**

- `conda create` makes an isolated Python environment so LeRobot's dependencies don't conflict with other projects
- `lerobot[feetech]` is a pip "extra" — it pulls in the **SCServo SDK** (`feetech-servo-sdk`), which is Feetech's Python library for speaking their servo protocol. This SDK provides the low-level `PacketHandler` and `PortHandler` classes that LeRobot wraps.
- The SDK itself uses **pyserial** underneath — Python's standard library for talking to serial ports. pyserial opens `/dev/ttyACM0`, sets the baud rate, and handles read/write of raw bytes.

**The abstraction stack:**
```
Your Python code
    ↓
LeRobot's FeetechMotorsBus class  (high-level: "read position of joint 3")
    ↓
SCServo SDK PacketHandler          (mid-level: "build packet, send, parse response")
    ↓
pyserial Serial object             (low-level: "write these bytes to /dev/ttyACM0")
    ↓
OS kernel serial driver            (kernel: "send these bytes over USB")
    ↓
USB-to-UART chip on adapter board  (hardware: convert USB to UART signals)
    ↓
Physical wire to servo             (electrical: TTL voltage levels on wire)
    ↓
Servo's onboard MCU                (firmware: parse packet, execute command)
```

### 2.2 Find Your Serial Ports

```bash
python -m lerobot.find_port
```

**What this script does:**

It enumerates all serial ports on your system (via `serial.tools.list_ports` from pyserial) and prints them. On Linux, USB serial devices show up as `/dev/ttyACM0`, `/dev/ttyUSB0`, etc. On Mac, they appear as `/dev/tty.usbmodemXXXXX`. On Windows, `COM3`, `COM4`, etc.

**Why do you need this?** The serial port path is how the OS identifies which physical USB device you're talking to. If you have two bus adapters plugged in (one for leader, one for follower), they'll show up as two different ports.

**What's happening at the OS level:** When you plug in the Waveshare adapter, the OS kernel detects a USB device, loads the appropriate driver (usually `cdc_acm` for ACM devices or `ch341` for CH340 chips), and creates a device file in `/dev/`. This file is a "character device" — reading from it gives you incoming serial data, writing to it sends serial data out.

### 2.3 Configure Motor IDs

This is the most critical step and people get confused by it constantly. Here's what's happening and why:

**The problem:** Every STS3215 ships from the factory with the **same default ID** (usually 1) and the **same default baud rate** (1,000,000). If you daisy-chain 6 motors with the same ID, you can't address them individually — they'd all respond to every command simultaneously, creating bus collisions and corrupted data.

**The solution:** Configure each motor **one at a time**, giving each a unique ID (1 through 6).

#### New method (LeRobot v0.5+):

```bash
lerobot-setup-motors \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodemXXXXX
```

#### Old method (still works):

```bash
python lerobot/scripts/configure_motor.py \
  --port /dev/tty.usbmodemXXXXX \
  --brand feetech \
  --model sts3215 \
  --baudrate 1000000 \
  --ID 1
```

**What this script does, step by step:**

1. **Opens the serial port** at a known baud rate via pyserial
2. **Scans all possible baud rates** (1M, 500K, 250K, 128K, 115200, 57600, 38400, 19200) and all possible motor IDs (0-253) to find the connected motor. It does this by sending PING packets at each combination and listening for a response.
3. **Reads the motor's model number** from its internal register (address 3-4) to verify it's actually an STS3215 (model number 777 in decimal)
4. **Writes the desired baud rate** to the motor's baud rate register (address 6). The value isn't the actual baud rate — it's an index: 0=1M, 1=500K, 2=250K, etc.
5. **Writes the desired ID** to the motor's ID register (address 5)
6. **Sets the position to 2048** (the midpoint of the 0-4096 range) — this centers the motor's range so you have ±180° of motion
7. **Sets homing offset to 0** — clears any existing offset calibration

**Why one motor at a time?** Because if two motors share the same ID and you send "change your ID to 2", BOTH motors change to ID 2 — now you have the same problem. You must connect, configure, disconnect, then connect the next.

**The 2048 midpoint explained:** The 12-bit encoder gives values 0 to 4095. The motor can only rotate within this range (it has software limits). 2048 is exactly in the middle, meaning you get ±2048 steps (±180°) of travel in either direction. When you set position=2048 during configuration, the motor physically rotates to this midpoint. The "one-key calibration" feature (writing 128 to register address 40) sets whatever position the motor is currently in as the new 2048 midpoint.

**Servo register map (key registers):**

| Address | Name | Size | Description |
|---|---|---|---|
| 3-4 | Model Number | 2 bytes | 777 for STS3215 |
| 5 | ID | 1 byte | 0-253 (0xFE = broadcast) |
| 6 | Baud Rate | 1 byte | Index (0=1Mbps) |
| 9-10 | Min Angle Limit | 2 bytes | Software limit (default 0) |
| 11-12 | Max Angle Limit | 2 bytes | Software limit (default 4095) |
| 21 | Lock | 1 byte | 0=EEPROM unlocked (can write persistent settings) |
| 33 | Operating Mode | 1 byte | 0=Position, 1=Speed CL, 2=Speed OL, 3=Step |
| 40 | Calibration | 1 byte | Write 128 to set current pos as midpoint |
| 42-43 | Goal Position | 2 bytes | Target position (0-4095) |
| 46-47 | Running Speed | 2 bytes | Movement speed |
| 56-57 | Present Position | 2 bytes | Current encoder reading |
| 58-59 | Present Speed | 2 bytes | Current speed |
| 60-61 | Present Load | 2 bytes | Current load (torque) |
| 62 | Present Voltage | 1 byte | Supply voltage × 10 |
| 63 | Present Temperature | 1 byte | Temperature in °C |

### 2.4 Assemble the Arms

I won't repeat the physical assembly (see the official video/guide), but the key electrical concept:

**Daisy-chain wiring:** Each motor has two 3-pin connectors (GND, VCC, DATA). You connect Motor 1 to the bus adapter, then Motor 1 to Motor 2, Motor 2 to Motor 3, and so on. The data line runs through all of them in series. Power is distributed to each motor through the same chain. The order of the chain doesn't matter for communication (IDs are already set), but it matters for mechanical assembly — you want short cable runs.

**Leader arm special note for SO-101:** The leader arm uses motors with **different gear ratios** for joints 1-3 (1/147, 1/191, 1/345) compared to the follower (all standard). In the SO-100, you had to physically **remove the gear train** from the leader motors to reduce friction (since you're moving them by hand). The SO-101 improved this with lower-ratio gears that are easier to backdrive.

### 2.5 Calibrate

```bash
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodemXXXXX \
  --robot.id=my_follower_arm
```

**What calibration does and why it's necessary:**

The raw encoder values (0-4095) are **arbitrary** — they depend on how you physically mounted the motor horn on the output shaft. If your motor 2 was mounted 15° off from the "expected" zero, every command will be 15° wrong. Calibration creates a mapping from raw encoder values to meaningful joint angles.

**The calibration process:**

1. **Move to the "middle of range" position**: You physically move each joint to the center of its mechanical range and press Enter. The script records these raw encoder values as the "neutral" positions.

2. **Move through full range**: You move each joint through its entire range of motion. The script records the min and max encoder values for each joint.

3. **Save calibration file**: The script writes a `.json` file to `~/.cache/calibration/` containing per-joint offsets and ranges. Every future command is adjusted by these offsets before being sent to the motors.

**Why not just set all motors to 2048 during assembly?** Because the motor horn (the cross-shaped piece connecting the motor shaft to the arm link) can only be installed in discrete positions (25 splines, so every 14.4°). You can't perfectly align it to 2048 during assembly. Calibration compensates for this mechanical imprecision.

**What the calibration data looks like:**

```json
{
  "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": -47, "range_min": 1024, "range_max": 3072},
  "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": 12, ...},
  ...
}
```

---

## Part 3: Understanding the LeRobot Software Architecture

### 3.1 The Layer Cake

```
┌─────────────────────────────────────────────┐
│  lerobot-teleop / lerobot-record / lerobot-train  │  ← CLI entry points
├─────────────────────────────────────────────┤
│  LeRobot Robot Interface (Robot base class)  │  ← Abstracts "any robot"
├─────────────────────────────────────────────┤
│  SO101Follower / SO101Leader                 │  ← Specific robot implementations
├─────────────────────────────────────────────┤
│  FeetechMotorsBus                            │  ← Motor communication layer
├─────────────────────────────────────────────┤
│  SCServo SDK (PacketHandler, PortHandler)    │  ← Feetech protocol layer
├─────────────────────────────────────────────┤
│  pyserial (Serial)                           │  ← OS serial port access
├─────────────────────────────────────────────┤
│  OS Kernel (USB-CDC-ACM driver)              │  ← Hardware abstraction
├─────────────────────────────────────────────┤
│  USB → UART chip → Servo bus                 │  ← Physical hardware
└─────────────────────────────────────────────┘
```

### 3.2 Key Files and What They Do

#### `lerobot/robots/so101_follower/so101_follower.py`

This is the **robot implementation** — it tells LeRobot what an SO-101 follower arm looks like:

```python
class SO101Follower(Robot):
    def __init__(self, config):
        self.bus = FeetechMotorsBus(
            port=config.port,
            motors={
                "shoulder_pan":  Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex":    Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex":    Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll":    Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_M100_100),
            },
            calibration=self.calibration,
        )
```

**What `Motor(1, "sts3215", MotorNormMode.RANGE_M100_100)` means:**
- `1` = servo ID on the bus
- `"sts3215"` = motor model (determines register addresses, resolution=4096, etc.)
- `MotorNormMode.RANGE_M100_100` = normalize raw positions to [-100, 100] range. This is crucial — policies trained on this arm can potentially transfer to other arms because the action space is normalized, not in raw servo ticks.

**Key methods:**

- `connect()`: Opens serial port, pings all 6 motors, verifies they respond, loads calibration
- `get_observation()`: Calls `bus.sync_read("Present_Position")` to read all 6 joint positions in one bus transaction, also reads camera frames
- `send_action(action_dict)`: Writes target positions to all 6 motors via `bus.sync_write("Goal_Position", values)`

#### `lerobot/motors/feetech.py` (FeetechMotorsBus)

This is the **motor bus abstraction layer**. It translates high-level operations ("read Present_Position for all motors") into low-level serial commands.

**Key concept — sync_read vs individual read:**

- **Individual read**: Send a READ packet to motor 1, wait for response. Then motor 2, wait. Then motor 3... This is slow because you have the serial turnaround time for each motor.
- **sync_read**: Send a single SYNC_READ packet that addresses all 6 motors at once. Each motor responds in turn (in ID order). Much faster because there's only one command packet.

Similarly, **sync_write** sends all 6 target positions in one packet, and all motors start moving simultaneously. This is critical for smooth coordinated motion.

**The read/write flow in detail:**

```python
def sync_read(self, data_name):
    # 1. Look up the register address for "Present_Position" → address 56, 2 bytes
    # 2. Build a SYNC_READ packet: [0xFF][0xFF][0xFE][len][0x82][addr_lo][addr_hi][len][id1][id2]...
    # 3. Send the packet over serial
    # 4. Read 6 response packets (one per motor)
    # 5. Parse each response to extract the 2-byte position value
    # 6. Apply calibration offsets
    # 7. Normalize to the configured range (e.g., -100 to 100)
    # 8. Return as a dictionary: {"shoulder_pan": 0.45, "shoulder_lift": -12.3, ...}
```

#### `lerobot/scripts/setup_motors.py` (or `lerobot_setup_motors.py`)

This is the **motor configuration script** — runs during initial setup.

The newer version is interactive:
1. It knows the motor layout for `so101_follower` (which joints exist, their IDs)
2. It prompts you: "Connect the 'gripper' motor only and press enter"
3. When you press enter, it scans the bus for any motor (there should be exactly one)
4. It reads the motor's current ID and baud rate
5. It writes the correct ID (e.g., 6 for gripper) and sets baud rate to 1,000,000
6. It sets the position to 2048 and clears offsets
7. It prompts for the next motor

The older `configure_motor.py` version did the same thing but you had to manually specify the ID via command-line argument each time.

#### `lerobot/scripts/lerobot_calibrate.py`

Runs the calibration process described above. It:
1. Connects to the bus
2. Disables torque on all motors (so you can freely move them)
3. Prompts you to move to the neutral position
4. Reads and records all encoder values
5. Prompts you to sweep each joint through its full range
6. Records min/max for each joint
7. Computes offsets and saves to JSON

### 3.3 The Teleoperation Loop

When you run `lerobot-teleop`, here's what happens at ~30-60Hz:

```
LOOP:
  1. leader_positions = leader_bus.sync_read("Present_Position")   # Read 6 encoders
  2. follower_actions = apply_mapping(leader_positions)              # Map leader → follower
  3. follower_bus.sync_write("Goal_Position", follower_actions)     # Command 6 motors
  4. sleep(1/frequency)                                             # Maintain loop rate
```

**The mapping** handles the fact that leader and follower may have different calibration offsets, different drive directions, or that the leader might be gearless (SO-100) vs geared (SO-101).

### 3.4 The Data Collection Loop

When you run `lerobot-record`:

```
LOOP:
  1. leader_positions = leader.get_observation()        # Leader joint angles
  2. follower.send_action(map(leader_positions))         # Move follower
  3. follower_obs = follower.get_observation()           # Follower joint angles + cameras
  4. dataset.add_frame(
       observation=follower_obs,                         # What the robot "sees"
       action=follower_obs["positions"],                 # What joints it should reach
     )
  5. If episode_done: dataset.save_episode()
```

The dataset is saved in **LeRobotDataset format** — Parquet files for tabular data (joint positions, actions) and MP4 for camera video. This can be pushed to the Hugging Face Hub.

### 3.5 Training

```bash
lerobot-train \
  --policy=act \
  --dataset.repo_id=your_username/your_dataset
```

This trains a policy (e.g., **ACT** — Action Chunking with Transformers) on your collected data. The policy learns: given camera images and current joint positions (observation), predict the next sequence of joint positions (action chunk).

The trained model is a PyTorch checkpoint that can be loaded for inference:

```python
obs = robot.get_observation()
action = policy.select_action(obs)  # Neural network forward pass
robot.send_action(action)
```

---

## Part 4: Common Gotchas and Troubleshooting

### Motor Not Detected
- **Check jumpers**: The Waveshare board jumpers must be on "B" (USB) channel
- **Check power**: The motors need external power (5V or 12V PSU), not just USB power. USB only powers the communication chip.
- **Check cable orientation**: The 3-pin cable must match GND-VCC-DATA on both ends
- **Only one motor connected**: During setup, if you have two motors with the same ID (factory default), the bus will have collisions

### Servo Jitters or Oscillates
- PID tuning might be off. The default PID works for most cases, but heavy loads can cause oscillation.
- Power supply might be sagging under load. A 5V 5A supply is minimum for 6 motors.

### "Input voltage error"
- You're using the wrong power supply. 7.4V motors need ~5V supply, 12V motors need ~12V. Plugging 12V into 7.4V motors will likely damage them.

### Calibration Feels Wrong
- Re-run calibration. Make sure you're truly moving each joint through its FULL mechanical range during the sweep phase.
- Check that no motor horn is slipping on the shaft (tighten the center screw).

---

## Part 5: The Complete Conceptual Map

Here's how everything connects from electrons to AI:

```
[Your hand moves leader arm]
         ↓
[Magnetic encoder reads angle]  ← Hall effect sensor + magnet, 4096 steps
         ↓
[Servo MCU stores in register 56-57]  ← Onboard microcontroller
         ↓
[Host sends SYNC_READ via UART]  ← Half-duplex TTL, 1Mbps
         ↓
[USB-UART chip converts to USB]  ← CH340/CP2102 on Waveshare board
         ↓
[pyserial reads bytes]  ← Python serial library
         ↓
[SCServo SDK parses packet]  ← Feetech protocol layer
         ↓
[FeetechMotorsBus applies calibration]  ← LeRobot motor abstraction
         ↓
[SO101Leader.get_observation() returns dict]  ← LeRobot robot interface
         ↓
[Teleoperation loop maps leader→follower]  ← Control logic
         ↓
[SO101Follower.send_action(positions)]  ← LeRobot robot interface
         ↓
[FeetechMotorsBus.sync_write()]  ← Builds SYNC_WRITE packet
         ↓
[SCServo SDK serializes packet]  ← [0xFF][0xFF][0xFE][len][0x83]...
         ↓
[pyserial writes bytes to /dev/ttyACM1]
         ↓
[USB-UART chip converts to UART]
         ↓
[All 6 follower servos receive simultaneously]
         ↓
[Each servo's PID controller drives to goal position]
         ↓
[DC motor spins through gear train]  ← Coreless motor + metal gears
         ↓
[Follower arm moves to match leader]

Meanwhile, cameras capture frames → stored as observations →
used to train policy → policy replaces human in the loop
```

---

## Appendix: Quick Reference Commands

```bash
# 1. Install
conda create -y -n lerobot python=3.10 && conda activate lerobot
conda install ffmpeg -c conda-forge
pip install 'lerobot[feetech]'

# 2. Find ports
python -m lerobot.find_port

# 3. Setup motors (repeat for leader with --teleop.type=so101_leader)
lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0

# 4. Calibrate follower
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower

# 5. Calibrate leader
lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=my_leader

# 6. Teleoperate
lerobot-teleop --robot.type=so101_follower --robot.port=/dev/ttyACM0 --teleop.type=so101_leader --teleop.port=/dev/ttyACM1

# 7. Record dataset
lerobot-record --robot.type=so101_follower --robot.port=/dev/ttyACM0 --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --dataset.repo_id=your_name/your_dataset

# 8. Train
lerobot-train --policy=act --dataset.repo_id=your_name/your_dataset

# 9. Deploy
lerobot-eval --policy.path=outputs/train/checkpoints/last/pretrained_model --robot.type=so101_follower --robot.port=/dev/ttyACM0
```
