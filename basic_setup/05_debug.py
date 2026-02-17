from servo_utils import ServoBus, to_u16

PORT="/dev/cu.usbmodem5AE60854401"
MOTOR_IDS=[1,2,3,4,5,6]

# register addresses
REG_ID=5
REG_BAUD=6
REG_MIN=9
REG_MAX=11
REG_OFFSET=31
REG_MODE=33
REG_TORQUE_CAL=40
REG_GOAL=42
REG_PRESENT=56

bus=ServoBus(PORT)

def r1(mid, addr):
    p = bus.read_reg(mid, addr, 1)
    return p[0] if p else None

def r2(mid, addr):
    p = bus.read_reg(mid, addr, 2)
    return to_u16(p[0], p[1]) if p else None

print("id | baud | mode | min  | max  | offset | t/cal | goal | present")
print("---+------+-----+------+------+--------+------+------ +--------")
for mid in MOTOR_IDS:
    row = {
        "id": r1(mid, REG_ID),
        "baud": r1(mid, REG_BAUD),
        "mode": r1(mid, REG_MODE),
        "min": r2(mid, REG_MIN),
        "max": r2(mid, REG_MAX),
        "offset": r2(mid, REG_OFFSET),
        "tcal": r1(mid, REG_TORQUE_CAL),
        "goal": r2(mid, REG_GOAL),
        "present": r2(mid, REG_PRESENT),
    }
    print(f"{mid:>2} | {str(row['baud']):>4} | {str(row['mode']):>3} | "
          f"{str(row['min']):>4} | {str(row['max']):>4} | {str(row['offset']):>6} | "
          f"{str(row['tcal']):>4} | {str(row['goal']):>4} | {str(row['present']):>7}")

bus.close()
