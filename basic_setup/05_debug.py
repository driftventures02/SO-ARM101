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

def r1(motor_id, addr):
    p = bus.read_reg(motor_id, addr, 1)
    return p[0] if p else None

def r2(motor_id, addr):
    p = bus.read_reg(motor_id, addr, 2)
    return to_u16(p[0], p[1]) if p else None

print("id | baud | mode | min  | max  | offset | t/cal | goal | present")
print("---+------+-----+------+------+--------+------+------ +--------")
for motor_id in MOTOR_IDS:
    row = {
        "id": r1(motor_id, REG_ID),
        "baud": r1(motor_id, REG_BAUD),
        "mode": r1(motor_id, REG_MODE),
        "min": r2(motor_id, REG_MIN),
        "max": r2(motor_id, REG_MAX),
        "offset": r2(motor_id, REG_OFFSET),
        "tcal": r1(motor_id, REG_TORQUE_CAL),
        "goal": r2(motor_id, REG_GOAL),
        "present": r2(motor_id, REG_PRESENT),
    }
    print(f"{motor_id:>2} | {str(row['baud']):>4} | {str(row['mode']):>3} | "
          f"{str(row['min']):>4} | {str(row['max']):>4} | {str(row['offset']):>6} | "
          f"{str(row['tcal']):>4} | {str(row['goal']):>4} | {str(row['present']):>7}")

bus.close()
