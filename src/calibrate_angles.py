from machine import Pin, PWM
from servo_translator import translate
import time
import math

shulder_servo = PWM(Pin(0), freq=50)
elbow_servo = PWM(Pin(1), freq=50)

shoulder_offset, elbow_offset = 42, 0


def calibrate(
        filename: str = "calibration_data.csv",
        La: float = 155, Lb: float = 155
):
    with open(filename, 'w', encoding='utf-8') as f:
        # write column header
        f.write("desired, shulder, elbow \n")

        for angle in range(20, 181, 10):
            shulder_servo.duty_u16(translate(angle + shoulder_offset))
            elbow_servo.duty_u16(translate(angle + elbow_offset))
            print("current angle is: ", angle)
            AC = float(input("Enter length AC(mm): "))
            Bx = float(input("Enter distance from B to intersection of extended CB with X-axis(mm): "))

            shoulder_rad = math.atan(Bx / La)
            elbow_rad = math.acos(
                (La**2 + Lb**2 - AC**2) / (2 * La * Lb)
            )

            shoulder_deg = math.degrees(shoulder_rad)
            elbow_deg = math.degrees(elbow_rad)

            f.write(f"{angle}, {shoulder_deg}, {elbow_deg} \n")

            print("-------------------------------------------------")

def calibration_setup(filename: str = "calibration_data.csv"):
    errors = {
        "shulder": {},
        "elbow": {}
    }

    with open(filename, 'r', newline="", encoding='utf-8') as f:
        next(f)  # skip header line

        for line in f:
            line = line.strip()
            row = line.split(',')

            # conert every elements in row to int
            row = [float(x) for x in row]

            # key: desired value, value: (shulder, elbow)
            errors["shulder"][row[0]] = row[1]
            errors["elbow"][row[0]] = row[2]

    return errors

def send_compensated_angle(table: dict, angle: float):
    # If angle is exactly a calibration point
    if angle in table:
        return table[angle]
    
    # linear interpolation
    x0 = angle // 10 + 10
    x1 = x0 + 10
    y0 = table[x0]
    y1 = table[x1]

    ratio = (angle - x0) / (x1 - x0)
    error = y0 + ratio * (y1 - y0)

    return error
        

calibrate()
# error_tables = calibration_setup()

# print(send_compensated_angle(error_tables["shulder"], 10))
