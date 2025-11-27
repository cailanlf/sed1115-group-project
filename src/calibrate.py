from machine import PWM, Pin, ADC
from servo_translator import translate

shoulder = PWM(Pin(0), freq=50)
elbow = PWM(Pin(1), freq=50)

x = ADC(Pin(26))
y = ADC(Pin(27))

while True:
    shoulder_angle = x.read_u16() / 65535 * 180
    elbow_angle = y.read_u16() / 65535 * 180

    shoulder.duty_u16(translate(shoulder_angle))
    elbow.duty_u16(translate(elbow_angle))

    print(f"shoulder: {shoulder_angle:.2f}, elbow: {elbow_angle:.2f}")


