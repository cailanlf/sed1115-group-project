from machine import PWM, Pin, ADC
from servo_translator import translate

shoulder = PWM(Pin(0), freq=50)
elbow = PWM(Pin(1), freq=50)
wrist = PWM(Pin(2), freq=50)

x = ADC(Pin(26))
y = ADC(Pin(27))

while True:
    angle = x.read_u16() / 65535 * 180
    # elbow_angle = y.read_u16() / 65535 * 180

    wrist.duty_u16(translate(angle))

    # print(f"shoulder: {shoulder_angle:.2f}, elbow: {elbow_angle:.2f}")
    print(f"wrist: {translate(angle):.2f}")

