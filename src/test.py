from machine import PWM, Pin, I2C, ADC
from servo_translator import translate

from ads1x15 import ADS1015
from time import sleep

shoulder = PWM(Pin(0), freq=50)
elbow = PWM(Pin(1), freq=50)

i2c = I2C(1, scl=Pin(15), sda=Pin(14))

ads = ADS1015(i2c, address=0x48)

print(ads.read(channel1=0))

x = ADC(Pin(26))
y = ADC(Pin(27))

def calibrate() -> 'tuple[float, float]':
    """
    Calibrate the arm to get the offset angles.
    """
    i2c = I2C(1, scl=Pin(15), sda=Pin(14))
    ads = ADS1015(i2c, address=0x48)

    shoulder_angle = ads.raw_to_v(ads.read(channel1=0)) / 3.3 * 180
    elbow_angle = ads.raw_to_v(ads.read(channel1=1)) / 3.3 * 180

    shoulder.duty_u16(translate(0))
    elbow.duty_u16(translate(180))

    sleep(0.5)

    shoulder_limit = ads.raw_to_v(ads.read(channel1=0)) / 3.3 * 180
    elbow_limit = ads.raw_to_v(ads.read(channel1=1)) / 3.3 * 180

    shoulder_offset = shoulder_limit - shoulder_angle
    elbow_offset = elbow_limit - elbow_angle

    return shoulder_offset, elbow_offset


shoulder_offset, elbow_offset = calibrate()

print(f"shoulder offset: {shoulder_offset:.2f}, elbow offset: {elbow_offset:.2f}")


# while True:
#     shoulder_angle = x.read_u16() / 65535 * 180
#     elbow_angle = y.read_u16() / 65535 * 180

#     shoulder_feedback = ads.raw_to_v(ads.read(channel1=0)) / 3.3 * 180
#     elbow_feedback = ads.raw_to_v(ads.read(channel1=1)) / 3.3 * 180

#     shoulder.duty_u16(translate(shoulder_angle))
#     elbow.duty_u16(translate(elbow_angle))

#     print(f"shoulder: {shoulder_angle:.2f}, elbow: {elbow_angle:.2f}")
#     print(f"shoulder feedback: {shoulder_feedback:.2f}, elbow feedback: {elbow_feedback:.2f}")  


