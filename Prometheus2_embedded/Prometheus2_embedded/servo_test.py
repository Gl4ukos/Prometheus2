from machine import Pin, PWM
from time import sleep

#9 - 110 - 200


MAX = 180
MIN= 110
servo1 = PWM(Pin(9))
servo2 = PWM(Pin(8))
servo3 = PWM(Pin(7))
servo4 = PWM(Pin(6))
for s in [servo1, servo2, servo3, servo4]:
    s.freq(50)

servos = [servo1, servo2, servo3, servo4]

def set_servo_angle(angle, servo_index):
    min_duty = 3276
    max_duty = 6553
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servos[servo_index].duty_u16(duty)


# Move the servo back and forth with debug messages
while True:
    break
    for angle in range(MIN+1, MAX, 1):  # Sweep from 0° to 180°
        set_servo_angle(angle,0)
        sleep(0.02)
    for angle in range(MAX, MIN, -1):  # Sweep from 180° back to 0°
        set_servo_angle(angle,0)
        sleep(0.02)

while True:
    #set_servo_angle(MIN,0)
    #set_servo_angle(MIN ,1)
    set_servo_angle(MIN ,2)
    set_servo_angle(MIN,3)

