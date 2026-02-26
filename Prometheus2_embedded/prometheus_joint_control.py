from machine import Pin, ADC, PWM
import time
from servo_config import *

        
# === ADC INPUT FOR SERVO CONTROl
adc_pins = [ADC(26), ADC(27), ADC(28)]
pots = [0,0,0]

def read_voltage():
    global adc_pins
    global pots
    
    DEADBAND = 0.01
    for i in range(len(adc_pins)):
        raw = adc_pins[i].read_u16()
        voltage_normalized = raw / 65535
        if abs(voltage_normalized - pots[i]) > DEADBAND:
            pots[i] = 0.3*voltage_normalized + 0.7*pots[i]
            
read_voltage()
time.sleep(1)
read_voltage()
# === SERVO SETUP
servos = [servo(servo_arm_config), servo(servo_forearm_config), servo(servo_wrist_config)]

# === MAIN LOOP
print_index = 0
for i in range(len(pots)):
    servos[i].update_servo_from_pot_gradual(pots[i])
    
while True:
    read_voltage()
    for i in range(len(pots)):
        servos[i].update_servo_from_pot(pots[i])
    if(print_index%10==0):
        print(pots)
    print_index += 1
    time.sleep(0.01)