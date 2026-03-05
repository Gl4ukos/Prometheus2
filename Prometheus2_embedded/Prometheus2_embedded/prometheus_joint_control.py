from machine import Pin, ADC, PWM
import time
from servo_config import *
from Led_control import *
        
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
servos = [servo(servo_arm_config), servo(servo_forearm_config), servo(servo_wrist_config), servo(servo_base_config)]
# === LEDS SETUP
leds = LED_INTERFACE()
# === SWITCH SETUP
switch1 = Pin(4, Pin.IN, Pin.PULL_UP)
switch2 = Pin(3, Pin.IN, Pin.PULL_UP)
base_servo_val = 0.45 #stays still in this position
base_sevo_step = 0.00
# === MAIN LOOP
print_index = 0
    
while True:
    read_voltage()
    for i in range(len(pots)):
        servos[i].update_servo_from_pot(pots[i])
        
    if switch1.value() == 1:
        base_servo_val = 0.55
    elif switch2.value() == 1:
        base_servo_val = 0.35
    elif switch1.value() ==0 and switch2.value() ==0:
        base_servo_val = 0.45
        
    servos[3].update_servo_from_pot(base_servo_val)
    leds.update_leds()
    if(print_index%10==0):
        print(pots, base_servo_val )
    print_index += 1
    time.sleep(0.01)