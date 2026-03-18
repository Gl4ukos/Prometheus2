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
servos = [servo(servo_arm_config), servo(servo_forearm_config), servo(servo_wrist_config), servo(servo_base_config), servo(servo_gripper_config)]
# === LEDS SETUP
board_led = Pin(25,Pin.OUT)
# === SWITCH SETUP
switch1 = Pin(17, Pin.IN, Pin.PULL_UP)
switch2 = Pin(16, Pin.IN, Pin.PULL_UP)
base_servo_val = 0.45 #stays still in this position
base_sevo_step = 0.00

# === GRIPPER SETUP
gripper_activate_button = Pin(20, Pin.IN, Pin.PULL_UP)


# === GLOBAL VARS
print_index = 0
JOINT_CTR = 1
POS_CTR = 2
CONTROL = JOINT_CTR

ee_pose = [0.0, 0.0, 0.0]
duty_values = [0.0, 0.0, 0.0]

# === MAIN LOOP
while True:    
    # check potentiometer values
    read_voltage()
        
    # update body servos
    for i in range(len(pots)):
        duty_values[i] = servos[i].update_servo_from_pot(pots[i])
        if(print_index%10==0):
#             board_led.toggle()
            print(duty_values, switch1.value(), switch2.value(), gripper_activate_button.value())
                
    # update base servo
    if switch1.value() == 1:
        if switch2.value() == 1:
            base_servo_val = 0.45
        else:
            base_servo_val = 0.55
    elif switch2.value() == 1:
        base_servo_val = 0.35
    elif switch1.value() ==0 and switch2.value() ==0:
        base_servo_val = 0.45
            
    servos[3].update_servo_from_pot(base_servo_val)
    
    
    # update gripper servo
    if gripper_activate_button.value()==0:
        servos[4].update_servo_from_pot(1)
    else:
        servos[4].update_servo_from_pot(0)
    
    # info output
    print_index += 1
    time.sleep(0.01)
