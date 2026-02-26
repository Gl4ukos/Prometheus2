from machine import Pin, ADC, PWM
import time

servo_arm_config = {
        "pin": 6,
        "max_d": 8400,
        "min_d": 3000,
        "home_d": 6500,
        "start_d": 8400
    }

servo_forearm_config = {
        "pin": 7,
        "max_d": 8000, 
        "min_d": 2000,
        "home_d": 6000,
        "start_d": 8000
    }

servo_wrist_config = {
        "pin": 9,
        "max_d": 8000, 
        "min_d": 5500,
        "home_d": 8000,
        "start_d": 8000
    }

# SERVOS DEFINITION
class servo:
    def __init__(self, servo_config_dict):
        self.pin_out = PWM(Pin(servo_config_dict["pin"]))
        self.pin_out.freq(50)
        self.max_duty = servo_config_dict["max_d"]
        self.min_duty = servo_config_dict["min_d"]
        self.home_duty = servo_config_dict["home_d"]
        self.start_duty = servo_config_dict["start_d"]
        self.curr_duty = self.start_duty
        self.pin_out.duty_u16(self.start_duty)
    
    # ! potentiometer value between 0 - 1 !
    def update_servo_from_pot(self, potentiometer_value):
        self.curr_duty = int(self.min_duty + potentiometer_value * (self.max_duty - self.min_duty))
        self.pin_out.duty_u16(self.curr_duty)
    
    def update_servo_from_pot_gradual(self, potentiometer_value):
        target_duty = int(self.min_duty + potentiometer_value * (self.max_duty - self.min_duty))
        print(self.curr_duty, target_duty)
        step = 50 if target_duty > self.curr_duty else -50
        for duty in range(self.curr_duty, target_duty, step):
            self.pin_out.duty_u16(duty)
            time.sleep(0.05)
        self.pin_out.duty_u16(target_duty)
        self.curr_duty = target_duty
    
    def set_duty(self, value):
        step = 50 if target_duty > self.curr_duty else -50
        for duty in range(self.curr_duty, target_duty, step):
            self.pin_out.duty_u16(duty)
            time.sleep(0.05)
        self.pin_out.duty_u16(target_duty)
        self.curr_duty = target_duty
        
    def home_pose(self):
        self.set_duty(self.home_duty)
            
    def start_pose(self):
        self.set_duty(self.start_duty)

