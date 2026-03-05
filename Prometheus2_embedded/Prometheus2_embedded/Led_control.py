from machine import Pin
import time

OFF_POWER = -1
ON_POWER = 0
JOINT_CONTROL = 1
COORDINATE_CONTROL = 2

class LED_INTERFACE:
    def __init__(self):
        self.switch1 = Pin(3, Pin.IN, Pin.PULL_UP)
        self.switch2 = Pin(4, Pin.IN, Pin.PULL_UP)
        
        self.blink_rates = [100, 200, 400, 600, 800, 1000]
        
        self.red = Pin(16, Pin.OUT)
        self.red.value(0)
        self.red_time = time.ticks_ms()
        self.red_rate = None
        
        self.yellow = Pin(17, Pin.OUT)
        self.yellow.value(0)
        self.yellow_time = time.ticks_ms()
        self.yellow_rate = None
        
        self.yellow2 = Pin(20, Pin.OUT)
        self.yellow2.value(0)
        self.yellow2_time = time.ticks_ms()
        self.yellow2_rate = None
        
        self.white = Pin(18, Pin.OUT)
        self.white.value(0)
        self.white_time = time.ticks_ms()
        self.white_rate = None
        
        self.STATE = ON_POWER
        self.CONTROL = JOINT_CONTROL
        self.INFO = 0
        self.WARNING = 0
        self.ERROR = 0
        
        

    def update_leds(self):
        now = time.ticks_ms()
        
        if not self.red_rate == None:
            if time.ticks_diff(now, self.red_time) >= self.red_rate:
                self.red.toggle()
                self.red_time = now
        if not self.yellow_rate == None:
            if time.ticks_diff(now, self.yellow_time) >= self.yellow_rate:
                self.yellow.toggle()
                self.yellow_time = now
        if not self.yellow2_rate == None:
            if time.ticks_diff(now, self.yellow2_time) >= self.yellow2_rate:
                self.yellow2.toggle()
                self.yellow2_time = now
        if not self.white_rate == None:
            if time.ticks_diff(now, self.white_time) >= self.white_rate:
                self.white.toggle()
                self.white_time = now
        
        if self.STATE == ON_POWER:
            self.yellow2_rate = self.blink_rates[3]
        elif self.STATE == OFF_POWER:
            self.yellow2_rate = None
            self.red_rate = None
            self.yellow_rate = None
            self.white_rate = None
            
        if self.CONTROL == JOINT_CONTROL:
            self.white_rate = self.blink_rates[2]
        elif self.CONTROL == COORDINATE_CONTROL:
            self.white_rate = self.blink_rates[0]
            
            
if __name__ == "__main__":
    ledd = LED_INTERFACE()
    while True:
        ledd.update_leds()
        #time.sleep(1)
        
        