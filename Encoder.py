 # Class to monitor a rotary encoder and update a value.  You can either read the value when you need it, by calling getValue(), or
# you can configure a callback which will be called whenever the value changes.

import time
import RPi.GPIO as GPIO

class Encoder:

    def __init__(self, leftPin, rightPin, callback=None):
        self.leftPin = leftPin #A
        self.rightPin = rightPin # B
        self.value = 0
        self.state = '00'
        self.direction = None
        self.callback = callback
        self.rising_edges = 0
        self.falling_edges = 0
        GPIO.setup(self.leftPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.rightPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        if GPIO.event_detected(leftPin):
            GPIO.remove_event_detect(leftPin)
        GPIO.add_event_detect(self.leftPin, GPIO.BOTH, callback=self.transitionOccurred)  
        GPIO.add_event_detect(self.rightPin, GPIO.BOTH, callback=self.transitionOccurred)  

        # GPIO.add_event_detect(self.leftPin, GPIO.BOTH, callback=self.enc_A)  
        # GPIO.add_event_detect(self.rightPin, GPIO.BOTH, callback=self.enc_B)  

    # def enc_A(self, channel): # Check edges for Left encoder
    #     if GPIO.input(self.leftPin):
    #         self.rising_edges += 1
    #         print("Encoder A Rising Edge detected")
    #     else:
    #         self.falling_edges += 1
    #         print("Encoder A Falling Edge detected")

    # def enc_B(self, channel): # Check edges for Left encoder
    #     if GPIO.input(self.rightPin):
    #         self.rising_edges += 1
    #         print("Encoder B Rising Edge detected")
    #     else:
    #         self.falling_edges += 1
    #         print("Encoder B Falling Edge detected")

    def transitionOccurred(self, channel):
        p1 = GPIO.input(self.leftPin)
        p2 = GPIO.input(self.rightPin)
        newState = "{}{}".format(p1, p2)

        if GPIO.input(self.rightPin):
            self.rising_edges += 1
            # print("Encoder B Rising Edge detected")
        else:
            self.falling_edges += 1
            # print("Encoder B Falling Edge detected")

        if GPIO.input(self.rightPin):
            self.rising_edges += 1
            # print("Encoder B Rising Edge detected")
        else:
            self.falling_edges += 1
            # print("Encoder B Falling Edge detected")

        if self.state == "00": # Resting position
            if newState == "01": # Turned right 1
                self.direction = "R"
                self.rising_edges += 1
            elif newState == "10": # Turned left 1
                self.direction = "L"
                self.rising_edges += 1
            elif newState == "11": # Turned left 1
                self.rising_edges += 2

        elif self.state == "01": # R1 or L3 position
            if newState == "11": # Turned right 1
                self.direction = "R"
                self.rising_edges += 1
            elif newState == "00": # Turned left 1
                if self.direction == "L":
                    self.falling_edges += 1
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        elif self.state == "10": # R3 or L1
            if newState == "11": # Turned left 1
                self.direction = "L"
                self.rising_edges += 1
            elif newState == "00": # Turned right 1
                if self.direction == "R":
                    self.value = self.value + 1
                    self.falling_edges += 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        else: # self.state == "11"
            if newState == "01": # Turned left 1
                self.direction = "L"
                self.falling_edges += 1
            elif newState == "10": # Turned right 1
                self.direction = "R"
                self.falling_edges += 1
            elif newState == "00": # Skipped an intermediate 01 or 10 state, but if we know direction then a turn is complete
                self.falling_edges += 2
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                elif self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                
        self.state = newState
        

    def getValue(self):
        return self.value
    
    def get_distance(self): 
        return self.value * 4.32 
    
def valueChanged(value, direction):
    print("* New value: {}, Direction: {}".format(value, direction))



GPIO.setmode(GPIO.BCM)

# e1 = Encoder(26, 19)
