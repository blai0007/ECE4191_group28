import RPi.GPIO as GPIO    
from time import sleep


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
        GPIO.add_event_detect(self.leftPin, GPIO.BOTH, callback=self.transitionOccurred)  
        GPIO.add_event_detect(self.rightPin, GPIO.BOTH, callback=self.transitionOccurred)  
   

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

GPIO.setmode(GPIO.BCM)

encoder1_left_pin = 7 # 7
encoder2_left_pin = 23
encoder1_right_pin = 8
encoder2_right_pin = 24

# Initialise Pins
GPIO.setmode(GPIO.BCM)

# e1 = Encoder(encoder1_left_pin, encoder1_right_pin)
GPIO.cleanup()

# GPIO.add_event_detect(encoder1_left_pin, GPIO.BOTH, callback=self.transitionOccurred)  
# e1 = Encoder(encoder1_left_pin, encoder1_right_pin)
# # e2 = Encoder(encoder2_left_pin, encoder2_right_pin)

# import lgpio
# import RPi.GPIO as GPIO
# from time import sleep
# from Encoder import Encoder

# # Initialize GPIO
# h = lgpio.gpiochip_open(0)

# # Set Pins
# in1_left = 5 
# in2_left = 6 

# in1_right = 19
# in2_right = 26

# encoder1_left_pin = 7 
# encoder2_left_pin = 23
# encoder1_right_pin = 8
# encoder2_right_pin = 24

# left_speed = 75
# right_speed = 75

# # Set GPIO pins as output
# lgpio.gpio_claim_output(h, in1_left)
# lgpio.gpio_claim_output(h, in2_left)
# lgpio.gpio_claim_output(h, in1_right)
# lgpio.gpio_claim_output(h, in2_right)

# # Reset all pins to LOW
# lgpio.gpio_write(h, in1_left, 0)
# lgpio.gpio_write(h, in2_left, 0)
# lgpio.gpio_write(h, in1_right, 0)
# lgpio.gpio_write(h, in2_right, 0)

# # Initialize encoders
# e1 = Encoder(encoder1_left_pin, encoder1_right_pin)

# # Don't forget to close the GPIO at the end of your program
# lgpio.gpiochip_close(h)