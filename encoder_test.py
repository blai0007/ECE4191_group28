import RPi.GPIO as GPIO          
from time import sleep
from Encoder import Encoder


# Set Pins
in1_left = 5 # 23
in2_left = 6 # 24
# en_left =  11 #25                # Simulating encoder

in1_right = 19
in2_right = 26
# en_right = 13               # simulating encoder

encoder1_left_pin = 7 # 7
encoder2_left_pin = 23
encoder1_right_pin = 8
encoder2_right_pin = 24

left_speed = 75
right_speed = 75
prev_encoder1_value = 0
prev_encoder2_value = 0

# Initialise Pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1_left,GPIO.OUT)
GPIO.setup(in2_left,GPIO.OUT)
GPIO.setup(en_left,GPIO.OUT)

GPIO.setup(in1_right,GPIO.OUT)
GPIO.setup(in2_right,GPIO.OUT)
GPIO.setup(en_right,GPIO.OUT)

GPIO.output(in1_left,GPIO.LOW)
GPIO.output(in2_left,GPIO.LOW)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.LOW)

e1 = Encoder(encoder1_left_pin, encoder1_right_pin)
e2 = Encoder(encoder2_left_pin, encoder2_right_pin)