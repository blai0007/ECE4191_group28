import RPi.GPIO as GPIO          
from time import sleep
import numpy as np
import pygame
import os
from Encoder import Encoder
import board
import busio
from adafruit_pca9685 import PCA9685


# Set Pins
in1_left = 5 # 23
in2_left = 6 # 24
en_left =  11 #25                # Simulating encoder

in1_right = 19
in2_right = 26
en_right = 13               # simulating encoder

encoder1_left_pin = 7
encoder2_left_pin = 23
encoder1_right_pin = 8
encoder2_right_pin = 24

# Initialise Pygame Module
pygame.init()
SCREEN_WIDTH =  600
SCREEN_HEIGHT = 400

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

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

# p_left=GPIO.PWM(en_left,1000)
# p_right=GPIO.PWM(en_right,1000)

e1 = Encoder(encoder1_left_pin, encoder1_right_pin)
e2 = Encoder(encoder2_left_pin, encoder2_right_pin)

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # Set PWM frequency for motor control

def set_motor(in1, in2, motor_num, direction, speed):
    if direction: # forward
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
    else: # back
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)

    pca.channels[motor_num].duty_cycle = speed

# input a percentage 0-100 to set speed
def set_speed(percentage_val):
    speed = int(np.floor((percentage_val/100) * 65535))# CircuitPython apparently converts to 16 bit number 
    print(speed)
    return speed


# Enable the Motor Drivers
# p_left.start(70)
# p_right.start(70)
print("\n")
uInput = input("Start Test?: y/n")

if uInput == 'y':
    # GPIO.output(in1_left,GPIO.HIGH)
    # GPIO.output(in2_left,GPIO.LOW)
    # GPIO.output(in1_right,GPIO.LOW)
    # GPIO.output(in2_right,GPIO.HIGH)
    set_motor(in1_left, in2_left, motor_num=1, direction=1, speed=set_speed(75))
    set_motor(in1_right, in2_right, motor_num=0, direction=0, speed=set_speed(75))
    print("Stop robot using space bar")
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE : 
                GPIO.output(in1_left,GPIO.LOW)
                GPIO.output(in2_left,GPIO.LOW)
                GPIO.output(in1_right,GPIO.LOW)
                GPIO.output(in2_right,GPIO.LOW)

print(f"Encoder 1 Rising Edge:{e1.rising_edges}")
print(f"Encoder 1 Falling Edge:{e1.falling_edges}")
e1_avg = (e1.rising_edges + e1.falling_edges)/2
print(f"e1 Avg = {e1_avg}")

print(f"Encoder 2 Rising Edge:{e2.rising_edges}")
print(f"Encoder 2 Falling Edge:{e2.falling_edges}")
e2_avg = (e2.rising_edges + e2.falling_edges)/2
print(f"e2 Avg = {e2_avg}")

both_enc_avg = (e1_avg + e2_avg) / 2
deg_per_tick = 360/both_enc_avg

dataDir = "{}/".format(os.getcwd())

fileNameS = "{}Rotation_calib.txt".format(dataDir)
np.savetxt(fileNameS, np.array([scale]), delimiter=',')
