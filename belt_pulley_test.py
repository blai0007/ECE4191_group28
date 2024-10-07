import RPi.GPIO as GPIO          
from time import sleep
import pygame
# from Encoder import Encoder
from adafruit_servokit import ServoKit
import board
import busio
from adafruit_pca9685 import PCA9685
import numpy as np
import os

# kit = ServoKit(channels=16)
# Set Pins
in1_left_belt = 17 # 23
in2_left_belt = 27 # 24
# en_left =  11 #25                # Simulating encoder

in1_right_belt = 22
in2_right_belt = 10
# en_right = 13               # simulating encoder

# Initialise Pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1_left_belt,GPIO.OUT)
GPIO.setup(in2_left_belt,GPIO.OUT)

GPIO.setup(in1_right_belt,GPIO.OUT)
GPIO.setup(in2_right_belt,GPIO.OUT)

GPIO.output(in1_left_belt,GPIO.LOW)
GPIO.output(in2_left_belt,GPIO.LOW)
GPIO.output(in1_right_belt,GPIO.LOW)
GPIO.output(in2_right_belt,GPIO.LOW)



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

left_speed = 100
right_speed = 100

try :
    while(True):
        set_motor(in1_left_belt, in2_left_belt, motor_num=2, direction=1, speed=set_speed(left_speed))
        set_motor(in1_right_belt, in2_right_belt, motor_num=3, direction=1, speed=set_speed(right_speed))
        sleep(0.1)

except KeyboardInterrupt : 
    GPIO.cleanup()