import RPi.GPIO as GPIO          
from time import sleep
import pygame
from Encoder import Encoder
from adafruit_servokit import ServoKit
import board
import busio
from adafruit_pca9685 import PCA9685
import numpy as np
from PI_Controller import PIController
from gpiozero import RotaryEncoder, Button
import os

# Set Pins
in1_left_belt = 17 # 23
in2_left_belt = 27 # 24

in1_right_belt = 22
in2_right_belt = 10
# Set Pins
in1_left = 5 # 23
in2_left = 6 # 24
# en_left =  11 #25                # Simulating encoder

in1_right = 19
in2_right = 26
# en_right = 13               # simulating encoder

encoder1_left_pin = 25
encoder2_left_pin = 23
encoder1_right_pin = 16
encoder2_right_pin = 24

controller_vals = []
# Setpoint
expected_rpm = 75 # EXPECTED SPEED OF MOTOR 0-100
expected_tick_per_sec = expected_rpm * (900/60)
dt = 0.1


# Initialise Pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1_left,GPIO.OUT)
GPIO.setup(in2_left,GPIO.OUT)
# GPIO.setup(en_left,GPIO.OUT)

GPIO.setup(in1_right,GPIO.OUT)
GPIO.setup(in2_right,GPIO.OUT)
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

# kit = ServoKit(channels=16)

GPIO.output(in1_left,GPIO.LOW)
GPIO.output(in2_left,GPIO.LOW)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.LOW)

pi_controller = PIController(Kp=10, Ki=0.06)

left_belt_speed = 100
right_belt_speed = 100

e1 = RotaryEncoder(encoder1_left_pin, encoder1_right_pin, max_steps = 100000000)
e2 = RotaryEncoder(encoder2_left_pin, encoder2_right_pin, max_steps = 100000000)

ticks_left = e1.steps
ticks_right = e2.steps
ticks_left_prev = 0
ticks_right_prev = 0

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


try :
    while(True):
        left_ticks_iter = abs(ticks_left-ticks_left_prev) / dt
        right_ticks_iter = abs(ticks_right-ticks_right_prev) / dt
        m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
        m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

        set_motor(in1_left, in2_left, motor_num=1, direction=1, speed=set_speed(m1_speed))
        set_motor(in1_right, in2_right, motor_num=0, direction=1, speed=set_speed(m2_speed))
        set_motor(in1_left_belt, in2_left_belt, motor_num=2, direction=1, speed=set_speed(left_belt_speed))
        set_motor(in1_right_belt, in2_right_belt, motor_num=3, direction=1, speed=set_speed(right_belt_speed))
        sleep(0.01)

except KeyboardInterrupt : 
    GPIO.cleanup()