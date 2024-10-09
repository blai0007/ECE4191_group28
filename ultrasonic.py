from gpiozero import DistanceSensor
import RPi.GPIO as GPIO
from time import sleep
import pygame
import os
import numpy as np
import math
from PI_Controller import PIController
import RPi.GPIO as GPIO   
from gpiozero import RotaryEncoder
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio
import PI_Controller

# PWM module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # Set PWM frequency for motor control

# Ultrasonic Pi Pins
echo = 11
trigger = 9

# LEFT MOTOR - WHEELS
in1_left = 5                
in2_left = 6                

# RIGHT MOTOR - WHEELS
in1_right = 19                
in2_right = 26

#encoder pins 
encoder1_left_pin = 25
encoder2_left_pin = 23
encoder1_right_pin = 16
encoder2_right_pin = 24

# input a percentage 0-100 to set speed
def set_speed(percentage_val):
    speed = int(np.floor((percentage_val/100) * 65535))# CircuitPython apparently converts to 16 bit number 
    print(speed)
    return speed

def calc_ticks_per_iter(current, prev, dt):
    abs(current - prev) / dt

def set_motor(in1, in2, motor_num, direction, speed):
    if direction: # forward
        GPIO.output(in1,GPIO.HIGH)  
        GPIO.output(in2,GPIO.LOW)
    else: # back
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
    speed = set_speed(speed)
    pca.channels[motor_num].duty_cycle = speed

# pi_controller = PIController(Kp=0.0003,Ki=0.01,Kd=0) #0.043

#Setup of GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(echo,GPIO.IN)
GPIO.setup(trigger,GPIO.OUT)
GPIO.setup(in1_left,GPIO.OUT)
GPIO.setup(in2_left,GPIO.OUT)
GPIO.setup(in1_right,GPIO.OUT)
GPIO.setup(in2_right,GPIO.OUT)

pi_controller = PIController(Kp=0.0001,Ki=0.01,Kd=0)

e1 = RotaryEncoder(encoder1_left_pin, encoder1_right_pin, max_steps=100000000)
e2 = RotaryEncoder(encoder2_left_pin, encoder2_right_pin, max_steps=100000000) 

ultrasonic = DistanceSensor(echo=echo,trigger=trigger,threshold_distance=0.07)

#Intialisation of Motors - Starting ON GOING BACKWARDS 
GPIO.output(in1_left,GPIO.LOW)              
GPIO.output(in2_left,GPIO.HIGH)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.HIGH)

dt = 1/500
ticks_per_full_rotation = 900                            # TODO : Change this after wheel calibration
degrees_per_tick = 360 / ticks_per_full_rotation     
rps = 1 / 1.37
deg_per_s = rps*360/500
w_expected = 260
ticks_left_prev = 0 
ticks_right_prev = 0

set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=70)
set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=90)

while True: 
    # Motor control logic
    left_ticks_iter = e1.steps - ticks_left_prev
    right_ticks_iter = e2.steps - ticks_right_prev
    w_left = (left_ticks_iter / dt) * degrees_per_tick 
    w_right = (right_ticks_iter / dt) * degrees_per_tick 

    m1_speed = 70
    m2_speed = max(0, min(100, pi_controller.motor_setpoint(w_right, w_left, dt)))

    ticks_left_prev = e1.steps
    ticks_right_prev = e2.steps

    set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=m1_speed)
    set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=m2_speed)

    if ultrasonic.distance < 0.07:
        break

    sleep(dt)  # Sleep for the specified time step
    if ultrasonic.distance < 0.07:
        break

ultrasonic.wait_for_in_range()

print(f'distance before stopping is {ultrasonic.distance}')
before_dis = ultrasonic.distance
#Stop Motors
GPIO.output(in1_left,GPIO.LOW)              
GPIO.output(in2_left,GPIO.LOW)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.LOW)

print(f'distance after stopping is {ultrasonic.distance}')
after_dis = ultrasonic.distance
cal = before_dis-after_dis
print(f'difference in distance is {cal}')


    
