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

#Setup of GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(echo,GPIO.IN)
GPIO.setup(trigger,GPIO.OUT)
GPIO.setup(in1_left,GPIO.OUT)
GPIO.setup(in2_left,GPIO.OUT)
GPIO.setup(in1_right,GPIO.OUT)
GPIO.setup(in2_right,GPIO.OUT)

ultrasonic = DistanceSensor(echo=echo,trigger=trigger,threshold_distance=0.3) 

#Intialisation of Motors - Starting ON GOING BACKWARDS 
GPIO.output(in1_left,GPIO.LOW)              
GPIO.output(in2_left,GPIO.HIGH)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.HIGH)

pca.channels[0].duty_cycle = 0.9
pca.channels[1].duty_cycle = 0.7

ultrasonic.wait_for_in_range()

#Stop Motors
GPIO.output(in1_left,GPIO.LOW)              
GPIO.output(in2_left,GPIO.LOW)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.LOW)


    
