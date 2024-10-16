# Python Script
# https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-pi/

import RPi.GPIO as GPIO          
from time import sleep
import pygame
from Encoder import Encoder
from adafruit_servokit import ServoKit
import board
import busio
from adafruit_pca9685 import PCA9685
import numpy as np
from yolo import YOLODetector
import time
import cv2

# kit = ServoKit(channels=16)
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

left_speed = 100
right_speed = 100
prev_encoder1_value = 0
prev_encoder2_value = 0
DIRECTION = 0

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

# p_left=GPIO.PWM(en_left,10)
# p_right=GPIO.PWM(en_right,10)

e1 = Encoder(encoder1_left_pin, encoder1_right_pin)
e2 = Encoder(encoder2_left_pin, encoder2_right_pin)


# Enable the Motor Drivers
# p_left.start(100)
# p_right.start(100)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

def drive_stop():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.LOW) 
    
def update_keyboard(DIRECTION):
    for event in pygame.event.get():
        if event.type == pygame.quit : 
            break

        if event.type == pygame.KEYDOWN: 
            if event.key == pygame.K_UP :
                DIRECTION = 1
                
                print("forward") 
            if event.key == pygame.K_DOWN :
                DIRECTION = 2
                
                print("back")
            if event.key == pygame.K_LEFT :
                DIRECTION = 3
                set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=set_speed(left_speed))
                set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=set_speed(right_speed))
                print("left")
            if event.key == pygame.K_RIGHT :
                DIRECTION = 4
                set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(left_speed))
                set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=set_speed(right_speed))
                print("right")

            if event.key == pygame.K_s :
                drive_stop()

            if event.key == pygame.K_q : 
                print("Quiting")
                GPIO.cleanup()
                break
            return DIRECTION

def change_speed(e1, e2, left_speed, right_speed):
    left_ticks_iter = abs(e1.getValue()-prev_encoder1_value)
    right_ticks_iter = abs(e2.getValue()-prev_encoder2_value)

    # if left_ticks_iter > right_ticks_iter:
        # left_speed = 0
        # left_speed -= 0.5
        # right_speed += 0.5
        # print(f"This iteration, LEFT ticks are more by {left_ticks_iter-right_ticks_iter}")

    # elif left_ticks_iter < right_ticks_iter:
        # right_speed = 0
        # right_speed -= 0.5
        # left_speed += 0.5
        # print(f"This iteration, RIGHT ticks are more by {-left_ticks_iter+right_ticks_iter}")
    if left_speed >= 100:
        left_speed = 100
    elif right_speed >= 100:
        right_speed = 100

    return left_speed, right_speed

# key press
# rotate bot
# conrim 360? Y/N
# print ticks per full rotation 

# YOLO STUFF
path = 'yolo_test.pt'
vs = cv2.VideoCapture(0)
yolo = YOLODetector(path)
time.sleep(0.2)

while(True):
    _,frame = vs.read()
    yolo.find_ball(frame)
    cv2.imshow('Frame',frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break

    DIRECTION = update_keyboard(DIRECTION)
    print("#############################################")
    print(f"Encoder 1 :{e1.getValue()}")
    print(f"Encoder 2 :{e2.getValue()}")
    print(f"Encoder 1 (R+F):{(e1.rising_edges+e1.falling_edges)/2}")
    print(f"Encoder 2 (R+F):{(e2.rising_edges+e2.falling_edges)/2}")


    left_speed, right_speed = change_speed(e1,e2, left_speed, right_speed)

    if DIRECTION == 1:
        set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(left_speed))
        set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=set_speed(right_speed))
    elif DIRECTION == 2:
        set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=set_speed(left_speed))
        set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=set_speed(right_speed))
    # elif DIRECTION == 3:

    prev_encoder1_value = e1.getValue()
    prev_encoder2_value = e2.getValue()

    print("\n")
    print(f"LEFT_SPEED : {left_speed}")
    print(f"RIGHT_SPEED : {right_speed}")
    # print("#######################################")
    # print(f"Encoder 1 Rising Edge:{e1.rising_edges}")
    # print(f"Encoder 1 Falling Edge:{e1.falling_edges}")

    sleep(0.1)

