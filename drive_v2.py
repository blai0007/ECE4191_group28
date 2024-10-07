# Python Script
# https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-pi/

import RPi.GPIO as GPIO          
from time import sleep
import pygame
import numpy as np
from adafruit_pca9685 import PCA9685
from gpiozero import RotaryEncoder
from PI_Controller import PIController
import board
import busio

pi_controller = PIController(Kp=10, Ki=0.06)
# kit = ServoKit(channels=16)

# Set Pins
in1_left = 5 # 23
in2_left = 6 # 24
# en_left =  11 #25                # Simulating encoder

in1_right = 19
in2_right = 26
# en_right = 13               # simulating encoder

encoder1_left_pin = 7
encoder2_left_pin = 23
encoder1_right_pin = 8
encoder2_right_pin = 24

speed = 1 # throttle speed from 0 to 1
DIRECTION = "S"
# Initialise Pygame Module
pygame.init()
SCREEN_WIDTH =  600
SCREEN_HEIGHT = 400

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

# Initialise Pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1_left,GPIO.OUT)
GPIO.setup(in2_left,GPIO.OUT)
# GPIO.setup(en_left,GPIO.OUT)

GPIO.setup(in1_right,GPIO.OUT)
GPIO.setup(in2_right,GPIO.OUT)
# GPIO.setup(en_right,GPIO.OUT)

GPIO.output(in1_left,GPIO.LOW)
GPIO.output(in2_left,GPIO.LOW)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.LOW)

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # Set PWM frequency for motor control

e1 = RotaryEncoder(encoder1_left_pin, encoder1_right_pin, max_steps = 100000000)
e2 = RotaryEncoder(encoder2_left_pin, encoder2_right_pin, max_steps = 100000000)

# Enable the Motor Drivers
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

def drive_forward():
    m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
    m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

    set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(m1_speed))
    set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=set_speed(m2_speed)) 
    print("forward")

def drive_backwards():
    m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
    m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

    set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=set_speed(m1_speed))
    set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=set_speed(m2_speed))
    print("BACKWARDS")

def drive_left():
    m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
    m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

    set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=set_speed(m1_speed))
    set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=set_speed(m2_speed))
    print("LEFT")

def drive_right():
    m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
    m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

    set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(m1_speed))
    set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=set_speed(m2_speed))
    print("RIGHT")  

def drive_stop():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.LOW) 
    

def update_keyboard():
    for event in pygame.event.get():
        if event.type == pygame.quit : 
            break

        if event.type == pygame.KEYDOWN: 
            if event.key == pygame.K_UP :
                drive_forward()
                print("forward")
                DIRECTION = "F"
                #drive_forward()

            if event.key == pygame.K_DOWN : 
                drive_backwards()
                print("BACKWARDS")
                #drive_backwards()
                DIRECTION="B"

            if event.key == pygame.K_LEFT : 
                drive_left()
                print("LEFT")
                DIRECTION="L"

            if event.key == pygame.K_RIGHT : 
                drive_right()
                print("RIGHT")
                #drive_right()
                DIRECTION="R"

            if event.key == pygame.K_SPACE : 
                drive_stop()
                print("RIGHT")
                DIRECTION = "S"
                #drive_right()

            if event.key == pygame.K_q : 
                print("Quiting")
                GPIO.cleanup()
                break


# key press
# rotate bot
# conrim 360? Y/N
# print ticks per full rotation 


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

while(True):
    update_keyboard()
    print(f"Encoder 1 :{e1.steps}")
    print(f"Encoder 2 :{e2.steps}")

    # Setpoint
    expected_rpm = 75 # EXPECTED SPEED OF MOTOR 0-100
    expected_tick_per_sec = expected_rpm * (900/60)
    dt = 0.1

    ticks_left_prev = e2.steps
    ticks_right_prev = e1.steps
    # Ticks per second
    left_ticks_iter = abs(e1.steps - ticks_left_prev) / dt
    right_ticks_iter = abs(e2.steps - ticks_right_prev) / dt

    # print(f"LEFT Motor Speed : {left_motor_speed}")
    # print(f"RIGHT Motor Speed : {right_motor_speed}")

    # print("#######################################")
    # print(f"Encoder 1 Rising Edge:{e1.rising_edges}")
    # print(f"Encoder 1 Falling Edge:{e1.falling_edges}")

    # print(f"Encoder 2 Rising Edge:{e2.rising_edges}")
    # print(f"Encoder 2 Falling Edge:{e2.falling_edges}")
    # print(f"Encoder 1 :{(e1.rising_edges+e1.falling_edges)/2}")
    # print(f"Encoder 2 :{(e2.rising_edges+e2.falling_edges)/2}")

    sleep(0.1)

