# Python Script
# https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-pi/

import RPi.GPIO as GPIO          
from time import sleep
import pygame
from Encoder import Encoder
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

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
GPIO.setup(en_left,GPIO.OUT)

GPIO.setup(in1_right,GPIO.OUT)
GPIO.setup(in2_right,GPIO.OUT)
GPIO.setup(en_right,GPIO.OUT)

GPIO.output(in1_left,GPIO.LOW)
GPIO.output(in2_left,GPIO.LOW)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.LOW)

p_left=GPIO.PWM(en_left,10)
p_right=GPIO.PWM(en_right,10)

e1 = Encoder(encoder1_left_pin, encoder1_right_pin)
e2 = Encoder(encoder2_left_pin, encoder2_right_pin)

left_motor_speed = 100
right_motor_speed = 100

prev_encoder1_value = 0
prev_encoder2_value = 0

# Enable the Motor Drivers
p_left.start(left_motor_speed)
p_right.start(right_motor_speed)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

def drive_forward():
    GPIO.output(in1_left,GPIO.HIGH)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.HIGH)
    GPIO.output(in2_right,GPIO.LOW)
    # kit.continuous_servo[0].throttle = speed
    # kit.continuous_servo[1].throttle = speed
    print("forward")

def drive_backwards():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.HIGH)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.HIGH)
    # kit.continuous_servo[0].throttle = -speed
    # kit.continuous_servo[1].throttle = -speed
    print("BACKWARDS")

def drive_left():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.HIGH)
    GPIO.output(in1_right,GPIO.HIGH)
    GPIO.output(in2_right,GPIO.LOW)
    print("LEFT")

def drive_right():
    GPIO.output(in1_left,GPIO.HIGH)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.HIGH)
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
                GPIO.output(in1_left,GPIO.HIGH)
                GPIO.output(in2_left,GPIO.LOW)
                GPIO.output(in1_right,GPIO.HIGH)
                GPIO.output(in2_right,GPIO.LOW)
                print("forward")
                DIRECTION = "F"
                #drive_forward()

            if event.key == pygame.K_DOWN : 
                GPIO.output(in1_left,GPIO.LOW)
                GPIO.output(in2_left,GPIO.HIGH)
                GPIO.output(in1_right,GPIO.LOW)
                GPIO.output(in2_right,GPIO.HIGH)
                print("BACKWARDS")
                #drive_backwards()
                DIRECTION="B"

            if event.key == pygame.K_LEFT : 
                GPIO.output(in1_left,GPIO.LOW)
                GPIO.output(in2_left,GPIO.HIGH)
                GPIO.output(in1_right,GPIO.HIGH)
                GPIO.output(in2_right,GPIO.LOW)
                print("LEFT")
                DIRECTION="L"

            if event.key == pygame.K_RIGHT : 
                GPIO.output(in1_left,GPIO.HIGH)
                GPIO.output(in2_left,GPIO.LOW)
                GPIO.output(in1_right,GPIO.LOW)
                GPIO.output(in2_right,GPIO.HIGH)
                print("RIGHT")
                #drive_right()
                DIRECTION="R"

            if event.key == pygame.K_SPACE : 
                GPIO.output(in1_left,GPIO.LOW)
                GPIO.output(in2_left,GPIO.LOW)
                GPIO.output(in1_right,GPIO.LOW)
                GPIO.output(in2_right,GPIO.LOW)
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


def change_speed(e1, e2):
    if abs(e1.getValue()-prev_encoder1_value) > abs(e2.getValue()-prev_encoder2_value):
        left_motor_speed -= 1
        p_left.ChangeDutyCycle(left_motor_speed)

    elif abs(e1.getValue()-prev_encoder1_value) < abs(e2.getValue()-prev_encoder2_value):
        right_motor_speed -= 1
        p_right.ChangeDutyCycle(right_motor_speed)


while(True):
    update_keyboard()
    print(f"Encoder 1 :{e1.getValue()}")
    print(f"Encoder 2 :{e2.getValue()}")

    change_speed(e1,e2)

    prev_encoder1_value = e1.getValue()
    prev_encoder2_value = e2.getValue()

    print(f"LEFT Motor Speed : {left_motor_speed}")
    print(f"RIGHT Motor Speed : {right_motor_speed}")

    # print("#######################################")
    # print(f"Encoder 1 Rising Edge:{e1.rising_edges}")
    # print(f"Encoder 1 Falling Edge:{e1.falling_edges}")

    # print(f"Encoder 2 Rising Edge:{e2.rising_edges}")
    # print(f"Encoder 2 Falling Edge:{e2.falling_edges}")
    # print(f"Encoder 1 :{(e1.rising_edges+e1.falling_edges)/2}")
    # print(f"Encoder 2 :{(e2.rising_edges+e2.falling_edges)/2}")

    sleep(0.1)

