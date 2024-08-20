# Python Script
# https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-pi/

import RPi.GPIO as GPIO          
from time import sleep
import pygame

# Set Pins
in1_left = 23
in2_left = 24
en_left = 25

in1_right = 19
in2_right = 26
en_right = 13


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

p_left=GPIO.PWM(en_left,1000)
p_right=GPIO.PWM(en_right,1000)

# Enable the Motor Drivers
p_left.start(25)
p_right.start(25)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

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

            if event.key == pygame.K_DOWN : 
                GPIO.output(in1_left,GPIO.LOW)
                GPIO.output(in2_left,GPIO.HIGH)
                GPIO.output(in1_right,GPIO.LOW)
                GPIO.output(in2_right,GPIO.HIGH)
                print("BACKWARDS")

            if event.key == pygame.K_LEFT : 
                print("LEFT")

            if event.key == pygame.K_RIGHT : 
                print("RIGHT")

            if event.key == pygame.K_q : 
                print("Quiting")
                GPIO.cleanup()
                break

while(True):
    update_keyboard()
    sleep(0.1)

