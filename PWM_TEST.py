# from adafruit_servokit import ServoKit
from time import sleep
import board
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO
import pygame


# Initialise Pygame Module
pygame.init()
SCREEN_WIDTH =  600
SCREEN_HEIGHT = 400

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

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

# Function used to control individual motors
# in1 and in2: Pin for in1 and in2 (Should be defined at the top)
# motor_num: Select motor number 0-16 (This is the Enable pin (should be connected to PWM module))
# direction (binary): direction of motor (0: Back, 1: Forward)
# speed: Speed of motor
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
    speed = (percentage_val/100) * 65535 # CircuitPython apparently converts to 16 bit number 
    return speed

#Example: Set motor 0 to 75% duty cycle
# set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(75))

#Example: Set motor 2 to 100% duty cycle backwards
# set_motor(in1_left, in2_left, motor_num=2, direction=0, speed=set_speed(100))

def update_keyboard():
    for event in pygame.event.get():
        if event.type == pygame.quit : 
            break

        if event.type == pygame.KEYDOWN: 
            if event.key == pygame.K_UP :
                set_motor(in1_left, in2_left, motor_num=1, direction=1, speed=set_speed(75))
                set_motor(in1_right, in2_right, motor_num=0, direction=1, speed=set_speed(75))
                print("forward")
        if event.type == pygame.KEYDOWN: 
            if event.key == pygame.K_DOWN :
                set_motor(in1_left, in2_left, motor_num=1, direction=0, speed=set_speed(75))
                set_motor(in1_right, in2_right, motor_num=0, direction=0, speed=set_speed(75))
                print("back")
        if event.type == pygame.KEYDOWN: 
            if event.key == pygame.K_LEFT :
                set_motor(in1_left, in2_left, motor_num=1, direction=0, speed=set_speed(75))
                set_motor(in1_right, in2_right, motor_num=0, direction=1, speed=set_speed(75))
                print("left")
        if event.type == pygame.KEYDOWN: 
            if event.key == pygame.K_RIGHT :
                set_motor(in1_left, in2_left, motor_num=1, direction=1, speed=set_speed(75))
                set_motor(in1_right, in2_right, motor_num=0, direction=0, speed=set_speed(75))
                print("right")
        if event.key == pygame.K_q : 
            print("Quiting")
            GPIO.cleanup()
            break
                
while True:
    update_keyboard()
    sleep(0.1)