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
import os
from PI_Controller import PIController

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

left_speed = 75
right_speed = 75
prev_encoder1_value = 0
prev_encoder2_value = 0
DIRECTION = 0

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # Set PWM frequency for motor control


pygame.init()
SCREEN_WIDTH =  900
SCREEN_HEIGHT = 500

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

# Initiate Pygame Pictures
WIN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("LOCALISATION")

WHITE = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'White.png')), (SCREEN_WIDTH, SCREEN_HEIGHT))

BLUE = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'Blue.png')), (548, 411))

ORIGIN = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'Origin.png')), (10, 10))

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
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

# Robot Class
class robot : 
    def __init__(self) : 

        # PYGAME COORDINATES
        self.x_pygame = 100              
        self.y_pygame = 438          
        self.starting_x_pygame = 100       
        self.starting_y_pygame = 438      
        self.x_cartesian = self.x_pygame - self.starting_x_pygame
        self.y_cartesian = -(self.y_pygame - self.starting_y_pygame)
        self.deg = 0

        # LOCALISATION PARAMETERS
        self.m_per_tick = 30/720 *3 #(1000 / 10400) / 10        #cm                    # Nathan and Bryan checked this, measure again if unsure
        self.ticks_per_full_rotation = 3659 # revs per 360*ticks per rev #3600 #1800 # 7500 #700  
        self.degrees_per_tick = 360 / self.ticks_per_full_rotation      

        # LOCALISATION (ENCODERS)
        self.ticks_left = 0
        self.ticks_right = 0
        self.ticks_left_prev = 0
        self.ticks_right_prev = 0
        self.left_mag = 0
        self.right_mag = 0
        self.left_a = 0
        self.left_b = 0

        # VISUALISATION
        self.width = 19
        self.height = 23
        self.image = pygame.image.load(os.path.join('PNGs', 'spaceship_red.png'))
        self.blit = pygame.transform.rotate(pygame.transform.scale(self.image, (self.width, self.height)), 180)
        self.rect = pygame.Rect(700, 300, self.width, self.height)

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
                set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(left_speed))
                set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=set_speed(right_speed))
                DIRECTION = 1
                
                print("forward") 
            if event.key == pygame.K_DOWN :
                set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=set_speed(left_speed))
                set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=set_speed(right_speed))
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

            if event.key == pygame.K_SPACE :
                drive_stop()
                DIRECTION = 0
                print("STOP") 

            if event.key == pygame.K_q : 
                print("Quiting")
                GPIO.cleanup()
                break
    return DIRECTION
        
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

def change_speed(e1, e2, left_speed, right_speed):
    left_ticks_iter = abs(e1.getValue()-prev_encoder1_value)
    right_ticks_iter = abs(e2.getValue()-prev_encoder2_value)

    if left_ticks_iter > right_ticks_iter:
        # left_speed = 0
        left_speed -= 5
        right_speed += 5
        print(f"This iteration, LEFT ticks are more by {left_ticks_iter-right_ticks_iter}")

    elif left_ticks_iter < right_ticks_iter:
        # right_speed = 0
        right_speed -= 5
        left_speed += 5
        print(f"This iteration, RIGHT ticks are more by {-left_ticks_iter+right_ticks_iter}")

    if left_speed >= 100:
        left_speed = 100
    elif right_speed >= 100:
        right_speed = 100

    return left_speed, right_speed

def localisation(robot, e1_value, e2_value, e1, e2) : 
    distance_moved = 0
    degrees_turned = 0
    robot.ticks_left = e1_value
    robot.ticks_right = e2_value

    left_mag = (e1.rising_edges+e1.falling_edges)/2 #(e1.rising_edges+e1.falling_edges)/2
    robot.left_mag += left_mag
    robot.left_a += e1.rising_edges
    robot.left_b += e1.falling_edges
    
    print(f"left magnitude={left_mag}")
    
    right_mag = (e2.rising_edges+e2.falling_edges)/2

    print(f"right magnitude={right_mag}") 
    robot.right_mag += right_mag

    left_ticks_iter = abs(robot.ticks_left-robot.ticks_left_prev)
    right_ticks_iter = abs(robot.ticks_right-robot.ticks_right_prev)

    # MOVE FORWARDS
    if (robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its Forwards")
        if (robot.ticks_left-robot.ticks_left_prev) < (robot.ticks_right - robot.ticks_right_prev) : 
            print("Titling Leftwards")
            R = (right_ticks_iter*robot.width) / (-left_ticks_iter+right_ticks_iter)
            v = right_ticks_iter / 0.1
            w = v/R
            new_robot_deg = robot.deg + w*0.1
            robot.y_pygame -= (np.cos(np.deg2rad(robot.deg)) - np.cos(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)
            robot.x_pygame += (np.sin(np.deg2rad(robot.deg)) - np.sin(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)
            robot.deg = new_robot_deg

        if (robot.ticks_left-robot.ticks_left_prev) > (robot.ticks_right - robot.ticks_right_prev) : 
            print("Titling Rightwards")
            R = (left_ticks_iter*robot.width) / (-left_ticks_iter+right_ticks_iter)
            v = left_ticks_iter / 0.1
            w = v/R
            new_robot_deg = robot.deg + w*0.1
            robot.y_pygame -= (np.cos(np.deg2rad(robot.deg)) - np.cos(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)
            robot.x_pygame += (np.sin(np.deg2rad(robot.deg)) - np.sin(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)
            robot.deg = new_robot_deg

        else : 
            R = (left_ticks_iter+right_ticks_iter) / 2
            # v = (left_ticks_iter+right_ticks_iter)/0.1
            robot.y_pygame -= np.cos(np.deg2rad(robot.deg)) * (R * robot.m_per_tick)
            robot.x_pygame += np.sin(np.deg2rad(robot.deg)) * (R * robot.m_per_tick)
            
        
    # MOVE BACKWARDS
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        print("moving_backwards")
        if (robot.ticks_left-robot.ticks_left_prev) < (robot.ticks_right - robot.ticks_right_prev) : 
            print("Titling Leftwards")
            R = (right_ticks_iter*robot.width) / (-left_ticks_iter+right_ticks_iter)
            v = right_ticks_iter / 0.1
            w = v/R
            new_robot_deg = robot.deg - w*0.1
            robot.y_pygame -= -(np.cos(np.deg2rad(robot.deg)) - np.cos(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)
            robot.x_pygame += -(np.sin(np.deg2rad(robot.deg)) - np.sin(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)
            robot.deg = new_robot_deg

        if (robot.ticks_left-robot.ticks_left_prev) > (robot.ticks_right - robot.ticks_right_prev) : 
            print("Titling Rightwards")
            R = (left_ticks_iter*robot.width) / (left_ticks_iter-right_ticks_iter)
            v = left_ticks_iter / 0.1
            w = v/R
            new_robot_deg = robot.deg - w*0.1
            robot.y_pygame -= -(np.cos(np.deg2rad(robot.deg)) - np.cos(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)
            robot.x_pygame += -(np.sin(np.deg2rad(robot.deg)) - np.sin(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)
            robot.deg = new_robot_deg

        else : 
            R = (left_ticks_iter+right_ticks_iter) / 2
            # v = (left_ticks_iter+right_ticks_iter)/0.1
            robot.y_pygame -= -np.cos(np.deg2rad(robot.deg)) * (R * robot.m_per_tick)
            robot.x_pygame += -np.sin(np.deg2rad(robot.deg)) * (R * robot.m_per_tick)
            

    # MOVE LEFT
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its MOVING LEFT")
        degrees_turned = -(left_mag) * robot.degrees_per_tick
        print(f"Deg turned : {degrees_turned}")
        #deg_turned = rotation_calib 

    # MOVE RIGHT
    if ( robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        degrees_turned = (left_mag) * robot.degrees_per_tick
        print(f"Deg turned : {degrees_turned}")
        print("Its MOVING RIGHT")

    # robot.y_pygame -= np.cos(np.deg2rad(robot.deg)) * distance_moved
    # robot.x_pygame += np.sin(np.deg2rad(robot.deg)) * distance_moved
    # robot.deg += degrees_turned

    print(f"Moving in x :  {np.sin(np.deg2rad(degrees_turned)) * distance_moved}")
    print(f"Distance Moved : {distance_moved}cm")

    if robot.deg < 0 : 
        robot.deg = 360 + robot.deg

    elif robot.deg > 360 :
        robot.deg = robot.deg - 360

    robot.ticks_left_prev = robot.ticks_left
    robot.ticks_right_prev = robot.ticks_right

    print(f"ROBOT LEFT_TICK : {robot.ticks_left_prev}")
    print(f"ROBOT Right_TICK : {robot.ticks_right_prev}")

    # print(np.sin(degrees_turned) * distance_moved)

    e1.rising_edges = 0
    e2.rising_edges =0
    e1.falling_edges = 0
    e2.falling_edges = 0
    return

def draw_window(robot, left_speed, right_speed, e1_value, e2_value):
    WIN.blit(WHITE, (0, 0))
    WIN.blit(BLUE, (100,50))
    WIN.blit(ORIGIN, (robot.starting_x_pygame+(robot.width/2), robot.starting_y_pygame+(robot.height/2)))
    robot.blit = pygame.transform.rotate(pygame.transform.scale(robot.image, (robot.width, robot.height)), -robot.deg+180)
    WIN.blit(robot.blit, (robot.x_pygame, robot.y_pygame))

    robot.x_cartesian = robot.x_pygame - robot.starting_x_pygame
    robot.y_cartesian = -(robot.y_pygame - robot.starting_y_pygame)

    # FONTS / TEXTS
    pygame.font.init()
    my_font = pygame.font.SysFont('Comic Sans MS', 30)
    location_title_txt = my_font.render("Position: ", False, (0, 0, 0))
    location_txt = my_font.render(f'({np.round((robot.x_cartesian),2)},{np.round(robot.y_cartesian,2)})', False, (0, 0, 0))
    WIN.blit(location_title_txt, (658,0))
    WIN.blit(location_txt, (678,40))

    degrees_title_txt = my_font.render("Degrees : ", False, (0, 0, 0))
    degrees_txt = my_font.render(f'{np.round(robot.deg,2)}', False, (0, 0, 0))
    WIN.blit(degrees_title_txt, (658,80))
    WIN.blit(degrees_txt, (678,120))

    left_ticks_title_txt = my_font.render("LEFT TICK (NORMAL) :", False, (0, 0, 0))
    left_ticks_txt = my_font.render(f'{np.round(e1_value)}', False, (0, 0, 0))
    WIN.blit(left_ticks_title_txt, (658,160))
    WIN.blit(left_ticks_txt, (678,200))

    right_ticks_title_txt = my_font.render("Right TICK (NORMAL) :", False, (0, 0, 0))
    right_ticks_txt = my_font.render(f'{np.round(e2_value)}', False, (0, 0, 0))
    WIN.blit(right_ticks_title_txt, (658,240))
    WIN.blit(right_ticks_txt, (678,280))

    left_speed_title_txt = my_font.render("LEFT SPEED : ", False, (0, 0, 0))
    left_speed_txt = my_font.render(f'{left_speed}', False, (0, 0, 0))
    WIN.blit(left_speed_title_txt, (658,320))
    WIN.blit(left_speed_txt, (678,360))

    right_speed_title_txt = my_font.render("RIGHT SPEED : ", False, (0, 0, 0))
    right_speed_txt = my_font.render(f'{right_speed}', False, (0, 0, 0))
    WIN.blit(right_speed_title_txt, (658,400))
    WIN.blit(right_speed_txt, (678,460))

    pygame.display.update()

# key press
# rotate bot
# conrim 360? Y/N
# print ticks per full rotation 
Robot = robot()

while(True):
    DIRECTION = update_keyboard(DIRECTION)
    print("#############################################")
    print(f"Encoder 1 :{e1.getValue()}")
    print(f"Encoder 2 :{e2.getValue()}")
    print(f"Encoder 1 (R+F):{(e1.rising_edges+e1.falling_edges)/2}")
    print(f"Encoder 2 (R+F):{(e2.rising_edges+e2.falling_edges)/2}")


    left_speed, right_speed = change_speed(e1,e2, left_speed, right_speed)

    # if DIRECTION == 1:
    #     set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(left_speed))
    #     set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=set_speed(right_speed))
    # elif DIRECTION == 2:
    #     set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=set_speed(left_speed))
    #     set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=set_speed(right_speed))
    # elif DIRECTION == 3:

    prev_encoder1_value = e1.getValue()
    prev_encoder2_value = e2.getValue()

    print("\n")
    print(f"LEFT_SPEED : {left_speed}")
    print(f"RIGHT_SPEED : {right_speed}")
    print(f"Dirction : {DIRECTION}")
    # print("#######################################")
    # print(f"Encoder 1 Rising Edge:{e1.rising_edges}")
    # print(f"Encoder 1 Falling Edge:{e1.falling_edges}")

    # Setpoint
    expected_rpm = 75 # EXPECTED SPEED OF MOTOR 0-100
    expected_tick_per_sec = expected_rpm * (900/60)
    dt = 0.1

    # Ticks per second
    left_ticks_iter = abs(robot.ticks_left-robot.ticks_left_prev) / dt
    right_ticks_iter = abs(robot.ticks_right-robot.ticks_right_prev) / dt

    pi_controller = PIController(kp=0.5, ki=0.01)

    if DIRECTION == 1 :
        m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
        m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

        set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(m1_speed))
        set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=set_speed(m2_speed))

    elif DIRECTION == 2 :
        set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=set_speed(left_speed))
        set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=set_speed(right_speed))

    if DIRECTION == 3 :
        set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=set_speed(left_speed))
        set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=set_speed(right_speed))

    if DIRECTION == 4: 
        set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(left_speed))
        set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=set_speed(right_speed))

    draw_window(Robot, left_speed, right_speed, e1.getValue(), e2.getValue())
    localisation(Robot, e1.getValue(), e2.getValue(), e1, e2)
    sleep(0.1)

