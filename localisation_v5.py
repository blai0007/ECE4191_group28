from time import sleep
import pygame
import os
import numpy as np
import math
from PI_Controller import PIController
import RPi.GPIO as GPIO   
from gpiozero import RotaryEncoder, DistanceSensor
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio
from threshold_detect import detector
import cv2
import time

# Starting the PWM MODULE
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # Set PWM frequency for motor control

# Set Pins
in1_left = 5                # LEFT MOTOR - WHEELS
in2_left = 6                

in1_right = 19                 # RIGHT MOTOR - WHEELS
in2_right = 26

encoder1_left_pin = 25          # ENCODER PINS
encoder2_left_pin = 23
encoder1_right_pin = 16 
encoder2_right_pin = 24

# Ultrasonic Pi Pins
echo = 11
trigger = 9

in1_left_belt = 17
in2_left_belt = 27

in1_right_belt = 22
in2_right_belt = 10

# ROTARY ENCODER INIT
e1 = RotaryEncoder(encoder1_left_pin, encoder1_right_pin, max_steps = 100000000)
e2 = RotaryEncoder(encoder2_left_pin, encoder2_right_pin, max_steps = 100000000)

ultrasonic = DistanceSensor(echo=echo,trigger=trigger,threshold_distance=0.07)

# Initialise Pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1_left,GPIO.OUT)
GPIO.setup(in2_left,GPIO.OUT)

GPIO.setup(in1_right_belt,GPIO.OUT)
GPIO.setup(in2_right_belt,GPIO.OUT)

GPIO.setup(in1_left_belt,GPIO.OUT)
GPIO.setup(in2_left_belt,GPIO.OUT)

GPIO.setup(in1_right,GPIO.OUT)
GPIO.setup(in2_right,GPIO.OUT)

GPIO.setup(echo,GPIO.IN)
GPIO.setup(trigger,GPIO.OUT)

GPIO.output(in1_left,GPIO.LOW)              # Setting them all low at first
GPIO.output(in2_left,GPIO.LOW)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.LOW)
GPIO.output(in1_left_belt,GPIO.LOW)
GPIO.output(in2_left_belt,GPIO.LOW)
GPIO.output(in1_right_belt,GPIO.LOW)
GPIO.output(in2_right_belt,GPIO.LOW)

# Initialise Pygame Module
pygame.init()
SCREEN_WIDTH =  900
SCREEN_HEIGHT = 500

# Initiate Pygame Pictures
WIN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("LOCALISATION")

WHITE = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'White.png')), (SCREEN_WIDTH, SCREEN_HEIGHT))

BLUE = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'Blue.png')), (548, 411))

BOX = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'Brown.png')), (45/2, 30))

ORIGIN = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'Origin.png')), (10, 10))

# GLOBAL VARIABLES - FLAGS
GOING_BACK = 0
TURNING_BACK = 0
MOVING_BACK = 0
TURN_TO_BALL = 0
MOVING_TO_BALL = 0

MOVING = 0
MOVING_TARGET = 0
TURNING_TARGET = 0
TURN_TO_REVERSE = 0
MOVE_TO_REVERSE = 0

BALL_FOUND = 0
MOVE_TO_BOX = 0
FLAG_TARGET = 0

# robot Class
class robot : 
    def __init__(self) : 
        # PYGAME VARIABLES
        self.x_pygame = 614               #400
        self.y_pygame = 411           #200
        self.starting_x_pygame = 100       #400
        self.starting_y_pygame = 418      #200
        self.x_target_pygame = 0
        self.y_target_pygame = 0

        # CARTESIAN ROBOT VARIABLES
        self.x_cartesian = self.x_pygame - self.starting_x_pygame
        self.y_cartesian = -(self.y_pygame - self.starting_y_pygame)
        self.deg = 0
        self.x_deposit_cartesian = 0
        self.x_target_cartesian = 0
        self.y_target_cartesian = 0

        # MEASURED (THINGS)
        self.cm_per_tick = 60 / 3300                                  # Nathan and Bryan checked this, measure again if unsure
        self.ticks_per_full_rotation = 3900                            # TODO : Change this after wheel calibration
        self.degrees_per_tick = 360 / self.ticks_per_full_rotation
        self.degrees_per_tick_wheel = 360 / 900    #900     

        # WAITING TIME (DT)
        self.drive_dt = 0.2
        self.turning_dt = 0.2
        self.loop_dt = 0.001

        # SEARCH PATTERN
        self.search_pattern = [(439,80),(460,60),(430,168),(400,275),(371,167),(343,60),(297,167),(251,275),(223,167),(195,60),(153,167),(112,275),(83,167),(54,60),
                               (83,167),(112,275),(153,167),(195,60),(223,167),(251,275),(297,167),(343,60),(371,167),(400,275),(430,168),(460,60),(439,80)]
        # self.search_pattern = [(480,100), (400,200), (370,250), (300,250), (200,250), (100,250), (100,170), (100,100), (200,100), (300,100), (350,100)]
        # self.search_pattern = [(50,100), (100,100), (200, 100), (300, 100), (350,100),  (480, 200), (400, 300), (300, 300), (200,300), (100,300), (100, 200)]
        # self.search_pattern = [(50,100), (100,200), (200, 200), (300, 200), (400,200), (300,200), (200, 200)]
        self.ball_target_pattern = []
        self.ball_target_pattern_iter = 0
        self.search_pattern_iter = 0

        # BALL COUNT
        self.balls_collected = 0

        # VISUALISATION (PYGAME SPRITE)
        self.width = 26
        self.height = 54
        self.wheel_seperation = 24
        self.wheel_radius = 5.5 / 2
        self.image = pygame.image.load(os.path.join('PNGs', 'spaceship_red.png'))
        self.blit = pygame.transform.rotate(pygame.transform.scale(self.image, (self.width, self.height)), 180)
        self.rect = pygame.Rect(700, 300, self.width, self.height)

        # LOCALISATION (ENCODERS)
        self.ticks_left = 0
        self.ticks_right = 0
        self.ticks_left_prev = 0
        self.ticks_right_prev = 0
        self.left_ticks_iter = 0
        self.right_ticks_iter = 0

        self.w_left = 0
        self.w_right = 0 

        # THRESHOLDS
        self.turning_threshold = 20
        self.moving_threshold = 30

class box() : 
    def __init__(self) : 
        self.box_width = 60
        self.box_height = 45

        self.x_box_cartesian = 0
        self.y_box_cartesian = 411
        self.x_deposit_cartesian = 100
        self.y_deposit_cartesian = 300

# VISION FUNCTIONS
def drive_to_ball(robot, area):
    if area > 1000 : 
        if area < 35000 :       # or area > 10000
            m1_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.left_ticks_iter, robot.drive_dt)))
            m2_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.right_ticks_iter, robot.drive_dt)))

            set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
            set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)
            return 0

        elif area > 35000 : # or area < 10000
            drive_stop()
            print("It stopped")
            time.sleep(3)
            
            return 1
        
def center_ball(robot, center):
    m1_speed = 20 
    m2_speed = 20
    
    x_coord = center[0]
    if x_coord <=250 or x_coord >= 350:
        # drive_stop()
        if x_coord < 250: #Ball is on left
            print("On the Left")

            set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=m1_speed)
            set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)
            time.sleep(robot.turning_dt)
            drive_stop()
            return 0

        if x_coord > 350: #Ball is on right
            print("On the Right")
            set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
            set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=m2_speed)
            time.sleep(robot.turning_dt)
            drive_stop()
            return 0
        
    else:
        print("Ball is within 250-350 pixels")
        return 1

# MOTOR CONTROL FUNCTIONS
#       input a percentage 0-100 to set speed

def set_speed(percentage_val):
    speed = int(np.floor((percentage_val/100) * 65535))         # CircuitPython apparently converts to 16 bit number 
    return speed

def set_motor(in1, in2, motor_num, direction, speed):
    if direction: # forward
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
    else: # back
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
    speed = set_speed(speed)
    pca.channels[motor_num].duty_cycle = speed

# FUNCTION TO STOP ALL DRIVING
def drive_stop():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.LOW) 

# DEETERMINING THE NEXT SEARCH PATTERN WAYPOINT
def find_location(robot) : 
    if robot.search_pattern_iter > len(robot.search_pattern)-1 : 
        robot.search_pattern_iter = 1

    (robot.x_target_cartesian, robot.y_target_cartesian) = robot.search_pattern[robot.search_pattern_iter]
    robot.search_pattern_iter += 1
    # print((robot.x_target_cartesian, robot.y_target_cartesian))
    print(f"Target : {robot.x_target_cartesian, robot.y_target_cartesian}")

    robot.x_target_pygame = robot.x_target_cartesian + robot.starting_x_pygame
    robot.y_target_pygame = -robot.y_target_cartesian + robot.starting_y_pygame

    return 1

# MOVING TO BOX FUNCTIONS (TODO : STILL IN SIMULATION - SAME THING AS LOCALISATION_V4)
def turn_to_reverse(robot) :
    ideal_degree = 90

    print(f"TURNING --> Ideal Degree : {ideal_degree}, Current Deg : {robot.deg}")
    if (robot.deg < (ideal_degree-robot.turning_threshold)) or (robot.deg > (ideal_degree+robot.turning_threshold)):           # Not facing centre
        if robot.deg > ideal_degree : 
            m1_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.left_ticks_iter, robot.drive_dt)))
            m2_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.right_ticks_iter, robot.drive_dt)))

            set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=m1_speed)
            set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)

            sleep(robot.turning_dt)
            drive_stop()

        else : 
            m1_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.left_ticks_iter, robot.drive_dt)))
            m2_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.right_ticks_iter, robot.drive_dt)))

            set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
            set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=m2_speed)

            sleep(robot.turning_dt)
            drive_stop()

        return 0
    else : 
        print("FINISHED TURNING : READY TO REVERSE")
        return 1
    
def move_to_reverse(robot, ultrasonic) : 
    # distance_x = robot.x_cartesian - robot.x_deposit_cartesian

    # if distance_x > 10 :
    #     m1_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.left_ticks_iter, robot.drive_dt)))
    #     m2_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.right_ticks_iter, robot.drive_dt)))

    #     set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=m1_speed)
    #     set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=m2_speed)
    #     return 0
    if ultrasonic.distance < 0.09:
        print("ARRIVED AT BOX")
        drive_stop()
        kit.servo[8].angle = 10
        sleep(2)
        kit.servo[8].angle = 100
        return 1
    else:
        print("REVERSING TO BOX")
        m1_speed = 95
        m2_speed = max(0, min(100, pi_controller.motor_setpoint(robot.w_right, robot.w_left, robot.drive_dt)))
        set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=m1_speed)
        set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=m2_speed)

        sleep(robot.drive_dt)
        
        return 0

# TURNING & MOVING TO TARGET
def turn_to_target(robot, e1, e2) : 
    # print(f"Turning to Target : {robot.x_target_pygame, robot.y_target_pygame}")
    distance_x = -(robot.x_pygame - robot.x_target_pygame)
    distance_y = (robot.y_pygame - robot.y_target_pygame)
    ideal_degree = 0

    if (distance_x > 0 ) and (distance_y > 0) : 
        ideal_degree = 90 - math.degrees(math.atan(abs(distance_y)/ abs(distance_x)))
        # ideal_degree = 270 - math.degrees(math.atan(abs(distance_y/distance_x)))
        #print("Quad 1")

    elif (distance_x < 0 ) and (distance_y > 0) : 
        ideal_degree = 270 + math.degrees(math.atan(abs(distance_y)/ abs(distance_x)))
        #print("Quad 2")

    elif (distance_x < 0 ) and (distance_y < 0) : 
        ideal_degree = 270 - math.degrees(math.atan(abs(distance_y)/abs(distance_x)))
        #print("Quad 3")

    elif (distance_x > 0 ) and (distance_y < 0) : 
        ideal_degree = 90 + math.degrees(math.atan(abs(distance_y)/abs(distance_x)))
        #print("Quad 4")
    

    print(f"TURNING --> Ideal Degree : {ideal_degree}, Current Deg : {robot.deg}")
    if (robot.deg < (ideal_degree-robot.turning_threshold)) or (robot.deg > (ideal_degree+robot.turning_threshold)):           # Not facing centre
        if (robot.deg > ideal_degree and abs(ideal_degree-robot.deg)<180 )or (robot.deg < ideal_degree and abs(ideal_degree-robot.deg)>180) : 
            m1_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.left_ticks_iter, robot.drive_dt)))
            m2_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.right_ticks_iter, robot.drive_dt)))

            set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=m1_speed)
            set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)

            sleep(robot.turning_dt)
            drive_stop()

        else : 
            m1_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.left_ticks_iter, robot.drive_dt)))
            m2_speed = 80 #max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.right_ticks_iter, robot.drive_dt)))

            set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
            set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=m2_speed)

            sleep(robot.turning_dt)
            drive_stop()
        return 0

    else : 
        print(f"FACING TARGET")
        return 1
    
def moving_to_target(robot, e1, e2) : 
    print("MOVING TO TARGET")
    distance_x = robot.x_cartesian - robot.x_target_cartesian
    distance_y = -(robot.y_cartesian - robot.y_target_cartesian)

    distance_overall = np.sqrt(distance_x**2 + distance_y**2)
    #print(f"distance : {distance_overall}")

    if distance_overall > robot.moving_threshold : 
        m1_speed = 80#max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.left_ticks_iter, robot.drive_dt)))
        m2_speed = 80#max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.right_ticks_iter, robot.drive_dt)))

        set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
        set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)

        sleep(robot.drive_dt)
        drive_stop()

        return 0

    else : 
        print("TARGET Reached")
        drive_stop()
        return 1
    
# FUNCTION TO DRIV FORWARD
def drive_forward(robot) : 
    m1_speed = 80
    m2_speed = 80

    set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
    set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)

    sleep(robot.drive_dt)
    drive_stop()
    
def localisation(robot) : 
    distance_moved = 0
    degrees_turned = 0

    # Determing left and right tick changes
    robot.left_ticks_iter = robot.ticks_left-robot.ticks_left_prev
    robot.right_ticks_iter = robot.ticks_right-robot.ticks_right_prev

    # MOVE FORWARDS
    if (robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("PYGAME ACKNOWLEDGE :  IT IS MOVING FORWARD")
        # Determing wheel angular velocity (LEFT & RIGHT)
        robot.w_left = (robot.left_ticks_iter / robot.drive_dt) * robot.degrees_per_tick_wheel        # deg / s
        robot.w_right = (robot.right_ticks_iter / robot.drive_dt) * robot.degrees_per_tick_wheel      # deg / s

        # Determing wheel linear velocity (LEFT & RIGHT)
        v_left = np.deg2rad(robot.w_left)*robot.wheel_radius                  # cm / s
        v_right = np.deg2rad(robot.w_right)*robot.wheel_radius                # cm / s

        # Determing whole robot's angular and linear velocity
        v = (np.deg2rad(robot.w_left)*robot.wheel_radius + np.deg2rad(robot.w_right)*robot.wheel_radius)/2                              # cm / s
        w = np.rad2deg(abs(np.deg2rad(robot.w_left)*robot.wheel_radius - np.deg2rad(robot.w_right)*robot.wheel_radius)/robot.wheel_seperation  )    # deg / s
        #print(f"PYGAME ACKNOWLEDGE : w = {w} deg / s")
        #print(f"PYGAME ACKNOWLEDGE : v = {v} cm / s")

        # LEFT WHEEL IS SLOWER THAN RIGHT WHEEL (TILT LEFT)
        if (robot.ticks_left-robot.ticks_left_prev) < (robot.ticks_right - robot.ticks_right_prev) :   
            print("PYGAME ACKNOWLEDGE :  IT IS MOVING FORWARD (TILT LEFTWARDS)")

            robot.y_pygame -= v*np.cos(np.deg2rad(robot.deg))*robot.drive_dt
            robot.x_pygame += v*np.sin(np.deg2rad(robot.deg))*robot.drive_dt
            robot.deg -= w*robot.drive_dt

        # LEFT WHEEL IS FASTER THAN RIGHT WHEEL (TILT RIGHT)
        elif (robot.ticks_left-robot.ticks_left_prev) > (robot.ticks_right - robot.ticks_right_prev ) : 
            print("PYGAME ACKNOWLEDGE :  IT IS MOVING FORWARD (TILT RIGHTWARDS)")

            robot.y_pygame -= v*np.cos(np.deg2rad(robot.deg))*robot.drive_dt
            robot.x_pygame += v*np.sin(np.deg2rad(robot.deg))*robot.drive_dt
            robot.deg = robot.deg + w*robot.drive_dt

        # LEFT WHEEL IS THE SAME SPEEED WITH RIGHT WHEEL (NO TILT)
        else : 
            print("PYGAME ACKNOWLEDGE :  IT IS MOVING FORWARD (NO TILT)")
            robot.y_pygame -= v*np.cos(np.deg2rad(robot.deg)) * robot.drive_dt
            robot.x_pygame += v*np.sin(np.deg2rad(robot.deg)) * robot.drive_dt

    # ROBOT IS ROTATING LEFT
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        # Determing wheel angular velocity (LEFT & RIGHT)
        robot.w_left = (robot.left_ticks_iter / robot.turning_dt) * robot.degrees_per_tick_wheel          # deg / s
        robot.w_right = (robot.right_ticks_iter / robot.turning_dt) * robot.degrees_per_tick_wheel        # deg / s

        # Determing wheel linear velocity (LEFT & RIGHT)
        v_left = np.deg2rad(robot.w_left)*robot.wheel_radius                  # cm / s
        v_right = np.deg2rad(robot.w_right)*robot.wheel_radius                # cm / s

        # Determing whole robot's angular and linear velocity
        v = (np.deg2rad(robot.w_left)*robot.wheel_radius + np.deg2rad(robot.w_right)*robot.wheel_radius)/2                                  # cm / s
        w = np.rad2deg(abs(np.deg2rad(robot.w_left)*robot.wheel_radius - np.deg2rad(robot.w_right)*robot.wheel_radius)/robot.wheel_seperation )         # deg / s
        degrees_turned = w*robot.turning_dt  

        #print(f"PYGAME ACKNOWLEDGE : w = {w} deg / s")
        #print(f"PYGAME ACKNOWLEDGE : v = {v} cm / s")
        print("PYGAME ACKNOWLEDGE :  IT IS ROTATING LEFT")
        #print(f"Deg turned : {degrees_turned}")
        robot.deg -= degrees_turned

    # ROBOT IS ROTATING RIGHT
    elif ( robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        # Determing wheel angular velocity (LEFT & RIGHT)
        robot.w_left = (robot.left_ticks_iter / robot.turning_dt) * robot.degrees_per_tick_wheel          # deg / s
        robot.w_right = (robot.right_ticks_iter / robot.turning_dt) * robot.degrees_per_tick_wheel        # deg / s

        # Determing wheel linear velocity (LEFT & RIGHT)
        v_left = np.deg2rad(robot.w_left)*robot.wheel_radius                  # cm / s
        v_right = np.deg2rad(robot.w_right)*robot.wheel_radius                # cm / s

        # Determing whole robot's angular and linear velocity
        v = (np.deg2rad(robot.w_left)*robot.wheel_radius + np.deg2rad(robot.w_right)*robot.wheel_radius)/2                              # cm / s
        w = np.rad2deg(abs(np.deg2rad(robot.w_left)*robot.wheel_radius - np.deg2rad(robot.w_right)*robot.wheel_radius)/robot.wheel_seperation)       # deg / s

        #print(f"PYGAME ACKNOWLEDGE : w = {w} deg / s")
        #print(f"PYGAME ACKNOWLEDGE : v = {v} cm / s")
        degrees_turned = w*robot.turning_dt   
        print("PYGAME ACKNOWLEDGE :  IT IS ROTATING RIGHT")  
        #print(f"Deg turned : {degrees_turned}")
        robot.deg += degrees_turned

    # CLAMPING ROBOT DEGREE BETWEEN 0 AND 360 
    if robot.deg < 0 : 
        robot.deg = 360 + robot.deg

    elif robot.deg > 360 :
        robot.deg = robot.deg - 360

    # UPDATING TICKS IN ROBOT CLASS
    robot.ticks_left_prev = robot.ticks_left
    robot.ticks_right_prev = robot.ticks_right

    return
    
# DRAWING PYGAME WINDOW FOR US TO SEE
def draw_window(robot):
    WIN.blit(WHITE, (0, 0))
    WIN.blit(BLUE, (100,50))
    WIN.blit(BOX, (100,50))
    WIN.blit(ORIGIN, (648,418))
    WIN.blit(ORIGIN, (robot.x_target_pygame, robot.y_target_pygame))
    robot.blit = pygame.transform.rotate(pygame.transform.scale(robot.image, (robot.width, robot.height)), -robot.deg+180)
    WIN.blit(robot.blit, (robot.x_pygame, robot.y_pygame))

    robot.x_cartesian = robot.x_pygame - robot.starting_x_pygame
    robot.y_cartesian = -(robot.y_pygame - robot.starting_y_pygame)

    # FONTS / TEXTS
    pygame.font.init()
    my_font = pygame.font.SysFont('Comic Sans MS', 15)
    location_title_txt = my_font.render("Position: ", False, (0, 0, 0))
    location_txt = my_font.render(f'({np.round((robot.x_cartesian),2)},{np.round(robot.y_cartesian,2)})', False, (0, 0, 0))
    WIN.blit(location_title_txt, (658,80))
    WIN.blit(location_txt, (678,100))

    degrees_title_txt = my_font.render("Degrees : ", False, (0, 0, 0))
    degrees_txt = my_font.render(f'{np.round(robot.deg,2)}', False, (0, 0, 0))
    WIN.blit(degrees_title_txt, (658,120))
    WIN.blit(degrees_txt, (678,140))

    left_ticks_title_txt = my_font.render("LEFT TICK (NORMAL) :", False, (0, 0, 0))
    left_ticks_txt = my_font.render(f'{np.round(robot.ticks_left)}', False, (0, 0, 0))
    WIN.blit(left_ticks_title_txt, (658,160))
    WIN.blit(left_ticks_txt, (678,180))

    right_ticks_title_txt = my_font.render("Right TICK (NORMAL) :", False, (0, 0, 0))
    right_ticks_txt = my_font.render(f'{np.round(robot.ticks_right)}', False, (0, 0, 0))
    WIN.blit(right_ticks_title_txt, (658,200))
    WIN.blit(right_ticks_txt, (678,220))

    target_title_txt = my_font.render("Target Now :  ", False, (0, 0, 0))
    target_txt = my_font.render(f'({np.round((robot.x_target_cartesian),2)},{np.round(robot.y_target_cartesian,2)})', False, (0, 0, 0))
    WIN.blit(target_title_txt, (658,240))
    WIN.blit(target_txt, (678,260))

    balls_title_txt = my_font.render("Balls Collected :  ", False, (0, 0, 0))
    balls_txt = my_font.render(f'{(robot.balls_collected)}', False, (0, 0, 0))
    WIN.blit(balls_title_txt, (658,280))
    WIN.blit(balls_txt, (678,300))

    ball_found_title_txt = my_font.render("Ball FOUND ? :", False, (0, 0, 0))
    if BALL_FOUND == 1: 
        ball_found_txt = my_font.render("YES", False, (0, 0, 0))
    else: 
        ball_found_txt = my_font.render("NO", False, (0, 0, 0))
    WIN.blit(ball_found_title_txt, (658,320))
    WIN.blit(ball_found_txt, (800,320))

    pygame.display.update()

# START PROGRAM
FPS = 60
Robot = robot()
Box = box()
Detector = detector()
pi_controller = PIController(Kp=0.0001,Ki=0.01,Kd=0)

# INITIALISING SERVOS
kit = ServoKit(channels=16)
kit.servo[4].angle = 0
kit.servo[15].angle = 140

# Initialise Video Capture
vs = cv2.VideoCapture(0)
time.sleep(0.2)

# START LOOP
try:
    left_belt_speed = 80
    right_belt_speed = 80
    set_motor(in1_left_belt, in2_left_belt, motor_num=2, direction=1, speed=left_belt_speed)
    set_motor(in1_right_belt, in2_right_belt, motor_num=3, direction=1, speed=right_belt_speed)
    while(True):
        print("###########################################################")
        # TODO  : Setpoint
        expected_rpm = 75 # EXPECTED SPEED OF MOTOR 0-100
        if MOVING:
            expected_tick_per_sec = expected_rpm * (900/60)
        else:
            expected_tick_per_sec = 0

        # Video Capture 
        _,frame = vs.read()
        frame, centroid, rad, area, mask = Detector.find_ball(frame)
        
        # FIND NEW LOCATION
        if not MOVING and not BALL_FOUND and not MOVE_TO_BOX: 
            find_location(Robot)
            MOVING = 1
            TURNING_TARGET = 1

        # MOVING TO BOX SUBFUNCTION
        elif MOVE_TO_BOX == 1 : 
            if TURNING_TARGET == 1 : 
                if (turn_to_target(Robot, e1, e2)) :
                    print(f"Finish Turning to BOX Waypoint : ({Box.x_deposit_cartesian},{Box.y_deposit_cartesian})") 
                    TURNING_TARGET = 0
                    MOVING_TARGET = 1

            if MOVING_TARGET == 1 : 
                FLAG_TARGET = moving_to_target(Robot, e1, e2)               # 1 means reached waypoint, 0 mean not yet
                if (FLAG_TARGET)==1 : 
                    print(f"Finish MOVING to BOX Waypoint : ({Box.x_deposit_cartesian},{Box.y_deposit_cartesian})") 
                    TURN_TO_REVERSE = 1
                    MOVING_TARGET = 0
                    
                elif (FLAG_TARGET) == 0 : 
                    MOVING_TARGET = 0
                    TURNING_TARGET = 1
                    
            elif TURN_TO_REVERSE == 1 : 
                if (turn_to_reverse(Robot))==1 :               # 1 means finsh, 0 to start reversing
                    print(f"FINISHING TURNING, BUTT IS NOW TO BOX (deg : {Robot.deg})")
                    TURN_TO_REVERSE = 0
                    MOVE_TO_REVERSE = 1

            elif MOVE_TO_REVERSE == 1 : 
                if (move_to_reverse(Robot, ultrasonic)) : 
                    print(f"MIGUEL PLS ... ")
                    Robot.balls_collected = 0
                    MOVE_TO_BOX = 0
                    MOVING = 0
                    BALL_FOUND = 0
                    MOVING_TARGET = 0 

        # MOVING TO BALL SUBFUNCTION
        elif BALL_FOUND == 1 :
            if centroid != None: 
                if TURN_TO_BALL == 1 :
                    print("SYSTEM ACKNOWLDGE : TURNING TO TENNIS BALL")
                    if (center_ball(Robot, centroid) == 1) : 
                        MOVING_TO_BALL = 1
                        TURN_TO_BALL = 0
                
                if MOVING_TO_BALL == 1 : 
                    print("SYSTEM ACKNOWLDGE : MOVING TO TENNIS BALL")
                    drive_forward(Robot)
                    TURN_TO_BALL = 1

                    if centroid == None :
                        Robot.balls_collected += 1
                        BALL_FOUND == 0
                        MOVING = 0
                        TURN_TO_BALL = 0
                        MOVING_TO_BALL = 0
                        
            elif centroid == None: 
                BALL_FOUND = 0
                MOVING = 0
                TURN_TO_BALL = 0
                MOVING_TO_BALL = 0
                Robot.balls_collected += 1

                

            # if TURNING_TARGET == 1 : 
            #     if (turn_to_target(Robot, e1, e2)) : 
            #         TURNING_TARGET = 0
            #         MOVING_TARGET = 1

            # if MOVING_TARGET == 1 : 
            #     if (moving_to_target(Robot, e1, e2)) : 
            #         FLAG_TARGET = moving_to_target(Robot, e1, e2)
            #         if (FLAG_TARGET)==1 : 
            #             MOVING = 0
            #             MOVING_TARGET = 0
                        
            #         elif (FLAG_TARGET) == 0 : 
            #             MOVING_TARGET = 0
            #             TURNING_TARGET = 1

        # MOVING TO SEARCH PATTERN WAYPOINT
        elif BALL_FOUND == 0 : 
            if TURNING_TARGET == 1 : 
                if (turn_to_target(Robot, e1, e2)) : 
                    TURNING_TARGET = 0
                    MOVING_TARGET = 1

            if MOVING_TARGET == 1 : 
                FLAG_TARGET = moving_to_target(Robot, e1, e2)
                if (FLAG_TARGET)==1 : 
                    MOVING = 0
                    MOVING_TARGET = 0
                    
                elif (FLAG_TARGET) == 0 : 
                    MOVING_TARGET = 0
                    TURNING_TARGET = 1

        # BALL FINDING (VISION)
        if centroid != None or area != None: 
            if area > 1000 : 
                print("VISION ACKNOWLEDGE : BALL DETECTED")
                BALL_FOUND = 1
                MOVING = 1
                TURN_TO_BALL = 1
                MOVING_TO_BALL = 0

    
        # TO SIMULATE BALL FINDING
        # for event in pygame.event.get():
        #     if event.type == pygame.quit : 
        #         break

            # if event.type == pygame.MOUSEBUTTONDOWN :
            #     if event.type == pygame.MOUSEBUTTONDOWN :
            #         BALL_FOUND = 1
            #         MOVING = 1
            #         TURNING_TARGET = 1
            #         MOVING_TARGET = 0
            #         # (Robot.x_target_pygame, Robot.y_target_pygame) = pygame.mouse.get_pos()
            #         (x_ball_target_pygame, y_ball_target_pygame) = pygame.mouse.get_pos()
            #         x_ball_target_cartesian = x_ball_target_pygame - Robot.starting_x_pygame
            #         y_ball_target_cartesian = -(y_ball_target_pygame - Robot.starting_y_pygame)


            #         # Robot.y_target_pygame = - Robot.y_target_pygame
            #         if ((x_ball_target_cartesian > 0) and (x_ball_target_cartesian < 548)) and ((y_ball_target_cartesian > 0) and (y_ball_target_cartesian < 370)): 
            #             BALL_FOUND = 0
            #             Robot.x_target_pygame = x_ball_target_pygame
            #             Robot.y_target_pygame = y_ball_target_pygame
            #             Robot.x_target_cartesian = Robot.x_target_pygame - Robot.starting_x_pygame
            #             Robot.y_target_cartesian = -(Robot.y_target_pygame - Robot.starting_y_pygame)
        
        # CHECKS THE BALLS (BALL COUNT)
        if Robot.balls_collected >= 2 and MOVE_TO_BOX == 0:  
            Robot.x_target_cartesian = Box.x_deposit_cartesian
            Robot.y_target_cartesian = Box.y_deposit_cartesian
            Robot.x_target_pygame = Robot.x_target_cartesian + Robot.starting_x_pygame
            Robot.y_target_pygame = - Robot.y_target_cartesian + Robot.starting_x_pygame           # Robot.starting_y_pygame

            if MOVE_TO_BOX == 0 : 
                MOVE_TO_BOX = 1
                MOVING = 1
                
                TURNING_TARGET = 1
                MOVING_TARGET = 0

        # CHCKS IF MIGUEL IS MOVING OUT OF THE BORDERS
        # if ((Robot.x_cartesian < 0) or (Robot.x_cartesian > 518)) or ((Robot.y_cartesian < 0) or (Robot.y_cartesian > 380)): 
        #     print("MIGUEL IS OUT OF BOUNDS")
        #     MOVING = 1
        #     BALL_FOUND = 0
        #     TURNING_TARGET = 1
        #     MOVING_TARGET = 0

        #     if (Robot.x_cartesian > 200 and Robot.y_cartesian > 200) : 
        #         Robot.x_target_cartesian = 300
        #         Robot.y_target_cartesian = 300

        #     elif (Robot.x_cartesian > 200 and Robot.y_cartesian < 200) : 
        #         Robot.x_target_cartesian = 300
        #         Robot.y_target_cartesian = 100

        #     elif (Robot.x_cartesian < 200 and Robot.y_cartesian < 200) : 
        #         Robot.x_target_cartesian = 100
        #         Robot.y_target_cartesian = 100

        #     elif (Robot.x_cartesian < 200 and Robot.y_cartesian > 200) : 
        #         Robot.x_target_cartesian = 100
        #         Robot.y_target_cartesian = 300

        #     Robot.x_target_pygame = Robot.x_target_cartesian + Robot.starting_x_pygame
        #     Robot.y_target_pygame = - Robot.y_target_cartesian + Robot.starting_y_pygame

        # LOCALISATION
        Robot.ticks_left = e1.steps
        Robot.ticks_right = e2.steps
        localisation(Robot)
        draw_window(Robot)

        # LEFT TICKS AND RIGHT TICKS
        print(f"ROBOT X : {Robot.x_cartesian}")
        print(f"ROBOT Y : {Robot.y_cartesian}")
        print(f"ROBOT.Deg : {Robot.deg}")
        print(f'Ball Count:{Robot.balls_collected}')
        sleep(Robot.loop_dt)

except KeyboardInterrupt:
    kit.servo[4].angle = 140
    kit.servo[15].angle = 0
    GPIO.cleanup()

