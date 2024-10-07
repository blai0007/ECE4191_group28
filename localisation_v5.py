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


# NOTE : #
# The robot is facing upwards"
# Set Pins
in1_left = 5 # 23
in2_left = 6 # 24
en_left =  11 #25                # Simulating encoder

in1_right = 19
in2_right = 26
en_right = 13               # simulating encoder

encoder1_left_pin = 25 # 7
encoder2_left_pin = 23
encoder1_right_pin = 16 #8
encoder2_right_pin = 24

left_speed = 75
right_speed = 75

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # Set PWM frequency for motor control

e1 = RotaryEncoder(encoder1_left_pin, encoder1_right_pin, max_steps = 100000000)
e2 = RotaryEncoder(encoder2_left_pin, encoder2_right_pin, max_steps = 100000000)

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
    os.path.join('PNGs', 'Brown.png')), (30, 50))

ORIGIN = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'Origin.png')), (10, 10))


GOING_BACK = 0
TURNING_BACK = 0
MOVING_BACK = 0

MOVING = 0
MOVING_TARGET = 0
TURNING_TARGET = 0
TURN_TO_REVERSE = 0
MOVE_TO_REVERSE = 0

BALL_FOUND = 0
MOVE_TO_BOX = 0

# robot Class
class robot : 
    def __init__(self) : 
        self.ticks_left = 0
        self.ticks_right = 0

        self.ticks_left_prev = 0
        self.ticks_right_prev = 0

        self.x_pygame = 100               #400
        self.y_pygame = 438           #200
        self.starting_x_pygame = 100       #400
        self.starting_y_pygame = 438      #200
        self.x_cartesian = self.x_pygame - self.starting_x_pygame
        self.y_cartesian = -(self.y_pygame - self.starting_y_pygame)
        self.deg = 0

        self.m_per_tick = 4                                # Nathan and Bryan checked this, measure again if unsure
        self.ticks_per_full_rotation = 3000                              # TODO : Change this after wheel calibration
        self.degrees_per_tick = 360 / self.ticks_per_full_rotation   

        self.prev_e1_val = 0
        self.prev_e2_val = 0 

        self.distance_per_iter = 2                          # TODO : Used only for demo 1 (Only 1n approx)
        self.deg_per_iter = 2

        self.x_deposit_cartesian = 0

        self.x_target_pygame = 0
        self.y_target_pygame = 0
        self.x_target_cartesian = 0
        self.y_target_cartesian = 0

        self.search_pattern = [(50,100), (100,200), (200, 200), (300, 200), (400,200), (300,200), (200, 200)]
        self.ball_target_pattern = []
        self.ball_target_pattern_iter = 0
        self.search_pattern_iter = 0

        self.balls_collected = 0

        # VISUALISATION
        self.width = 19
        self.height = 23
        self.image = pygame.image.load(os.path.join('PNGs', 'spaceship_red.png'))
        self.blit = pygame.transform.rotate(pygame.transform.scale(self.image, (self.width, self.height)), 180)
        self.rect = pygame.Rect(700, 300, self.width, self.height)

        # LOCALISATION (ENCODERS)
        self.ticks_left = 0
        self.ticks_right = 0
        self.ticks_left_prev = 0
        self.ticks_right_prev = 0
        self.left_mag = 0
        self.right_mag = 0
        self.left_a = 0
        self.left_b = 0

# input a percentage 0-100 to set speed
def set_speed(percentage_val):
    speed = int(np.floor((percentage_val/100) * 65535))# CircuitPython apparently converts to 16 bit number 
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

def drive_stop():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.LOW) 

def find_location(robot) : 
    # robot.x_target_cartesian = float(input("X Coordinate : "))
    # robot.y_target_cartesian = float(input("Y coordinate : "))
    if robot.search_pattern_iter > len(robot.search_pattern)-1 : 
        robot.search_pattern_iter = 1

    (robot.x_target_cartesian, robot.y_target_cartesian) = robot.search_pattern[robot.search_pattern_iter]
    robot.search_pattern_iter += 1
    # print((robot.x_target_cartesian, robot.y_target_cartesian))
    print(f"Target : {robot.x_target_cartesian, robot.y_target_cartesian}")

    robot.x_target_pygame = robot.x_target_cartesian + robot.starting_x_pygame
    robot.y_target_pygame = -robot.y_target_cartesian + robot.starting_y_pygame

    return 1

def turn_to_reverse(robot) :
    threshold = 5
    ideal_degree = 90

    if (robot.deg < (ideal_degree-threshold)) or (robot.deg > (ideal_degree+threshold)):           # Not facing centre
        if robot.deg > ideal_degree : 
            robot.ticks_left -= 40
            robot.ticks_right += 25
            # robot.deg -= robot.deg_per_iter
        else : 
            # robot.deg += robot.deg_per_iter
            robot.ticks_left += 40
            robot.ticks_right -= 25
        return 0
    else : 
        return 1
    
def move_to_reverse(robot) : 
    distance_x = robot.x_cartesian - robot.x_deposit_cartesian

    if distance_x > 10 :
        robot.ticks_left -= 20
        robot.ticks_right -= 20
        return 0

    else :
        return 1

def turn_to_target(robot) : 
    threshold = 5
    print(f"Turning to Target : {robot.x_target_pygame, robot.y_target_pygame}")
    distance_x = robot.x_pygame - robot.x_target_pygame
    distance_y = -(robot.y_pygame - robot.y_target_pygame)
    ideal_degree = 0

    if (distance_x > 0 ) and (distance_y > 0) : 
        ideal_degree = 270 - math.degrees(math.atan(abs(distance_y/distance_x)))
        print("Quad 1")

    elif (distance_x > 0 ) and (distance_y < 0) : 
        ideal_degree = 270 + math.degrees(math.atan(abs(distance_y/distance_x)))
        print("Quad 2")

    elif (distance_x < 0 ) and (distance_y < 0) : 
        ideal_degree = 90 - math.degrees(math.atan(abs(distance_y/distance_x)))
        print("Quad 3")

    elif (distance_x < 0 ) and (distance_y > 0) : 
        ideal_degree = 90 + math.degrees(math.atan(abs(distance_y/distance_x)))
        print("Quad 4")

    print(f"Ideal Degree : {ideal_degree}")
    if (robot.deg < (ideal_degree-threshold)) or (robot.deg > (ideal_degree+threshold)):           # Not facing centre
        if robot.deg > ideal_degree : 
            m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
            m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

            set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=m1_speed)
            set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)

            sleep(0.5)
            drive_stop()

            robot.ticks_left -= abs(e1.steps - robot.prev_e1_val)
            robot.ticks_right += abs(e2.steps - robot.prev_e2_val)
            # robot.deg -= robot.deg_per_iter

        else : 
            m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
            m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

            set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
            set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=m2_speed)

            sleep(0.5)
            drive_stop()

            # robot.deg += robot.deg_per_iter
            robot.ticks_left += abs(e1.steps - robot.prev_e1_val)
            robot.ticks_right -= abs(e2.steps - robot.prev_e2_val)

        return 0

    else : 
        print("FACING CENTER")
        print(f"ideal_degree:{ideal_degree}")
        return 1
    
def moving_to_target(robot) : 
    print("MOVING TO TARGET")
    distance_x = robot.x_cartesian - robot.x_target_cartesian
    distance_y = -(robot.y_cartesian - robot.y_target_cartesian)

    distance_overall = np.sqrt(distance_x**2 + distance_y**2)
    print(f"distance : {distance_overall}")

    if distance_overall > 20 : 
        m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
        m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

        set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
        set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)

        sleep(0.5)
        drive_stop()

        robot.ticks_left += abs(e1.steps - robot.prev_e1_val)
        robot.ticks_right += abs(e2.steps - robot.prev_e2_val)
        # robot.y_pygame -= np.cos(np.deg2rad(robot.deg)) * robot.distance_per_iter
        # robot.x_pygame += np.sin(np.deg2rad(robot.deg)) * robot.distance_per_iter
        return 0

    else : 
        print("TARGET Reached")
        drive_stop()
        MOVING = 0
        return 1
    
def localisation(robot) : 
    distance_moved = 0
    degrees_turned = 0
    
    left_ticks_iter = robot.ticks_left-robot.ticks_left_prev
    right_ticks_iter = robot.ticks_right-robot.ticks_right_prev

    # MOVE FORWARDS
    if (robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its Forwards")
        if (robot.ticks_left-robot.ticks_left_prev) < (robot.ticks_right - robot.ticks_right_prev) :   
            print("Titling Leftwards")
            # R = ((right_ticks_iter+left_ticks_iter)*(robot.width/2) / (left_ticks_iter-right_ticks_iter)) 
            # L = np.sqrt((np.cos(np.deg2rad(robot.deg)))**2 + (np.sin(np.deg2rad(robot.deg)))**2)
            # alpha = np.rad2deg(np.arctan(L/R))

            R = ((right_ticks_iter+left_ticks_iter)*(robot.width/2) / (-left_ticks_iter+right_ticks_iter)) * robot.m_per_tick
            v = np.sqrt((np.cos(np.deg2rad(robot.deg))*robot.m_per_tick)**2 + (np.sin(np.deg2rad(robot.deg))*robot.m_per_tick)**2)  / 0.1
            w = v/R
            new_robot_deg = robot.deg + w*0.1
            robot.y_pygame -= (np.cos(np.deg2rad(robot.deg)) - np.cos(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)
            robot.x_pygame += (-np.sin(np.deg2rad(robot.deg)) + np.sin(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)
            print(f"R : {R}")
            print(f"w : {w}")
            print(f"Y-Change : {(np.cos(np.deg2rad(robot.deg)) - np.cos(np.deg2rad(new_robot_deg))) * R * robot.m_per_tick}")
            print(f"X-Change : {-(np.sin(np.deg2rad(robot.deg)) + np.sin(np.deg2rad(new_robot_deg))) * (R * robot.m_per_tick)}")
            robot.deg = new_robot_deg

        elif (robot.ticks_left-robot.ticks_left_prev) > (robot.ticks_right - robot.ticks_right_prev ) : 
            print("Titling Rightwards")
            R = ((right_ticks_iter+left_ticks_iter)*(robot.width/2) / (left_ticks_iter-right_ticks_iter)) * robot.m_per_tick
            v = np.sqrt((np.cos(np.deg2rad(robot.deg))*robot.m_per_tick)**2 + (np.sin(np.deg2rad(robot.deg))*robot.m_per_tick)**2)  / 0.1
            w = v/R
            new_robot_deg = robot.deg + w*0.1
            robot.y_pygame -= (np.cos(np.deg2rad(robot.deg)) - np.cos(np.deg2rad(new_robot_deg))) * (R)
            robot.x_pygame += (-np.sin(np.deg2rad(robot.deg)) + np.sin(np.deg2rad(new_robot_deg))) * (R)
            print(f"R : {R}")
            print(f"Y-Change : {(np.cos(np.deg2rad(robot.deg)) - np.cos(np.deg2rad(new_robot_deg))) * R}")
            print(f"X-Change : {-(np.sin(np.deg2rad(robot.deg)) + np.sin(np.deg2rad(new_robot_deg))) * (R)}")
            robot.deg = new_robot_deg

        else : 
            # v = (left_ticks_iter+right_ticks_iter)/0.1
            robot.y_pygame -= np.cos(np.deg2rad(robot.deg)) * (robot.m_per_tick)
            robot.x_pygame += np.sin(np.deg2rad(robot.deg)) * (robot.m_per_tick)
            
    # MOVE LEFT
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its MOVING LEFT")
        degrees_turned = (right_ticks_iter - left_ticks_iter) * 0.1
        print(f"Deg turned : {degrees_turned}")
        robot.deg -= degrees_turned
        #deg_turned = rotation_calib 

    # # MOVE RIGHT
    if ( robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        degrees_turned = (-right_ticks_iter + left_ticks_iter) * 0.1
        print(f"Deg turned : {degrees_turned}")
        print("Its MOVING RIGHT")
        robot.deg += degrees_turned

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
    return

def ball_path(robot, x_target_cartesian, y_target_cartesian): 
    distance_x = robot.x_cartesian - robot.x_target_cartesian
    distance_y = -(robot.y_cartesian - robot.y_target_cartesian)

    increment_x = distance_x / 2
    increment_y = distance_y / 2

    for i in range(2) : 
        waypoint_x = robot.x_cartesian + increment_x
        waypoint_y = robot.y_cartesian + increment_y 

        robot.ball_target_pattern.append((waypoint_x, waypoint_y))

    robot.x_target_cartesian = robot.x_cartesian + increment_x
    robot.y_target_cartesian = robot.y_cartesian - increment_y
    
def draw_window(robot):
    WIN.blit(WHITE, (0, 0))
    WIN.blit(BLUE, (100,50))
    WIN.blit(BOX, (100,50))
    WIN.blit(ORIGIN, (robot.starting_x_pygame+(robot.width/2), robot.starting_y_pygame+(robot.height/2)))
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

# def find_nearest_waypoint(robot):
#     robot.search_pattern_iter = np.argmin(robot.search_pattern-(robot.x_cartesian, robot.y_cartesian))

def find_location_ball(robot) :
    # robot.x_target_cartesian = float(input("X Coordinate : "))
    # robot.y_target_cartesian = float(input("Y coordinate : "))
    if robot.ball_target_pattern_iter > len(robot.ball_target_pattern)-1 :      # Reached Ball
        # find_nearest_waypoint(robot)
        robot.ball_target_pattern = []
        robot.balls_collected += 1
        return 1

    (robot.x_target_cartesian, robot.y_target_cartesian) = robot.ball_target_pattern[robot.ball_target_pattern_iter]
    robot.ball_target_pattern_iter += 1
    # print((robot.x_target_cartesian, robot.y_target_cartesian))
    print(f"WAYPOINT To BALL : {robot.x_target_cartesian, robot.y_target_cartesian}")

    robot.x_target_pygame = robot.x_target_cartesian - robot.starting_x_pygame
    robot.y_target_pygame = -(robot.y_target_cartesian - robot.starting_y_pygame)
    return 0

# Start
FPS = 60
Robot = robot()
pi_controller = PIController(Kp=10, Ki=0.06)

kit = ServoKit(channels=16)
kit.servo[4].angle = 0
kit.servo[15].angle = 140
try:
    while(True):
        # Setpoint
        expected_rpm = 75 # EXPECTED SPEED OF MOTOR 0-100
        if MOVING:
            expected_tick_per_sec = expected_rpm * (900/60)
        else:
            expected_tick_per_sec = 0
        dt = 0.1
        Robot.prev_e1_val = e1.steps
        Robot.prev_e2_val = e2.steps
        # Ticks per second
        left_ticks_iter = abs(Robot.ticks_left-Robot.ticks_left_prev) / dt
        right_ticks_iter = abs(Robot.ticks_right-Robot.ticks_right_prev) / dt

        if not MOVING and not BALL_FOUND and not MOVE_TO_BOX: 
            find_location(Robot)
            MOVING = 1
            TURNING_TARGET = 1

        elif MOVE_TO_BOX == 1 : 
            if TURNING_TARGET == 1 : 
                if (turn_to_target(Robot)) : 
                    TURNING_TARGET = 0
                    MOVING_TARGET = 1

            if MOVING_TARGET == 1 : 
                if (moving_to_target(Robot)) : 
                    print("BOX REACHED")
                    TURN_TO_REVERSE = 1
                    MOVING_TARGET = 0
                    
            if TURN_TO_REVERSE == 1 : 
                if (turn_to_reverse(Robot)) : 
                    TURNING_TO_REVERSE = 0
                    MOVE_TO_REVERSE = 1

            if MOVE_TO_REVERSE == 1 : 
                if (move_to_reverse(Robot)) : 
                    Robot.balls_collected = 0
                    if MOVE_TO_BOX == 1 :
                        MOVE_TO_BOX = 0
                        MOVING = 0
                        BALL_FOUND = 0
                        MOVING_TARGET = 0 

        elif BALL_FOUND == 1 :
            if TURNING_TARGET == 1 : 
                if (turn_to_target(Robot)) : 
                    TURNING_TARGET = 0
                    MOVING_TARGET = 1

            if MOVING_TARGET == 1 : 
                if (moving_to_target(Robot)) : 
                    print("BALL FOUND")
                    MOVING = 0
                    MOVING_TARGET = 0 
                    if BALL_FOUND == 1 : 
                        print("BALL REACHED")
                        Robot.balls_collected += 1
                        BALL_FOUND = 0

        elif BALL_FOUND == 0 : 
            if TURNING_TARGET == 1 : 
                if (turn_to_target(Robot)) : 
                    TURNING_TARGET = 0
                    MOVING_TARGET = 1

            if MOVING_TARGET == 1 : 
                if (moving_to_target(Robot)) : 
                    MOVING = 0
                    MOVING_TARGET = 0
                
        for event in pygame.event.get():
            if event.type == pygame.quit : 
                break

            if event.type == pygame.MOUSEBUTTONDOWN :
                BALL_FOUND = 1
                MOVING = 1
                TURNING_TARGET = 1
                MOVING_TARGET = 0
                (Robot.x_target_pygame, Robot.y_target_pygame) = pygame.mouse.get_pos()
                # Robot.y_target_pygame = - Robot.y_target_pygame
                Robot.x_target_cartesian = Robot.x_target_pygame - Robot.starting_x_pygame
                Robot.y_target_cartesian = -(Robot.y_target_pygame - Robot.starting_y_pygame)

                # ball_path(Robot, x_ball_target_cartesian, y_ball_target_cartesian)

        if Robot.balls_collected >= 3 :  
            Robot.x_target_cartesian = 10 
            Robot.y_target_cartesian = 400
            Robot.x_target_pygame = Robot.x_target_cartesian + Robot.starting_x_pygame
            Robot.y_target_pygame = - Robot.y_target_cartesian + Robot.starting_y_pygame

            MOVE_TO_BOX = 1
            MOVING = 1
            
            TURNING_TARGET = 1
            MOVING_TARGET = 0

        localisation(Robot)
        draw_window(Robot)
        sleep(0.01)
except KeyboardInterrupt:
    kit.servo[4].angle = 140
    kit.servo[15].angle = 0
    GPIO.cleanup()

