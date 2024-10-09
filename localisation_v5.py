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

in1_left_belt = 17
in2_left_belt = 27

in1_right_belt = 22
in2_right_belt = 10

# ROTARY ENCODER INIT
e1 = RotaryEncoder(encoder1_left_pin, encoder1_right_pin, max_steps = 100000000)
e2 = RotaryEncoder(encoder2_left_pin, encoder2_right_pin, max_steps = 100000000)

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
    os.path.join('PNGs', 'Brown.png')), (30, 50))

ORIGIN = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'Origin.png')), (10, 10))

# GLOBAL VARIABLES - FLAGS
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
FLAG_TARGET = 0

# robot Class
class robot : 
    def __init__(self) : 
        # PYGAME VARIABLES
        self.x_pygame = 100               #400
        self.y_pygame = 418           #200
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
        self.degrees_per_tick_wheel = 360 / 40000    #900     

        # WAITING TIME (DT)
        self.drive_dt = 0.2
        self.turning_dt = 0.2
        self.loop_dt = 0.001

        # SEARCH PATTERN
        self.search_pattern = [(50,100), (100,200), (200, 200), (300, 200), (400,200), (300,200), (200, 200)]
        self.ball_target_pattern = []
        self.ball_target_pattern_iter = 0
        self.search_pattern_iter = 0

        # BALL COUNT
        self.balls_collected = 0

        # VISUALISATION (PYGAME SPRITE)
        self.width = 26
        self.height = 54
        self.wheel_seperation = self.width - 2
        self.wheel_radius = 5.39 
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

        # THRESHOLDS
        self.turning_threshold = 5
        self.moving_threshold = 30

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

        # Update ticks in robot class
        robot.ticks_left = e1.steps
        robot.ticks_right = e2.steps
        return 0
    else : 
        print("FINISHED TURNING : READY TO REVERSE")
        return 1
    
def move_to_reverse(robot) : 
    distance_x = robot.x_cartesian - robot.x_deposit_cartesian

    if distance_x > 10 :
        robot.ticks_left -= 20
        robot.ticks_right -= 20
        return 0

    else :
        return 1

# TURNING & MOVING TO TARGET
def turn_to_target(robot, e1, e2) : 
    # print(f"Turning to Target : {robot.x_target_pygame, robot.y_target_pygame}")
    distance_x = -(robot.x_pygame - robot.x_target_pygame)
    distance_y = (robot.y_pygame - robot.y_target_pygame)
    ideal_degree = 0

    if (distance_x > 0 ) and (distance_y > 0) : 
        ideal_degree = 90 - math.degrees(math.atan(abs(distance_y)/ abs(distance_x)))
        # ideal_degree = 270 - math.degrees(math.atan(abs(distance_y/distance_x)))
        print("Quad 1")

    elif (distance_x < 0 ) and (distance_y > 0) : 
        ideal_degree = 270 + math.degrees(math.atan(abs(distance_y)/ abs(distance_x)))
        print("Quad 2")

    elif (distance_x < 0 ) and (distance_y < 0) : 
        ideal_degree = 270 - math.degrees(math.atan(abs(distance_y)/abs(distance_x)))
        print("Quad 3")

    elif (distance_x > 0 ) and (distance_y < 0) : 
        ideal_degree = 90 + math.degrees(math.atan(abs(distance_y)/abs(distance_x)))
        print("Quad 4")
    

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

        # Update ticks in robot class
        robot.ticks_left = e1.steps
        robot.ticks_right = e2.steps
        return 0

    else : 
        print(f"FACING TARGET")
        return 1
    
def moving_to_target(robot, e1, e2) : 
    print("MOVING TO TARGET")
    distance_x = robot.x_cartesian - robot.x_target_cartesian
    distance_y = -(robot.y_cartesian - robot.y_target_cartesian)

    distance_overall = np.sqrt(distance_x**2 + distance_y**2)
    print(f"distance : {distance_overall}")

    if distance_overall > robot.moving_threshold : 
        m1_speed = 80#max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.left_ticks_iter, robot.drive_dt)))
        m2_speed = 80#max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, robot.right_ticks_iter, robot.drive_dt)))

        set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
        set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)

        sleep(robot.drive_dt)
        drive_stop()
        robot.ticks_left = e1.steps
        robot.ticks_right = e2.steps

        return 0

    else : 
        print("TARGET Reached")
        drive_stop()
        return 1
    
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
        w_left = (robot.left_ticks_iter / robot.drive_dt) * robot.degrees_per_tick_wheel
        w_right = (robot.right_ticks_iter / robot.drive_dt) * robot.degrees_per_tick_wheel

        # Determing wheel linear velocity (LEFT & RIGHT)
        v_left = w_left*robot.wheel_radius
        v_right = w_right*robot.wheel_radius

        # Determing whole robot's angular and linear velocity
        v = (w_left*robot.wheel_radius + w_right*robot.wheel_radius)/2
        w = abs(w_left*robot.wheel_radius - w_right*robot.wheel_radius)/robot.wheel_seperation

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
        w_left = (robot.left_ticks_iter / robot.turning_dt) * robot.degrees_per_tick_wheel
        w_right = (robot.right_ticks_iter / robot.turning_dt) * robot.degrees_per_tick_wheel

        # Determing wheel linear velocity (LEFT & RIGHT)
        v_left = w_left*robot.wheel_radius
        v_right = w_right*robot.wheel_radius

        # Determing whole robot's angular and linear velocity
        v = (w_left*robot.wheel_radius + w_right*robot.wheel_radius)/2
        w = abs(w_left*robot.wheel_radius - w_right*robot.wheel_radius)/robot.wheel_seperation
        degrees_turned = w*robot.turning_dt  
        print("PYGAME ACKNOWLEDGE :  IT IS ROTATING LEFT")
        print(f"Deg turned : {degrees_turned}")
        robot.deg -= degrees_turned

    # ROBOT IS ROTATING RIGHT
    elif ( robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        # Determing wheel angular velocity (LEFT & RIGHT)
        w_left = (robot.left_ticks_iter / robot.turning_dt) * robot.degrees_per_tick_wheel
        w_right = (robot.right_ticks_iter / robot.turning_dt) * robot.degrees_per_tick_wheel

        # Determing wheel linear velocity (LEFT & RIGHT)
        v_left = w_left*robot.wheel_radius
        v_right = w_right*robot.wheel_radius

        # Determing whole robot's angular and linear velocity
        v = (w_left*robot.wheel_radius + w_right*robot.wheel_radius)/2
        w = abs(w_left*robot.wheel_radius - w_right*robot.wheel_radius)/robot.wheel_seperation
        degrees_turned = w*robot.turning_dt   
        print("PYGAME ACKNOWLEDGE :  IT IS ROTATING RIGHT")  
        print(f"Deg turned : {degrees_turned}")
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

# START PROGRAM
FPS = 60
Robot = robot()
# pi_controller = PIController(Kp=10, Ki=0.2)

# INITIALISING SERVOS
kit = ServoKit(channels=16)
kit.servo[4].angle = 0
kit.servo[15].angle = 140

# START LOOP
try:
    left_belt_speed = 80
    right_belt_speed = 80
    # set_motor(in1_left_belt, in2_left_belt, motor_num=2, direction=1, speed=left_belt_speed)
    # set_motor(in1_right_belt, in2_right_belt, motor_num=3, direction=1, speed=right_belt_speed)
    while(True):
        print("###########################################################")
        # TODO  : Setpoint
        expected_rpm = 75 # EXPECTED SPEED OF MOTOR 0-100
        if MOVING:
            expected_tick_per_sec = expected_rpm * (900/60)
        else:
            expected_tick_per_sec = 0
        
        # 
        if not MOVING and not BALL_FOUND and not MOVE_TO_BOX: 
            find_location(Robot)
            MOVING = 1
            TURNING_TARGET = 1

        # MOVING TO BOX SUBFUNCTION
        elif MOVE_TO_BOX == 1 : 
            if TURNING_TARGET == 1 : 
                if (turn_to_target(Robot, e1, e2)) : 
                    TURNING_TARGET = 0
                    MOVING_TARGET = 1

            if MOVING_TARGET == 1 : 
                if (moving_to_target(Robot, e1, e2)) : 
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
                    kit.servo[5].angle = 20
                    sleep(5)
                    kit.servo[5].angle = 180
                    if MOVE_TO_BOX == 1 :
                        MOVE_TO_BOX = 0
                        MOVING = 0
                        BALL_FOUND = 0
                        MOVING_TARGET = 0 

        # MOVING TO BALL SUBFUNCTION
        elif BALL_FOUND == 1 :
            if TURNING_TARGET == 1 : 
                if (turn_to_target(Robot, e1, e2)) : 
                    TURNING_TARGET = 0
                    MOVING_TARGET = 1

            if MOVING_TARGET == 1 : 
                if (moving_to_target(Robot, e1, e2)) : 
                    FLAG_TARGET = moving_to_target(Robot, e1, e2)
                    if (FLAG_TARGET)==1 : 
                        MOVING = 0
                        MOVING_TARGET = 0
                        
                    elif (FLAG_TARGET) == 0 : 
                        MOVING_TARGET = 0
                        TURNING_TARGET = 1

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
        
        # TO SIMULATE BALL FINDING
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
        
        # CHECKS THE BALLS (BALL COUNT)
        if Robot.balls_collected >= 3 :  
            # Set target as the BOX
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
        print(f"LEFT_TICKS_ITER : {Robot.left_ticks_iter}")
        print(f"RIGHT_TICKS_ITER : {Robot.right_ticks_iter}")

        sleep(Robot.loop_dt)

except KeyboardInterrupt:
    kit.servo[4].angle = 140
    kit.servo[15].angle = 0
    GPIO.cleanup()

