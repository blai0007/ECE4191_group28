from time import sleep
import pygame
import os
import numpy as np
import math
#from Encoder import Encoder

# NOTE : #
# The robot is facing upwards"


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

class robot : 
    def __init__(self) : 
        self.ticks_left = 0
        self.ticks_right = 0

        self.ticks_left_prev = 0
        self.ticks_right_prev = 0

        self.x_pygame = 614               #400
        self.y_pygame = 411           #200
        self.starting_x_pygame = 100       #400
        self.starting_y_pygame = 418      #200
        self.x_cartesian = self.x_pygame - self.starting_x_pygame
        self.y_cartesian = -(self.y_pygame - self.starting_y_pygame)
        self.deg = 0

        self.cm_per_tick = 60 / 3300                                  # Nathan and Bryan checked this, measure again if unsure
        self.ticks_per_full_rotation = 3900                            # TODO : Change this after wheel calibration
        self.degrees_per_tick = 360 / self.ticks_per_full_rotation
        self.degrees_per_tick_wheel = 360 / 900     

        self.distance_per_iter = 2                          # TODO : Used only for demo 1 (Only 1n approx)
        self.deg_per_iter = 2
        self.dt = 0.002
        self.loop_dt = 0.001

        self.x_deposit_cartesian = 0

        self.x_target_pygame = 0
        self.y_target_pygame = 0
        self.x_target_cartesian = 0
        self.y_target_cartesian = 0

        # self.search_pattern = [(480,100), (480,200), (470,300), (300,300), (200,300), (100,300), (100,200), (100,100), (200,100), (300,100), (350,100)]
        # self.search_pattern = [(480,100), (400,200), (370,250), (300,250), (200,250), (100,250), (100,170), (100,100), (200,100), (300,100), (350,100)]
        self.search_pattern = [(460,60),(400,200),(400,275),(343,60),(251,275),(195,60),(112,275),(54,60),(112,275),(195,60),(251,275),(343,60),(400,275),(460,60)]
        self.ball_target_pattern = []
        self.ball_target_pattern_iter = 0
        self.search_pattern_iter = 0

        self.balls_collected = 0

        self.distance_moved = 0
        self.distance_moved_prev = 0

        # VISUALISATION
        self.width = 26
        self.height = 56
        self.wheel_seperation = self.width / 2 - 5
        self.wheel_radius = 5.39                          # 5.93 cm 
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

class box() : 
    def __init__(self) : 
        self.box_width = 60
        self.box_height = 45

        self.x_box_wall_cartesian = 30
        self.y_box_wall_cartesian = 370

        self.x_box_cartesian = 0
        self.y_box_cartesian = 411
        self.x_deposit_cartesian = 100
        self.y_deposit_cartesian = 380

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
    MOVING = 1

    return 1

def turn_to_reverse(robot) :
    threshold = 5
    ideal_degree = 90

    if (robot.deg < (ideal_degree-threshold)) or (robot.deg > (ideal_degree+threshold)):           # Not facing centre
        print("ENTERED TURN TO REVERSE")
        if robot.deg > ideal_degree : 
            robot.ticks_left -= 2
            robot.ticks_right += 1
            # robot.deg -= robot.deg_per_iter

        else : 
            # robot.deg += robot.deg_per_iter
            robot.ticks_left += 2
            robot.ticks_right -= 1

        return 0
    else : 
        print("DONE TURNING FOR REVERSE")
        return 1
    
def move_to_reverse(robot, box) : 
    distance_x = robot.x_cartesian - (box.x_box_wall_cartesian)
    distance_y = robot.y_cartesian - (box.y_box_wall_cartesian)

    distance = np.sqrt(distance_x**2 + distance_y**2)

    if distance > 5 :
        print(f"Distance : {distance}")
        robot.ticks_left -= 2
        robot.ticks_right -= 2
        return 0

    else :
        print("DEPOSITED BALLS")
        return 1


def turn_to_target(robot) : 
    threshold = 5
    print(f"Turning to Target : {robot.x_target_pygame, robot.y_target_pygame}")
    distance_x = -(robot.x_pygame - robot.x_target_pygame)
    distance_y = (robot.y_pygame - robot.y_target_pygame)
    print(f"distance_x : {distance_x}")
    print(f"distance_y : {distance_y}")

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

    print(f"Ideal Degree : {ideal_degree}")
    ideal_degree_lower = ideal_degree-threshold

    if (robot.deg < (ideal_degree-threshold)) or (robot.deg > (ideal_degree+threshold)):           # Not facing centre
        if (robot.deg > ideal_degree and abs(ideal_degree-robot.deg)<180 )or (robot.deg < ideal_degree and abs(ideal_degree-robot.deg)>180): 
            robot.ticks_left -= 1
            robot.ticks_right += 2
            # robot.deg -= robot.deg_per_iter

        else : 
            # robot.deg += robot.deg_per_iter
            robot.ticks_left += 1
            robot.ticks_right -= 2

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

    if distance_overall > 40 : 
        # robot.forward()
        robot.ticks_left += 1
        robot.ticks_right += 2
        # robot.y_pygame -= np.cos(np.deg2rad(robot.deg)) * robot.distance_per_iter
        # robot.x_pygame += np.sin(np.deg2rad(robot.deg)) * robot.distance_per_iter
        return 0

    else : 
        print("TARGET Reached")
        return 1
    
def localisation(robot) : 
    distance_moved = 0
    degrees_turned = 0

    left_ticks_iter = robot.ticks_left-robot.ticks_left_prev
    right_ticks_iter = robot.ticks_right-robot.ticks_right_prev
    w_left = (left_ticks_iter / robot.dt) * robot.degrees_per_tick_wheel
    w_right = (right_ticks_iter / robot.dt) * robot.degrees_per_tick_wheel
    v_left = w_left*robot.wheel_radius
    v_right = w_right*robot.wheel_radius

    v = (w_left*robot.wheel_radius + w_right*robot.wheel_radius)/2
    w = abs(w_left*robot.wheel_radius - w_right*robot.wheel_radius)/robot.wheel_seperation

    # MOVE FORWARDS
    if (robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its Forwards")
        if (robot.ticks_left-robot.ticks_left_prev) < (robot.ticks_right - robot.ticks_right_prev) :   
            print("Titling Leftwards")

            robot.y_pygame -= v*np.cos(np.deg2rad(robot.deg))*robot.dt
            robot.x_pygame += v*np.sin(np.deg2rad(robot.deg))*robot.dt
            robot.deg -= w*robot.dt

        elif (robot.ticks_left-robot.ticks_left_prev) > (robot.ticks_right - robot.ticks_right_prev ) : 
            print("Titling Rightwards")

            robot.y_pygame -= v*np.cos(np.deg2rad(robot.deg))*robot.dt
            robot.x_pygame += v*np.sin(np.deg2rad(robot.deg))*robot.dt
            robot.deg = robot.deg + w*robot.dt

        else : 
            robot.y_pygame -= v*np.cos(np.deg2rad(robot.deg)) * robot.dt
            robot.x_pygame += v*np.sin(np.deg2rad(robot.deg)) * robot.dt

    # BACKWARDS
    elif (robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        print("Its backwards")
        if (robot.ticks_left-robot.ticks_left_prev) < (robot.ticks_right - robot.ticks_right_prev) :   
            print("Titling Leftwards")

            robot.y_pygame += v*np.cos(np.deg2rad(robot.deg))*robot.dt
            robot.x_pygame -= v*np.sin(np.deg2rad(robot.deg))*robot.dt
            robot.deg -= w*robot.dt

        elif (robot.ticks_left-robot.ticks_left_prev) > (robot.ticks_right - robot.ticks_right_prev ) : 
            print("Titling Rightwards")

            robot.y_pygame += v*np.cos(np.deg2rad(robot.deg))*robot.dt
            robot.x_pygame -= v*np.sin(np.deg2rad(robot.deg))*robot.dt
            robot.deg = robot.deg + w*robot.dt

        else : 
            print("BACKWARDS NO TILT")
            robot.y_pygame += v*np.cos(np.deg2rad(robot.deg)) * robot.dt
            robot.x_pygame -= v*np.sin(np.deg2rad(robot.deg)) * robot.dt
        

    # MOVE LEFT
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its MOVING LEFT")
        degrees_turned = w*robot.dt   #(abs(right_ticks_iter)+abs(left_ticks_iter)) *  robot.degrees_per_tick  /2        # - left_ticks_iter
        print(f"Deg turned : {degrees_turned}")
        robot.deg -= degrees_turned

    # # MOVE RIGHT
    if ( robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        degrees_turned = w*robot.dt     #(-(right_ticks_iter+right_ticks_iter)) * robot.degrees_per_tick    / 2     # + left_ticks_iter
        print(f"Deg turned : {degrees_turned}")
        print("Its MOVING RIGHT")
        robot.deg += degrees_turned

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
    WIN.blit(ORIGIN, (robot.x_target_pygame, robot.y_target_pygame))
    WIN.blit(ORIGIN, (618,418))
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

    ball_found_title_txt = my_font.render("TURNING ? :", False, (0, 0, 0))
    if TURNING_TARGET == 1: 
        ball_found_txt = my_font.render("YES", False, (0, 0, 0))
    else: 
        ball_found_txt = my_font.render("NO", False, (0, 0, 0))
    WIN.blit(ball_found_title_txt, (658,360))
    WIN.blit(ball_found_txt, (800,360))


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
Box = box()
count = 0

while(True):
    print("############################")
    if not MOVING and not BALL_FOUND and not MOVE_TO_BOX: 
        find_location(Robot)
        MOVING = 1
        TURNING_TARGET = 1

    if MOVE_TO_BOX == 1 : 
        if TURNING_TARGET == 1 :                    # MOVING=1, MOVE_TO_BOX=1,  
            if (turn_to_target(Robot))==1 : 
                TURNING_TARGET = 0
                MOVING_TARGET = 1

        elif MOVING_TARGET == 1 :
            FLAG_TARGET = moving_to_target(Robot)
            if (FLAG_TARGET)==1 :
                print("REACHED_TO_MIDDLE_POINT") 
                TURN_TO_REVERSE = 1
                MOVING_TARGET = 0

            elif (FLAG_TARGET) == 0 :
                print("MOVE_TO_MIDDLE_POINT")
                TURNING_TARGET = 1 
                MOVING_TARGET = 0  
                
        elif TURN_TO_REVERSE == 1 : 
            if (turn_to_reverse(Robot)) : 
                MOVING_TARGET = 0
                TURNING_TARGET = 0
                MOVE_TO_REVERSE = 1
                TURN_TO_REVERSE = 0
                print(f"MOVE_TO_REVERSE : {MOVE_TO_REVERSE}")
                print(f"TURN_TO_REVERSE : {TURN_TO_REVERSE}")

        elif MOVE_TO_REVERSE == 1 : 
            if (move_to_reverse(Robot,Box)) : 
                Robot.balls_collected = 0
                if MOVE_TO_BOX == 1 :
                    print("BOX_REACHED")
                    MOVE_TO_REVERSE = 0
                    MOVE_TO_BOX = 0
                    MOVING = 0
                    BALL_FOUND = 0
                    MOVING_TARGET = 0 


    if BALL_FOUND == 1 :
        if TURNING_TARGET == 1 : 
            if (turn_to_target(Robot)) : 
                TURNING_TARGET = 0
                MOVING_TARGET = 1

        if MOVING_TARGET == 1 :  
            FLAG_TARGET = moving_to_target(Robot)
            if (FLAG_TARGET)==1 : 
                MOVING = 0
                MOVING_TARGET = 0
                if BALL_FOUND == 1 : 
                    print("BALL REACHED")
                    Robot.balls_collected += 1
                    BALL_FOUND = 0

            elif (FLAG_TARGET) == 0 :
                print("BALL FOUND") 
                TURNING_TARGET = 1
                MOVING = 0
                MOVING_TARGET = 0 

    elif BALL_FOUND == 0 : 
        if TURNING_TARGET == 1 : 
            if (turn_to_target(Robot)) : 
                TURNING_TARGET = 0
                MOVING_TARGET = 1

        if MOVING_TARGET == 1 : 
            FLAG_TARGET = moving_to_target(Robot)
            if (FLAG_TARGET)==1 : 
                MOVING = 0
                MOVING_TARGET = 0
            elif (FLAG_TARGET) == 0 : 
                MOVING_TARGET = 0
                TURNING_TARGET = 1
            
        
    for event in pygame.event.get():
        if event.type == pygame.quit : 
            break

        if event.type == pygame.MOUSEBUTTONDOWN :
            # (Robot.x_target_pygame, Robot.y_target_pygame) = pygame.mouse.get_pos()
            (x_ball_target_pygame, y_ball_target_pygame) = pygame.mouse.get_pos()
            x_ball_target_cartesian = x_ball_target_pygame - Robot.starting_x_pygame
            y_ball_target_cartesian = -(y_ball_target_pygame - Robot.starting_y_pygame)


            # Robot.y_target_pygame = - Robot.y_target_pygame
            if ((x_ball_target_cartesian > 0) and (x_ball_target_cartesian < 548)) and ((y_ball_target_cartesian > 0) and (y_ball_target_cartesian < 370)): 
                BALL_FOUND = 1
                MOVING = 1
                TURNING_TARGET = 1
                MOVING_TARGET = 0
                Robot.x_target_pygame = x_ball_target_pygame
                Robot.y_target_pygame = y_ball_target_pygame
                Robot.x_target_cartesian = Robot.x_target_pygame - Robot.starting_x_pygame
                Robot.y_target_cartesian = -(Robot.y_target_pygame - Robot.starting_y_pygame)           # Robot.starting_y_pygame


            # ball_path(Robot, x_ball_target_cartesian, y_ball_target_cartesian)

    if Robot.balls_collected >= 1 and MOVE_TO_BOX == 0:  
        Robot.x_target_cartesian = Box.x_deposit_cartesian
        Robot.y_target_cartesian = Box.y_deposit_cartesian
        Robot.x_target_pygame = Robot.x_target_cartesian + Robot.starting_x_pygame
        Robot.y_target_pygame = - Robot.y_target_cartesian + Robot.starting_y_pygame           # Robot.starting_y_pygame

        print(f"NEW WAYPOINT : ({Robot.x_target_pygame},{Robot.y_target_pygame})")
        if MOVE_TO_BOX == 0 : 
            MOVE_TO_BOX = 1
            MOVING = 1
            
            TURNING_TARGET = 1
            MOVING_TARGET = 0

    if ((Robot.x_cartesian < 0) or (Robot.x_cartesian > 518)) or ((Robot.y_cartesian < 0) or (Robot.y_cartesian > 400)): 
        MOVING = 1
        BALL_FOUND = 0
        TURNING_TARGET = 1
        MOVING_TARGET = 0

        if (Robot.x_cartesian > 200 and Robot.y_cartesian > 200) : 
            Robot.x_target_cartesian = 300
            Robot.y_target_cartesian = 300

        elif (Robot.x_cartesian > 200 and Robot.y_cartesian < 200) : 
            Robot.x_target_cartesian = 300
            Robot.y_target_cartesian = 100

        elif (Robot.x_cartesian < 200 and Robot.y_cartesian < 200) : 
            Robot.x_target_cartesian = 100
            Robot.y_target_cartesian = 100

        elif (Robot.x_cartesian < 200 and Robot.y_cartesian > 200) : 
            Robot.x_target_cartesian = 100
            Robot.y_target_cartesian = 300

        Robot.x_target_pygame = Robot.x_target_cartesian + Robot.starting_x_pygame
        Robot.y_target_pygame = - Robot.y_target_cartesian + Robot.starting_y_pygame

    localisation(Robot)
    draw_window(Robot)
    sleep(Robot.dt)
    sleep(Robot.loop_dt)
    count+=1
    print(f"COUNT : {count}")
