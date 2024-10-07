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

ORIGIN = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'Origin.png')), (10, 10))

GOING_BACK = 0
TURNING_BACK = 0
MOVING_BACK = 0

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

        self.mm_per_tick = 4.13                                 # Nathan and Bryan checked this, measure again if unsure
        self.ticks_per_full_rotation = 300                              # TODO : Change this after wheel calibration
        self.degrees_per_tick = 360 / self.ticks_per_full_rotation      

        self.distance_per_iter = 2                          # TODO : Used only for demo 1 (Only 1n approx)
        self.deg_per_iter = 2

        # VISUALISATION
        self.width = 19
        self.height = 23
        self.image = pygame.image.load(os.path.join('PNGs', 'spaceship_red.png'))
        self.blit = pygame.transform.rotate(pygame.transform.scale(self.image, (self.width, self.height)), 180)
        self.rect = pygame.Rect(700, 300, self.width, self.height)


def update_keyboard(robot):
    for event in pygame.event.get():
        if event.type == pygame.quit : 
            break

        if event.type == pygame.KEYDOWN: 
            if event.key == pygame.K_UP :
                print("UP")
                robot.y_pygame -= np.cos(np.deg2rad(robot.deg)) * robot.distance_per_iter
                robot.x_pygame += np.sin(np.deg2rad(robot.deg)) * robot.distance_per_iter

            if event.key == pygame.K_DOWN : 
                print("DOWN")
                robot.y_pygame += np.cos(np.deg2rad(robot.deg)) * robot.distance_per_iter
                robot.x_pygame -= np.sin(np.deg2rad(robot.deg)) * robot.distance_per_iter

            if event.key == pygame.K_LEFT : 
                print("LEFT")
                robot.deg -= 1
                

            if event.key == pygame.K_RIGHT : 
                print("RIGHT")
                robot.deg += 1

            if event.key == pygame.K_g :
                print("Going back")
                return 1

            if event.key == pygame.K_q : 
                print("Quiting")
                break

    if robot.deg < 0 : 
        robot.deg = 360 - robot.deg

    elif robot.deg > 360 :
        robot.deg = robot.deg - 360

    return 

def turning_back(robot) : 
    threshold = 2
    print("Turning to origin")
    distance_x = robot.x - robot.starting_x
    distance_y = -(robot.y - robot.starting_y)

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
            robot.deg -= robot.deg_per_iter

        else : 
            robot.deg += robot.deg_per_iter

        return 0

    else : 
        print("FACING CENTER")
        print(f"ideal_degree:{ideal_degree}")
        return 1
    
def moving_back(robot) : 
    print("MOVING BACK")
    distance_x = abs(robot.x_pygame - robot.starting_x_pygame)
    distance_y = abs(robot.y_pygame - robot.starting_y_pygame)

    distance_overall = np.sqrt(distance_x**2 + distance_y**2)

    if distance_overall > 2 : 
        robot.y -= np.cos(np.deg2rad(robot.deg)) * robot.distance_per_iter
        robot.x += np.sin(np.deg2rad(robot.deg)) * robot.distance_per_iter
        return 0

    else : 
        print("reached origin")
        return 1


def draw_window(robot):
    WIN.blit(WHITE, (0, 0))
    WIN.blit(BLUE, (100,50))
    WIN.blit(ORIGIN, (robot.starting_x_pygame+(robot.width/2), robot.starting_y_pygame+(robot.height/2)))
    robot.blit = pygame.transform.rotate(pygame.transform.scale(robot.image, (robot.width, robot.height)), -robot.deg+180)
    WIN.blit(robot.blit, (robot.x_pygame, robot.y_pygame))

    robot.x_cartesian = robot.x_pygame - robot.starting_x_pygame
    robot.y_cartesian = -(robot.y_pygame - robot.starting_y_pygame)

    # Fonts
    pygame.font.init()
    my_font = pygame.font.SysFont('Comic Sans MS', 30)
    location_title_txt = my_font.render("Position", False, (0, 0, 0))
    location_txt = my_font.render(f'({np.round((robot.x_cartesian),2)},{np.round(robot.y_cartesian,2)})', False, (0, 0, 0))
    WIN.blit(location_title_txt, (658,0))
    WIN.blit(location_txt, (68,40))

    degrees_title_txt = my_font.render("Degrees : ", False, (0, 0, 0))
    degrees_txt = my_font.render(f'{np.round(robot.deg,2)}', False, (0, 0, 0))
    WIN.blit(degrees_title_txt, (658,80))
    WIN.blit(degrees_txt, (678,120))

    pygame.display.update()
    

# Start
FPS = 60
Robot = robot()

while(True):
    if (update_keyboard(Robot)) : 
        GOING_BACK = 1
        TURNING_BACK = 1
    
    if GOING_BACK == 1 : 
        if TURNING_BACK == 1 : 
            if (turning_back(Robot)) : 
                TURNING_BACK = 0
                MOVING_BACK = 1

        if MOVING_BACK == 1 : 
            if (moving_back(Robot)) : 
                GOING_BACK = 0
                MOVING_BACK = 0

    draw_window(Robot)
    sleep(0.1)
