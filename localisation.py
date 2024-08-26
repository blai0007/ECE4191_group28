from time import sleep
import pygame
import os
import numpy as np
#from Encoder import Encoder

# NOTE : #
# The robot is facing upwards"

# Initialise Pygame Module
pygame.init()
SCREEN_WIDTH =  900
SCREEN_HEIGHT = 500

WIN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("LOCALISATION")

WHITE = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'white.png')), (SCREEN_WIDTH, SCREEN_HEIGHT))

ORIGIN = pygame.transform.scale(pygame.image.load(
    os.path.join('PNGs', 'Origin.png')), (10, 10))

class robot : 
    def __init__(self) : 
        self.ticks_left = 0
        self.ticks_right = 0

        self.ticks_left_prev = 0
        self.ticks_right_prev = 0

        self.x = 400
        self.y = 200
        self.starting_x = 400
        self.starting_y = 200
        self.deg = 0

        self.mm_per_tick = 4.13                                 # Nathan and Bryan checked this, measure again if unsure
        self.ticks_per_full_rotation = 300                              # TODO : Change this after wheel calibration
        self.degrees_per_tick = 360 / self.ticks_per_full_rotation      

        # VISUALISATION
        self.width = 55
        self.height = 40
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
                robot.ticks_right += 1
                robot.ticks_left += 1

            if event.key == pygame.K_DOWN : 
                print("DOWN")
                robot.ticks_right -= 1
                robot.ticks_left -= 1

            if event.key == pygame.K_LEFT : 
                robot.ticks_right += 1
                robot.ticks_left -= 1
                print("LEFT")
                #drive_left()

            if event.key == pygame.K_RIGHT : 
                robot.ticks_right -= 1
                robot.ticks_left += 1
                print("RIGHT")
                #drive_right()

            if event.key == pygame.K_q : 
                print("Quiting")
                break

    return 

def localisation(robot) : 
    distance_moved = 0
    degrees_turned = 0

    # MOVE FORWARDS
    if (robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its Forwards")
        distance_moved = (robot.ticks_left - robot.ticks_left_prev) * 4.13
        
        

    # MOVE BACKWARDS
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        print("Its Backwards")
        distance_moved = (robot.ticks_left - robot.ticks_left_prev) * 4.13

    # MOVE LEFT
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its MOVING LEFT")
        degrees_turned = (robot.ticks_left - robot.ticks_left_prev) * robot.degrees_per_tick
        print(f"Deg turned : {degrees_turned}")
        #deg_turned = rotation_calib 

    # MOVE RIGHT
    if ( robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        degrees_turned = -(robot.ticks_left_prev - robot.ticks_left) * robot.degrees_per_tick
        print(f"Deg turned : {degrees_turned}")
        print("Its MOVING RIGHT")

    robot.y -= np.cos(np.deg2rad(robot.deg)) * distance_moved
    robot.x += np.sin(np.deg2rad(robot.deg)) * distance_moved
    robot.deg += degrees_turned

    print(f"Moving in x :  {np.sin(np.deg2rad(degrees_turned)) * distance_moved}")

    if robot.deg < 0 : 
        robot.deg = 360 - robot.deg

    elif robot.deg > 360 :
        robot.deg = robot.deg - 360


    print(np.sin(degrees_turned) * distance_moved)
    return

def draw_window(robot):
    WIN.blit(WHITE, (0, 0))
    WIN.blit(ORIGIN, (robot.starting_x+20, robot.starting_y+20))
    robot.blit = pygame.transform.rotate(pygame.transform.scale(robot.image, (robot.width, robot.height)), -robot.deg+180)
    WIN.blit(robot.blit, (robot.x, robot.y))

    pygame.font.init()
    my_font = pygame.font.SysFont('Comic Sans MS', 30)
    location_txt = my_font.render(f'({np.round((robot.x- robot.starting_x),2)},{np.round((robot.y-robot.starting_y),2)})', False, (0, 0, 0))
    WIN.blit(location_txt, (0,0))
    degrees_txt = my_font.render(f'Deg {np.round(robot.deg,2)}', False, (0, 0, 0))
    WIN.blit(degrees_txt, (200,0))

    
    pygame.display.update()
    
        
# Start
FPS = 60
Robot = robot()

while(True):
    update_keyboard(Robot)
    localisation(Robot)
    Robot.ticks_left_prev = Robot.ticks_left
    Robot.ticks_right_prev = Robot.ticks_right
    print(f"X : {Robot.x}")
    print(f"Y : {Robot.y}")
    print(f"DEG : {Robot.deg}")

    draw_window(Robot)
    sleep(0.1)
