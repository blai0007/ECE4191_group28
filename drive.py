import pygame
# from Encoder.py import Encoder
from gpiozero import Robot
import time
# en1 = Encoder(8,10)
# en2 = Encoder(22,24)

pygame.init()
SCREEN_WIDTH =  600
SCREEN_HEIGHT = 400

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

# en1.get
# Drive function
def update_keyboard(robot):
    for event in pygame.event.get():
        if event.type == pygame.quit : 
            break

        if event.type == pygame.KEYDOWN: 
            if event.key == pygame.K_UP :
                robot.forward() 
                print("Forwardss")

            if event.key == pygame.K_DOWN : 
                robot.backwards()
                print("BACKWARDS")

            if event.key == pygame.K_LEFT : 
                print("LEFT")

            if event.key == pygame.K_RIGHT : 
                print("RIGHT")

# def getDistance():
#     wheel_rad = 27 #mm
#     degPerTick = 360/40
#     en1_rotated = en1.getValue() * degPerTick # deg
#     en2_rotated = en2.getValue() * degPerTick # deg
#     en1_distance = wheel_rad * en1_rotated 
#     en2_distance = wheel_rad * en2_rotated
#     return en1_distance, en2_distance

robot_1 = Robot(left=(5,6), right=(19,26))    # change it to gpiozero
print("Let's Go!")

while (True):
    # update_keyboard(robot_1)
    robot_1.forward()
    time.sleep(0.1)