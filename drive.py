import pygame
import Encoder.py
from gpiozero import Robot
en1 = Encoder(8,10)
en2 = Encoder(22,24)

en1.get
# Drive function
def update_keyboard(robot):
    for event in pygame.event.get():
        #FORWARD
        if event.type == pygame.KEYUP and event.key == pygame.K_UP:
            #STOP DRIVING ON RELEASE
            robot.stop()
        if event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
            #DRIVE ON PRESS
            robot.forward()
        #LEFT
        if event.type == pygame.KEYUP and event.key == pygame.K_LEFT:
            robot.stop()
        if event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
            robot.left()
        #RIGHT
        if event.type == pygame.KEYUP and event.key == pygame.K_RIGHT:
            robot.stop()
        if event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
            robot.right()
        #BACKWARD
        if event.type == pygame.KEYUP and event.key == pygame.K_DOWN:
            robot.stop()
        if event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
            robot.reverse()
        #STOP DRIVING
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            robot.stop()

def getDistance():
    wheel_rad = 27 #mm
    degPerTick = 360/40
    en1_rotated = en1.getValue() * degPerTick # deg
    en2_rotated = en2.getValue() * degPerTick # deg
    en1_distance = wheel_rad * en1_rotated 
    en2_distance = wheel_rad * en2_rotated
    return en1_distance, en2_distance
