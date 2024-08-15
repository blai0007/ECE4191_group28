import pygame
#from gpiozero import Robot

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