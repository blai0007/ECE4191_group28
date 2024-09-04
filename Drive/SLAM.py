import pygame
import os
import numpy as np

class SLAM:
    def __init__(self, robot, SCREEN_WIDTH =  900, SCREEN_HEIGHT = 500):
        self.sw = SCREEN_WIDTH
        self.sh = SCREEN_HEIGHT
        self.robot = robot

    def initialise(self):
        pygame.init()
        WIN = pygame.display.set_mode((self.sw, self.sh))
        pygame.display.set_caption("LOCALISATION")

        WHITE = pygame.transform.scale(pygame.image.load(
        os.path.join('PNGs', 'white.png')), (self.sw, self.sh))

        ORIGIN = pygame.transform.scale(pygame.image.load(
        os.path.join('PNGs', 'Origin.png')), (10, 10))

        BLUE = pygame.transform.scale(pygame.image.load(
        os.path.join('PNGs', 'blue.png')), (548, 411))

        return WIN, WHITE, ORIGIN, BLUE

    def draw_window(self, WIN, WHITE, ORIGIN, BLUE):
        WIN.blit(WHITE, (0, 0))
        WIN.blit(ORIGIN, (self.robot.starting_x+20, self.robot.starting_y+20))
        WIN.blit(BLUE, (100,50))
        self.robot.blit = pygame.transform.rotate(pygame.transform.scale(self.robot.image, (self.robot.width, self.robot.height)), -self.robot.deg+180)
        WIN.blit(self.robot.blit, (self.robot.x, self.robot.y))

        self.robot.x_cartesian = self.robot.x - self.robot.starting_x
        self.robot.y_cartesian = self.robot.y - self.robot.starting_y

        pygame.font.init()
        my_font = pygame.font.SysFont('Comic Sans MS', 30)
        location_txt = my_font.render(f'({np.round((self.robot.x_cartesian),2)},{np.round((-(self.robot.y_cartesian)),2)})', False, (0, 0, 0))
        WIN.blit(location_txt, (0,0))
        degrees_txt = my_font.render(f'Deg {np.round(self.robot.deg,2)}', False, (0, 0, 0))
        WIN.blit(degrees_txt, (0,50))

        E1_txt = my_font.render(f'E1 : {np.round(self.robot.ticks_left,2)}', False, (0, 0, 0))
        WIN.blit(E1_txt, (0,200))

        E2_txt = my_font.render(f'E2: {np.round(self.robot.ticks_right,2)}', False, (0, 0, 0))
        WIN.blit(E2_txt, (0,220))

        pygame.display.update()
