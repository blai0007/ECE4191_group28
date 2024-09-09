import RPi.GPIO as GPIO  
from Encoder import Encoder   
import pygame
import os

class Robot:
    def __init__(self, in1_l, in2_l, in1_r, in2_r, en_left, en_right, en1_l, en1_r, en2_l, en2_r):
        # Encoder ticks
        self.ticks_left = 0
        self.ticks_right = 0

        self.ticks_left_prev = 0
        self.ticks_right_prev = 0

        # PINS
        self.in1_l = in1_l
        self.in2_l = in2_l
        self.in1_r = in1_r
        self.in2_r = in2_r
        self.en_left = en_left
        self.en_right = en_right
        self.en1_l = en1_l
        self.en1_r = en1_r
        self.en2_l = en2_l
        self.en2_r = en2_r

        # Localisation variables
        self.x = 100                #400
        self.y = 461 -40            #200
        self.starting_x = 100       #400
        self.starting_y = 461 - 40  #200
        self.deg = 0

        self.m_per_tick = (1000 / 10400) / 10        #cm                    # Nathan and Bryan checked this, measure again if unsure
        self.ticks_per_full_rotation = 3596 # revs per 360*ticks per rev #3600 #1800 # 7500 #700                             # TODO : Change this after wheel calibration

        self.x_cartesian = self.x - self.starting_x
        self.y_cartesian = -(self.y - self.starting_y)  #Pygame views this as negative so consider
        self.degrees_per_tick = 360 / self.ticks_per_full_rotation     

        self.left_mag = 0
        self.right_mag = 0

        self.left_a = 0
        self.left_b = 0

        # Visualisation variables
        self.width = 55    
        self.height = 40
        self.image = pygame.image.load(os.path.join('PNGs', 'spaceship_red.png'))
        self.blit = pygame.transform.rotate(pygame.transform.scale(self.image, (self.width, self.height)), 180)
        self.rect = pygame.Rect(700, 300, self.width, self.height)

    def initialise(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1_l,GPIO.OUT)
        GPIO.setup(self.in2_l,GPIO.OUT)
        GPIO.setup(self.en_left,GPIO.OUT)

        GPIO.setup(self.in1_r,GPIO.OUT)
        GPIO.setup(self.in2_r,GPIO.OUT)
        GPIO.setup(self.en_right,GPIO.OUT)

        GPIO.output(self.in1_l,GPIO.LOW)
        GPIO.output(self.in2_l,GPIO.LOW)
        GPIO.output(self.in1_r,GPIO.LOW)
        GPIO.output(self.in2_r,GPIO.LOW)

        # Enabling motor drivers
        p_left=GPIO.PWM(self.en_left,1000)
        p_right=GPIO.PWM(self.en_right,1000)
        p_left.start(100)
        p_right.start(100)
        print("\n")
        print("The default speed & direction of motor is LOW & Forward.....")
        print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
        print("\n")  

        e1 = Encoder(self.en1_l,self.en1_r)
        e2 = Encoder(self.en2_l,self.en2_r)

        return e1, e2

    def forward(self):
        GPIO.output(self.in1_l,GPIO.HIGH)
        GPIO.output(self.in2_l,GPIO.LOW)
        GPIO.output(self.in1_r,GPIO.HIGH)
        GPIO.output(self.in2_r,GPIO.LOW)
        print('Forward')

    def backward(self):
        GPIO.output(self.in1_l,GPIO.LOW)
        GPIO.output(self.in2_l,GPIO.HIGH)
        GPIO.output(self.in1_r,GPIO.LOW)
        GPIO.output(self.in2_r,GPIO.HIGH)
        print('Backward')
    
    def left(self):
        GPIO.output(self.in1_l,GPIO.LOW)
        GPIO.output(self.in2_l,GPIO.HIGH)
        GPIO.output(self.in1_r,GPIO.HIGH)
        GPIO.output(self.in2_r,GPIO.LOW)
        print("Left")

    def right(self):
        GPIO.output(self.in1_l,GPIO.HIGH)
        GPIO.output(self.in2_l,GPIO.LOW)
        GPIO.output(self.in1_r,GPIO.LOW)
        GPIO.output(self.in2_r,GPIO.HIGH)
        print("Right")

    def stop(self):
        GPIO.output(self.in1_l,GPIO.LOW)
        GPIO.output(self.in2_l,GPIO.LOW)
        GPIO.output(self.in1_r,GPIO.LOW)
        GPIO.output(self.in2_r,GPIO.LOW)
        print("Stopped")