from adafruit_servokit import ServoKit
from time import sleep

kit = ServoKit(channels=16)

def servo1():
    for a in range(0,180) :
        kit.servo[0].angle = a
        sleep(0.1)

    for a in range(179,1,-1) :
        kit.servo[0].angle = a
        sleep(0.)

servo1()