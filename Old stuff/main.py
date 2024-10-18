from New.drive import *
from New.localisation import localisation
from New.robot import Robot
from New.SLAM import SLAM
from yolo import YOLODetector
import cv2

# Initialising variables
center = None
GOING_BACK = 0
TURNING_BACK = 0
MOVING_BACK = 0
STEP_1_TURN_COMPLETE = 0 #initialise out of while loop
STEP_1_DRIVE_COMPLETE = 0
STEP_1_SPIN_COMPLETE = 0
BALL_FOUND = 0

# Setting pins
in1_l = 5 # 23
in2_l = 6 # 24
en_left =  11 #25                # Simulating encoder

in1_r = 19
in2_r = 26
en_right = 13               # simulating encoder

en1_l = 7
en2_l = 23
en1_r = 8
en2_r = 24

# Initialising classes
robot = Robot(in1_l, in2_l, in1_r, in2_r, en_left, en_right, en1_l, en1_r, en2_l, en2_r)
e1,e2 = robot.initialise()

slam = SLAM(robot)
WIN, WHITE, ORIGIN, BLUE = slam.initialise()

path = 'yolo_test.pt'
vs = cv2.VideoCapture(0)
yolo = YOLODetector(path)
time.sleep(0.2)

while True:
    # Finding ball
    _,frame = vs.read()
    frame, center, rad, area = yolo.find_ball(frame)
    cv2.imshow('Frame',frame)
    key = cv2.waitKey(1)

    if BALL_FOUND == 0 :
        if find_ball_step1(robot, e1.getValue(), e2.getValue(), STEP_1_TURN_COMPLETE, center) :
            BALL_FOUND = 1
    else : 
        update_drive(robot, area, GOING_BACK, TURNING_BACK, MOVING_BACK)

    if key == ord("q"):
        break
    
