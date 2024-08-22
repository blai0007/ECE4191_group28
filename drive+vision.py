import multiprocessing
import RPi.GPIO as GPIO          
from time import sleep
import pygame
from Encoder import Encoder

from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils

# Set Pins
in1_left = 23
in2_left = 24
en_left = 25

in1_right = 19
in2_right = 26
en_right = 13

encoder1_left_pin = 14
encoder2_left_pin = 15
encoder1_right_pin = 8
encoder2_right_pin = 7

center = None

# Initialise Pygame Module
pygame.init()
SCREEN_WIDTH =  600
SCREEN_HEIGHT = 400

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

# Initialise Pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1_left,GPIO.OUT)
GPIO.setup(in2_left,GPIO.OUT)
GPIO.setup(en_left,GPIO.OUT)

GPIO.setup(in1_right,GPIO.OUT)
GPIO.setup(in2_right,GPIO.OUT)
GPIO.setup(en_right,GPIO.OUT)

GPIO.output(in1_left,GPIO.LOW)
GPIO.output(in2_left,GPIO.LOW)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.LOW)

p_left=GPIO.PWM(en_left,1000)
p_right=GPIO.PWM(en_right,1000)

e1 = Encoder(encoder1_left_pin, encoder1_right_pin)
e2 = Encoder(encoder2_left_pin, encoder2_right_pin)

########################### VISION INNIT #########################################
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	vs = VideoStream(src=0).start()
# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])
# allow the camera or video file to warm up
sleep(0.5)
####################### VISION INITIALISED ###########################

# Enable the Motor Drivers
p_left.start(25)
p_right.start(25)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

def update_keyboard():
    for event in pygame.event.get():
        if event.type == pygame.quit : 
            break

        if event.type == pygame.KEYDOWN: 
            if event.key == pygame.K_UP :
                GPIO.output(in1_left,GPIO.HIGH)
                GPIO.output(in2_left,GPIO.LOW)
                GPIO.output(in1_right,GPIO.HIGH)
                GPIO.output(in2_right,GPIO.LOW)
                print("forward")

            if event.key == pygame.K_DOWN : 
                GPIO.output(in1_left,GPIO.LOW)
                GPIO.output(in2_left,GPIO.HIGH)
                GPIO.output(in1_right,GPIO.LOW)
                GPIO.output(in2_right,GPIO.HIGH)
                print("BACKWARDS")

            if event.key == pygame.K_LEFT : 
                GPIO.output(in1_left,GPIO.LOW)
                GPIO.output(in2_left,GPIO.HIGH)
                GPIO.output(in1_right,GPIO.HIGH)
                GPIO.output(in2_right,GPIO.LOW)
                print("LEFT")

            if event.key == pygame.K_RIGHT : 
                GPIO.output(in1_left,GPIO.HIGH)
                GPIO.output(in2_left,GPIO.LOW)
                GPIO.output(in1_right,GPIO.LOW)
                GPIO.output(in2_right,GPIO.HIGH)
                print("RIGHT")

            if event.key == pygame.K_q : 
                print("Quiting")
                GPIO.cleanup()
                break

def drive_forward():
    GPIO.output(in1_left,GPIO.HIGH)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.HIGH)
    GPIO.output(in2_right,GPIO.LOW)
    print("forward")

def drive_backwards():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.HIGH)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.HIGH)
    print("BACKWARDS")

def drive_left():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.HIGH)
    GPIO.output(in1_right,GPIO.HIGH)
    GPIO.output(in2_right,GPIO.LOW)
    print("LEFT")

def drive_right():
    GPIO.output(in1_left,GPIO.HIGH)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.HIGH)
    print("RIGHT")

def drive_stop():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.LOW)
    print("STOP")

def center_ball():
    x_coord = center[0]
    while x_coord <=250 or x_coord >= 350:
        drive_stop()
        if x_coord < 250: #Ball is on left
            print("On the Left")
            drive_left()
            sleep(0.5)
            drive_stop
        if x_coord > 350: #Ball is on right
            print("On the Right")
            drive_right()
            sleep(0.5)
            drive_stop()
    print("Ball is within 250-350 pixels")
    # drive_forward()
    # sleep(1)
    # drive_stop()
    #test

# Function for the first while loop
def loop1():
    while(True):
        # update_keyboard()
        center_ball()
        
        drive_forward()
        sleep(1)
        drive_stop()

        e1.get_distance()
        e2.get_distance()
        sleep(0.1)
        

# Function for the second while loop
def loop2():
    while True:
        # keep looping
        # grab the current frame
        frame = vs.read()
        # handle the frame from VideoCapture or VideoStream
        frame = frame[1] if args.get("video", False) else frame
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            break
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
        # update the points queue
        pts.appendleft(center)
        
            # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue
            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
        # show the frame to our screen
        # draw lines for borders
        h,_ = frame.shape
        cv2.line(frame, (0,250), (h,250),(0,0,255))
        cv2.line(frame,(0,350),(h,350),(0,0,255))
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

if __name__ == "__main__":
    # Create two processes
    process1 = multiprocessing.Process(target=loop1)
    process2 = multiprocessing.Process(target=loop2)

    # Start the processes
    process1.start()
    process2.start()

    # Keep the main program running to allow the processes to continue
    try:
        while True:
            sleep(0.1)
            
    except KeyboardInterrupt:
        print("Terminating processes")
        process1.terminate()
        process2.terminate()
        process1.join()
        process2.join()