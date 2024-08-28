# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import math
import imutils
import time
import pygame
import os
import numpy as np
import RPi.GPIO as GPIO    
from Encoder import Encoder  

center = None
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GOING_BACK = 0
TURNING_BACK = 0
MOVING_BACK = 0

# Set Pins
in1_left = 5 # 23
in2_left = 6 # 24
en_left =  11 #25                # Simulating encoder

in1_right = 19
in2_right = 26
en_right = 13               # simulating encoder

encoder1_left_pin = 7
encoder2_left_pin = 23
encoder1_right_pin = 8
encoder2_right_pin = 24

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

        self.mm_per_tick = 600 / 2150                              # Nathan and Bryan checked this, measure again if unsure
        self.ticks_per_full_rotation = 1750                              # TODO : Change this after wheel calibration
        self.degrees_per_tick = 360 / self.ticks_per_full_rotation      

        # self.distance_per_iter = 0.2                          # TODO : Used only for demo 1 (Only 1n approx)
        # self.deg_per_iter = 5

        # VISUALISATION
        self.width = 55
        self.height = 40
        self.image = pygame.image.load(os.path.join('PNGs', 'spaceship_red.png'))
        self.blit = pygame.transform.rotate(pygame.transform.scale(self.image, (self.width, self.height)), 180)
        self.rect = pygame.Rect(700, 300, self.width, self.height)

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


# Enable the Motor Drivers
p_left.start(100)
p_right.start(100)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

# ENCODER SETUP
e1 = Encoder(encoder1_left_pin, encoder1_right_pin)
e2 = Encoder(encoder2_left_pin, encoder2_right_pin)

def drive_forward(Robot):
    GPIO.output(in1_left,GPIO.HIGH)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.HIGH)
    GPIO.output(in2_right,GPIO.LOW)
    # distance_moved = (robot.ticks_left - robot.ticks_left_prev) * 4.13
    # Robot.y -= np.cos(np.deg2rad(Robot.deg)) * Robot.distance_per_iter
    # Robot.x += np.sin(np.deg2rad(Robot.deg)) * Robot.distance_per_iter
    print("forward")

def drive_backwards(Robot):
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.HIGH)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.HIGH)
    # Robot.y += np.cos(np.deg2rad(Robot.deg)) * Robot.distance_per_iter
    # Robot.x -= np.sin(np.deg2rad(Robot.deg)) * Robot.distance_per_iter
    print("BACKWARDS")

def drive_left(Robot):
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.HIGH)
    GPIO.output(in1_right,GPIO.HIGH)
    GPIO.output(in2_right,GPIO.LOW)
    # Robot.deg -= Robot.deg_per_iter
    print("LEFT")

def drive_right(Robot):
    GPIO.output(in1_left,GPIO.HIGH)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.HIGH)
    # Robot.deg += Robot.deg_per_iter
    print("RIGHT")  

def drive_stop():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.LOW) 


def drive_to_ball(Robot, area, going_back):
    if not (going_back) :
        if area > 1000 : 
            if area < 30000 :       # or area > 10000
                drive_forward(Robot)
                return 0

            elif area > 30000 : # or area < 10000
                drive_stop()
                print("It stopped")
                return 1
            
    if Robot.deg < 0 : 
        Robot.deg = 360 - Robot.deg

    elif Robot.deg > 360 :
        Robot.deg = Robot.deg - 360


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
# greenLower = (29, 50, 50) # Turn saturation lower if brighter light	, turn brightness up if detecting black
# greenUpper = (73, 255, 255)
greenLower = (20, 50, 50) # darker
greenUpper = (57, 255, 255) # lighter
pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	vs = VideoStream(src=0).start()
# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])
# allow the camera or video file to warm up
time.sleep(2.0)

def center_ball(Robot, going_back):
    if not going_back :
        if center != None: 
            x_coord = center[0]
            if x_coord <=250 or x_coord >= 350:
                # drive_stop()
                if x_coord < 250: #Ball is on left
                    print("On the Left")
                    drive_left(Robot)
                    # time.sleep(0.1)
                    # drive_stop()
                if x_coord > 350: #Ball is on right
                    print("On the Right")
                    drive_right(Robot)
                    # time.sleep(0.1)
                    # drive_stop()
            else:
                print("Ball is within 250-350 pixels")
                drive_stop()

def automatic_brightness_and_contrast(image, clip_hist_percent=25):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Calculate grayscale histogram
    hist = cv2.calcHist([gray],[0],None,[256],[0,256])
    hist_size = len(hist)

    # Calculate cumulative distribution from the histogram
    accumulator = []
    accumulator.append(float(hist[0]))
    for index in range(1, hist_size):
        accumulator.append(accumulator[index -1] + float(hist[index]))

    # Locate points to clip
    maximum = accumulator[-1]
    clip_hist_percent *= (maximum/100.0)
    clip_hist_percent /= 2.0

    # Locate left cut
    minimum_gray = 0
    while accumulator[minimum_gray] < clip_hist_percent:
        minimum_gray += 1

    # Locate right cut
    maximum_gray = hist_size -1
    while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
        maximum_gray -= 1

    # Calculate alpha and beta values
    alpha = 255 / (maximum_gray - minimum_gray)
    beta = -minimum_gray * alpha

    '''
    # Calculate new histogram with desired range and show histogram 
    new_hist = cv2.calcHist([gray],[0],None,[256],[minimum_gray,maximum_gray])
    plt.plot(hist)
    plt.plot(new_hist)
    plt.xlim([0,256])
    plt.show()
    '''

    auto_result = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    return (auto_result, alpha, beta)

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

def turning_back(robot) : 
    threshold = 10
    print("Turning to origin")
    distance_x = (robot.x - robot.starting_x)
    distance_y = -(robot.y - robot.starting_y)
    ideal_degree = 0

    # Treat this as cartesian plane
    if (distance_x > 0 ) and (distance_y > 0) : 
        ideal_degree = 270 - math.degrees(math.atan(abs(distance_y/distance_x))) 

    elif (distance_x > 0 ) and (distance_y < 0) : 
        ideal_degree = 270 + math.degrees(math.atan(abs(distance_y/distance_x)))

    elif (distance_x < 0 ) and (distance_y < 0) : 
        ideal_degree = 90 - math.degrees(math.atan(abs(distance_y/distance_x)))

    elif (distance_x < 0 ) and (distance_y > 0) : 
        ideal_degree = 90 + math.degrees(math.atan(abs(distance_y/distance_x)))

    print(f"ideal degree : {ideal_degree}")

    if (robot.deg < (ideal_degree-threshold)) or (robot.deg > (ideal_degree+threshold)):           # Not facing centre 
            # robot.deg -= robot.deg_per_iter
            drive_left(Robot)

    else : 
        print("FACING CENTER")
        print(f"ideal_degree:{ideal_degree}")
        return 1
    
def moving_back(robot) : 
    print("MOVING BACK")
    distance_x = abs(robot.x - robot.starting_x)
    distance_y = abs(robot.y - robot.starting_y)

    distance_overall = np.sqrt(distance_x**2 + distance_y**2)

    if distance_overall > 1 : 
        drive_forward(Robot)
        return 0

    else : 
        print("reached origin")
        drive_stop()
        return 1


def localisation(robot, e1_value, e2_value) : 
    distance_moved = 0
    degrees_turned = 0
    Robot.ticks_left = e1_value
    Robot.ticks_right = e2_value

    # MOVE FORWARDS
    if (robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its Forwards")
        distance_moved = (robot.ticks_left - robot.ticks_left_prev) * robot.mm_per_tick
        
    # MOVE BACKWARDS
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        print("Its Backwards")
        distance_moved = (robot.ticks_left - robot.ticks_left_prev) * robot.mm_per_tick

    # MOVE LEFT
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its MOVING LEFT")
        degrees_turned = (robot.ticks_left - robot.ticks_left_prev) * robot.degrees_per_tick
        print(f"Deg turned : {degrees_turned}")
        #deg_turned = rotation_calib 

    # MOVE RIGHT
    if ( robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        degrees_turned = (robot.ticks_left_prev - robot.ticks_left) * robot.degrees_per_tick
        print(f"Deg turned : {degrees_turned}")
        print("Its MOVING RIGHT")

    robot.y -= np.cos(np.deg2rad(robot.deg)) * distance_moved
    robot.x += np.sin(np.deg2rad(robot.deg)) * distance_moved
    robot.deg += degrees_turned

    print(f"Moving in x :  {np.sin(np.deg2rad(degrees_turned)) * distance_moved}")

    if robot.deg < 0 : 
        robot.deg = 360 + robot.deg

    elif robot.deg > 360 :
        robot.deg = robot.deg - 360

    robot.ticks_left_prev = robot.ticks_left
    robot.ticks_right_prev = robot.ticks_right

    print(f"ROBOT LEFT_TICK : {Robot.ticks_left_prev}")
    print(f"ROBOT Right_TICK : {Robot.ticks_right_prev}")

    # print(np.sin(degrees_turned) * distance_moved)
    return

def draw_window(robot):
    WIN.blit(WHITE, (0, 0))
    WIN.blit(ORIGIN, (robot.starting_x+20, robot.starting_y+20))
    robot.blit = pygame.transform.rotate(pygame.transform.scale(robot.image, (robot.width, robot.height)), -robot.deg+180)
    WIN.blit(robot.blit, (robot.x, robot.y))

    pygame.font.init()
    my_font = pygame.font.SysFont('Comic Sans MS', 30)
    location_txt = my_font.render(f'({np.round((robot.x- robot.starting_x),2)},{np.round((-(robot.y-robot.starting_y)),2)})', False, (0, 0, 0))
    WIN.blit(location_txt, (0,0))
    degrees_txt = my_font.render(f'Deg {np.round(robot.deg,2)}', False, (0, 0, 0))
    WIN.blit(degrees_txt, (0,50))

    E1_txt = my_font.render(f'E1 : {np.round(robot.ticks_left,2)}', False, (0, 0, 0))
    WIN.blit(E1_txt, (0,200))

    E2_txt = my_font.render(f'E2: {np.round(robot.ticks_right,2)}', False, (0, 0, 0))
    WIN.blit(E2_txt, (0,220))

    pygame.display.update()

# Start
FPS = 60
Robot = robot()

# keep looping
while True:
    center = None
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
    # frame = cv2.rotate(frame, cv2.ROTATE_180) #Rotate Image 180 deg
    # frame,_,_ = automatic_brightness_and_contrast(frame)

    # frame = cv2.flip(frame,0) # Mirror Image if needed
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
    radius = 0
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
    area = radius ** 2 * math.pi

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
    h,w = frame.shape[:2]
    cv2.line(frame, (250,0), (250,h),(0,0,255),1)
    cv2.line(frame,(350,0),(350,h),(0,0,255),1)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # print(f"AREA : {area}")

    # Updates on driving
    if drive_to_ball(Robot, area, GOING_BACK) : 
        print("GOING BACK")
        GOING_BACK = 1
        TURNING_BACK = 1
    else :
        center_ball(Robot, GOING_BACK)

    # if (update_keyboard(Robot)) : 
    #     GOING_BACK = 1
    #     TURNING_BACK = 1
    
    if GOING_BACK == 1 : 
        if TURNING_BACK == 1 : 
            if (turning_back(Robot)) :
                TURNING_BACK = 0
                MOVING_BACK = 1

        if MOVING_BACK == 1 : 
            if (moving_back(Robot)) : 
                GOING_BACK = 0
                MOVING_BACK = 0

    localisation(Robot, e1.getValue(), e2.getValue())
    draw_window(Robot)
    print(f"E1 : {e1.getValue()}")
    print(f"E2 : {e2.getValue()}")
    time.sleep(0.001)
        

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break


# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()
# otherwise, release the camera
else:
	vs.release()
# close all windows
cv2.destroyAllWindows()