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

center = None
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

class Encoder:

    def __init__(self, leftPin, rightPin, callback=None):
        self.leftPin = leftPin
        self.rightPin = rightPin
        self.value = 0
        self.state = '00'
        self.direction = None
        self.callback = callback
        GPIO.setup(leftPin,GPIO.OUT)
        GPIO.setup(rightPin,GPIO.OUT)
    
    def get_distance(self): 
        return self.value * 4.32 

    def check_encoder(self):
        p1 = GPIO.input(self.leftPin)
        p2 = GPIO.input(self.rightPin)
        print(f"LEFT STATE : {p1}")
        print(f"RIGHT STATE : {p2}")
        newState = "{}{}".format(p1, p2)

        if self.state == "00": # Resting position
            if newState == "01": # Turned right 1
                self.direction = "R"
            elif newState == "10": # Turned left 1
                self.direction = "L"

        elif self.state == "01": # R1 or L3 position
            if newState == "11": # Turned right 1
                self.direction = "R"
            elif newState == "00": # Turned left 1
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        elif self.state == "10": # R3 or L1
            if newState == "11": # Turned left 1
                self.direction = "L"
            elif newState == "00": # Turned right 1
                if self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        else: # self.state == "11"
            if newState == "01": # Turned left 1
                self.direction = "L"
            elif newState == "10": # Turned right 1
                self.direction = "R"
            elif newState == "00": # Skipped an intermediate 01 or 10 state, but if we know direction then a turn is complete
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                elif self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                
        self.state = newState

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

# ENCODER SETUP
e1 = Encoder(encoder1_left_pin, encoder1_right_pin)
e2 = Encoder(encoder2_left_pin, encoder2_right_pin)

# Enable the Motor Drivers
p_left.start(45)
p_right.start(50)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

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


def drive_to_ball(area):
    if area > 1000 : 
        if area < 30000 or area > 10000:
            drive_forward()
        elif area > 30000 or area < 10000:
            drive_stop()


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
greenLower = (29, 50, 50) # Turn saturation lower if brighter light	, turn brightness up if detecting black
greenUpper = (73, 255, 255)
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

def center_ball():
	if center != None: 
		x_coord = center[0]
		if x_coord <=250 or x_coord >= 350:
			# drive_stop()
			if x_coord < 250: #Ball is on left
				print("On the Left")
				drive_left()
				time.sleep(0.1)
				drive_stop
			if x_coord > 350: #Ball is on right
				print("On the Right")
				drive_right()
				time.sleep(0.1)
				drive_stop()
		else:
			print("Ball is within 250-350 pixels")

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
                drive_forward()

            if event.key == pygame.K_DOWN : 
                print("DOWN")
                robot.ticks_right -= 1
                robot.ticks_left -= 1
                drive_backwards()

            if event.key == pygame.K_LEFT : 
                robot.ticks_right += 1
                robot.ticks_left -= 1
                print("LEFT")
                drive_left()

            if event.key == pygame.K_RIGHT : 
                robot.ticks_right -= 1
                robot.ticks_left += 1
                print("RIGHT")
                drive_right()

            if event.key == pygame.K_SPACE : 
                print("STOP")
                drive_stop()

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
    frame = cv2.rotate(frame, cv2.ROTATE_180) #Rotate Image 180 deg
    frame,_,_ = automatic_brightness_and_contrast(frame)

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

    center_ball()
    drive_to_ball(area)
    update_keyboard(Robot)
    localisation(Robot)
    Robot.ticks_left_prev = Robot.ticks_left
    Robot.ticks_right_prev = Robot.ticks_right
    print(f"X : {Robot.x}")
    print(f"Y : {Robot.y}")
    print(f"DEG : {Robot.deg}")

    # Encoder Stuff
    e1.check_encoder()
    e2.check_encoder()
    print(f"distance moved : {e1.get_distance()}")
    draw_window(Robot)
    time.sleep(0.1)
        

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