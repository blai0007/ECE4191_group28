import cv2
import numpy as np
import imutils
import time

class detector(object):
    def __init__(self, lower = (20, 50, 50), upper = (50, 255, 255)):
        self.lower = lower
        self.upper = upper
    
    def find_ball(self, frame):
        center = None
        radius = 0
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        if (len(cnts) > 0):
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # cv2.circle(frame, (int(x), int(y)), int(radius),
                #     (0, 255, 255), 2)
                # cv2.circle(frame, center, 5, (0, 0, 255), -1)
                self.draw_circle(frame, center, radius)
            else : 
                center = None

        # Drawing lines for borders
        h,w = frame.shape[:2]
        cv2.line(frame, (250,0), (250,h),(0,0,255),1)
        cv2.line(frame,(350,0),(350,h),(0,0,255),1)

        # Calc area
        area = radius ** 2 * np.pi

        return frame, center, radius, area 

    def draw_circle(self, frame, centroid, radius):
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), int(radius), (0, 255, 0), 4)
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 2, (0, 0, 255), 3)
        return None
    

vs = cv2.VideoCapture(0)
detect = detector()

time.sleep(0.2)

while True:
    _,frame = vs.read()
    frame, centroid, rad, area = detect.find_ball(frame)
    cv2.imshow('Frame',frame)

    key = cv2.waitKey(1)
    if key == ord("q"):
        break
