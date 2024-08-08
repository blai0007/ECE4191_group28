import cv2 
import numpy as np

vid = cv2.VideoCapture(0)       # Initiate Camera
dist = lambda x1,y1,x2,y2 : (x1-x2)**2 + (y1-y2)**2
prevCircle = None

while(True) :
    ret, frame = vid.read()     # Read Camera

    greyFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurFrame = cv2.GaussianBlur(greyFrame, (17,17), 0)

    ## PARAMS that might change
    dp = 1.2            # The higher, the higher chances of two circles merging
    minDist = 30        #Distance between 2 circles
    sensitivity = 80   # Higher, less circles detected
    accuracy = 60       # edgepoints that lead to accuracy
    minRadius = 20      # Circles that are far away
    maxRadius = 400

    circles = cv2.HoughCircles(blurFrame, cv2.HOUGH_GRADIENT, dp, minDist,param1 = sensitivity, param2=sensitivity, minRadius=minRadius, maxRadius=maxRadius)

    if circles is not None:
        circles = np.uint16( np.around(circles) )
        chosen = None 

        for i in circles[0,:] : 
            if chosen is None : chosen = i
            if prevCircle is not None : 
                if dist(chosen[0], chosen[1], prevCircle[0], prevCircle[1]) <= dist(i[0],i[1],prevCircle[0], prevCircle[1]):
                    chosen = i

        cv2.circle(frame, (chosen[0],chosen[1]), 1, (0,100,100), 3)
        cv2.circle(frame, (chosen[0],chosen[1]), chosen[2], (255,0,255), 3)
        prevCircle = chosen

    cv2.imshow('frame',frame)   # Show the image
    if cv2.waitKey(1) & 0xFF == ord('q'):       # IF we press "Q" button
        break