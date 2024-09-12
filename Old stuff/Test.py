import numpy as np
import matplotlib
import cv2
from imutils.video import VideoStream

vid = cv2.VideoCapture(0)

while(True):
    ret, frame = vid.read()
    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):       # IF we press "Q" button
        break

test_list = np.array([0,1,2])
print(test_list)