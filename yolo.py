import cv2 
from ultralytics import YOLO
from imutils.video import VideoStream
import time
import numpy as np
class YOLODetector(object):
    def __init__(self, path):
        self.model = YOLO(path)
    
    def find_ball(self, frame):
        results = self.model.predict(frame, conf = 0.5, verbose = False)
        # print(results)
        centroid, rad, area = None, None, None
        if results: 
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0] # top, left, bottom, right 
                    rad = max(x2 - x1, y2 - y1) / 2
                    centroid = ((x1 + x2) / 2, (y1 + y2) / 2)
                    self.draw_circle(frame, centroid, rad)
                    area = np.pi * rad**2
                    return frame, centroid, rad, area
                # result.show()
        return frame, centroid, rad, area
    
    def draw_circle(self, frame, centroid, radius):
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), int(radius), (0, 255, 0), 4)
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 2, (0, 0, 255), 3)
        return None

path = 'yolo_test_ncnn_model'
vs = cv2.VideoCapture(0)
yolo = YOLODetector(path)
time.sleep(0.2)

while True:
    _,frame = vs.read()
    # frame = cv2.resize(frame, (320,240), interpolation=cv2.INTER_LINEAR)
    frame, centroid, rad, area = yolo.find_ball(frame)

    # cv2.imshow('Frame',frame)
    print(f'Center:{centroid}\nRad:{rad}')


    key = cv2.waitKey(1)
    if key == ord("q"):
        break



# yolo.find_ball('test2.jpg')
    