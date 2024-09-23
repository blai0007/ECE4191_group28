import cv2 
from ultralytics import YOLO
from imutils.video import VideoStream
import time
import numpy as np
import torch
import pathlib
from pathlib import Path
pathlib.PosixPath = pathlib.WindowsPath

class YOLODetector(object):
    def __init__(self, path):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=path, force_reload=True)
        self.model.conf = 0.5
    
    def find_ball(self, frame):
        results = self.model(frame)
        # print(results)
        centroid, rad, area = None, None, None
        xyxy = results.xyxy[0]

        for result in xyxy:
            x1, y1, x2, y2, _, _ = result # top, left, bottom, right 
            rad = max(x2 - x1, y2 - y1) / 2
            centroid = ((x1 + x2) / 2, (y1 + y2) / 2)
            self.draw_circle(frame, centroid, rad)
            area = np.pi * rad**2
            return frame, centroid, rad, area
        return frame, centroid, rad, area
    
    def draw_circle(self, frame, centroid, radius):
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), int(radius), (0, 255, 0), 4)
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 2, (0, 0, 255), 3)
        return None

path = 'yolov5.pt'
vs = cv2.VideoCapture(0)
yolo = YOLODetector(path)
time.sleep(0.2)

while True:
    _,frame = vs.read()
    # frame = cv2.resize(frame, (160,120), interpolation=cv2.INTER_LINEAR)
    yolo.find_ball(frame)
    cv2.imshow('Frame',frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break

# yolo.find_ball('test2.jpg')


# path = 'yolov5.pt'
# model = torch.hub.load('ultralytics/yolov5', 'custom', path=path)
# results = model('test2.jpg')
# results.show()
# xyxy = results.xyxy[0]
# for result in xyxy:
#     x1,y1,x2,y2,_,_ = result
#     print(f'x1:{x1}, y1:{y1}, x2:{x2}, y2:{y2}')