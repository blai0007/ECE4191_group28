import cv2 
from ultralytics import YOLO
from imutils.video import VideoStream
import time

class YOLODetector(object):
    def __init__(self, path, thresh=0.5):
        self.model = YOLO(path)
        self.thresh = thresh
    
    def find_ball(self, frame):
        results = self.model(frame)
        centroid, r_px = [], None

        for result in results:
            for box in result.boxes:

                x1, y1, x2, y2 = box.xyxy[0] # 
                confidence = box.conf[0].item()  

                if confidence >= self.thresh:
                    r_px = max(x2 - x1, y2 - y1) / 2
                    centroid = ((x1 + x2) / 2, (y1 + y2) / 2)

                    self.draw_circle(frame, centroid, r_px)
                    return frame, centroid, r_px
        
        return frame, centroid, r_px
    
    def draw_circle(self, frame, centroid, radius):
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), int(radius), (0, 255, 0), 4)
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 2, (0, 0, 255), 3)
        return None

path = 'ECE4191_group28/yolo_test.pt'
vs = cv2.VideoCapture(0)
yolo = YOLODetector(path, thresh=0.5)
time.sleep(0.2)

while True:
    _,frame = vs.read()
    yolo.find_ball(frame)
    cv2.imshow('Frame',frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break
    