from tensorflow.lite.python.interpreter import Interpreter
import cv2
import numpy as np
import time

class TFLite(object):
    def __init__(self, path):
        self.model = Interpreter(model_path=path)
        self.model.allocate_tensors()

    def find_ball(self, frame, min_conf = 0.5):
        centroid, rad, area = None, None, None

        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()
        height = input_details[0]['shape'][1]
        width = input_details[0]['shape'][2]

        #Resizing frame
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        imH, imW, _ = frame.shape
        image_resized = cv2.resize(image_rgb, (width, height))
        input_data = np.expand_dims(image_resized, axis=0)
        input_data = np.float32(input_data)

        self.model.set_tensor(input_details[0]['index'],input_data)
        self.model.invoke()

        # Retrieve detection results
        boxes = self.model.get_tensor(output_details[1]['index'])[0] # Bounding box coordinates of detected objects
        # classes = self.model.get_tensor(output_details[3]['index'])[0] # Class index of detected objects
        scores = self.model.get_tensor(output_details[0]['index'])[0] # Confidence of detected objects

        detections = []

        for i in range(len(scores)):
            if ((scores[i] > min_conf) and (scores[i] <= 1.0)):
                ymin = int(max(1,(boxes[i][0] * imH)))
                xmin = int(max(1,(boxes[i][1] * imW)))
                ymax = int(min(imH,(boxes[i][2] * imH)))
                xmax = int(min(imW,(boxes[i][3] * imW)))

                centroid = ((xmin + xmax) / 2, (ymin + ymax) / 2)
                rad = max(xmax - xmin, ymax - ymin) / 2
                area = np.pi * rad**2
                self.draw_circle(frame, centroid, rad)
                
                detections.append([centroid, rad, area])
        
        return frame, detections


    def draw_circle(self, frame, centroid, radius):
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), int(radius), (0, 255, 0), 4)
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 2, (0, 0, 255), 3)
        return None
    
path = 'detect.tflite'
vs = cv2.VideoCapture(0)
tflite = TFLite(path)
time.sleep(0.2)

while True:
    _,frame = vs.read()
    # frame = cv2.resize(frame, (320,240), interpolation=cv2.INTER_LINEAR)
    frame, detections = tflite.find_ball(frame)

    cv2.imshow('Frame',frame)
    if detections != []:
        for i in detections:
            print(f'Centroid:{i[0]}, Rad:{i[1]}, Area:{i[2]}')
    else:
        print('No Balls Found!')


    key = cv2.waitKey(1)
    if key == ord("q"):
        break
