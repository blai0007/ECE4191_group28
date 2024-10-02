from picamera2 import Picamera2
import cv2
import time

# Initialize the camera
camera = Picamera2()
camera.configure(camera.create_still_configuration())
camera.start()

# Allow the camera to warm up
time.sleep(0.1)

while True:
    # Capture an image
    image = camera.capture_array()

    # Display the image
    cv2.imshow("Image", image)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break

