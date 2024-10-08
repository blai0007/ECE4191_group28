from picamera2 import Picamera2
import time

# Initialize the camera
camera = Picamera2()

# Configure the camera
camera_config = camera.create_still_configuration()
camera.configure(camera_config)

# Start the camera
camera.start()

# Wait for the camera to warm up
time.sleep(2)

# Capture an image and save it as "image.jpg"
camera.capture_file("image.jpg")

# Stop the camera
camera.stop()

print("Image captured and saved as 'image.jpg'")
