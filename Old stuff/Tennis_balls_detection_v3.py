import cv2
import numpy as np

# Initial min and max HSV filter values.
# These will be changed using trackbars
H_MIN = 0.2*256
H_MAX = 0.6*256
S_MIN = 0.5*256
S_MAX = 0.8*256
V_MIN =  0
V_MAX = 256

# Default capture width and height
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Max number of objects to be detected in frame
MAX_NUM_OBJECTS = 50

# Minimum and maximum object area
MIN_OBJECT_AREA = 20*20
MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH//1.5

# Names that will appear at the top of each window
windowName = "Original Image"
windowName1 = "HSV Image"
windowName2 = "Thresholded Image"
windowName3 = "After Morphological Operations"
trackbarWindowName = "Trackbars"

def on_trackbar(val):
    pass

def createTrackbars():
    # Create window for trackbars
    cv2.namedWindow(trackbarWindowName, 0)

    # Create trackbars and insert them into window
    cv2.createTrackbar("H_MIN", trackbarWindowName, H_MIN, H_MAX, on_trackbar)
    cv2.createTrackbar("H_MAX", trackbarWindowName, H_MAX, H_MAX, on_trackbar)
    cv2.createTrackbar("S_MIN", trackbarWindowName, S_MIN, S_MAX, on_trackbar)
    cv2.createTrackbar("S_MAX", trackbarWindowName, S_MAX, S_MAX, on_trackbar)
    cv2.createTrackbar("V_MIN", trackbarWindowName, V_MIN, V_MAX, on_trackbar)
    cv2.createTrackbar("V_MAX", trackbarWindowName, V_MAX, V_MAX, on_trackbar)

def drawObject(x, y, frame):
    # Draw crosshairs on your tracked image
    cv2.circle(frame, (x, y), 20, (0, 255, 0), 2)
    if y - 25 > 0:
        cv2.line(frame, (x, y), (x, y - 25), (0, 255, 0), 2)
    else:
        cv2.line(frame, (x, y), (x, 0), (0, 255, 0), 2)
    if y + 25 < FRAME_HEIGHT:
        cv2.line(frame, (x, y), (x, y + 25), (0, 255, 0), 2)
    else:
        cv2.line(frame, (x, y), (x, FRAME_HEIGHT), (0, 255, 0), 2)
    if x - 25 > 0:
        cv2.line(frame, (x, y), (x - 25, y), (0, 255, 0), 2)
    else:
        cv2.line(frame, (x, y), (0, y), (0, 255, 0), 2)
    if x + 25 < FRAME_WIDTH:
        cv2.line(frame, (x, y), (x + 25, y), (0, 255, 0), 2)
    else:
        cv2.line(frame, (x, y), (FRAME_WIDTH, y), (0, 255, 0), 2)

    cv2.putText(frame, f"{x},{y}", (x, y + 30), 1, 1, (0, 255, 0), 2)

def morphOps(thresh):
    # Create structuring element that will be used to "dilate" and "erode" image.
    erodeElement = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dilateElement = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))

    # Erode and dilate the image
    thresh = cv2.erode(thresh, erodeElement, iterations=2)
    thresh = cv2.dilate(thresh, dilateElement, iterations=2)
    return thresh

def trackFilteredObject(x, y, threshold, cameraFeed):
    temp = threshold.copy()
    contours, hierarchy = cv2.findContours(temp, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    refArea = 0
    objectFound = False

    if len(hierarchy) > 0:
        numObjects = len(hierarchy)
        if numObjects < MAX_NUM_OBJECTS:
            for contour in contours:
                moment = cv2.moments(contour)
                area = moment['m00']

                if area > MIN_OBJECT_AREA and area < MAX_OBJECT_AREA and area > refArea:
                    x = int(moment['m10'] / area)
                    y = int(moment['m01'] / area)
                    objectFound = True
                    refArea = area

            if objectFound:
                cv2.putText(cameraFeed, "Tracking Object", (0, 50), 2, 1, (0, 255, 0), 2)
                drawObject(x, y, cameraFeed)
        else:
            cv2.putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", (0, 50), 1, 2, (0, 0, 255), 2)

def main():
    # Some boolean variables for different functionality within this program
    trackObjects = False
    useMorphOps = False

    # Matrix to store each frame of the webcam feed
    cameraFeed = None
    HSV = None
    threshold = None

    # x and y values for the location of the object
    x = 0
    y = 0

    # Video capture object to acquire webcam feed
    capture = cv2.VideoCapture(1)

    # Set height and width of capture frame
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # Start an infinite loop where webcam feed is copied to cameraFeed matrix
    while True:
        ret, cameraFeed = capture.read()

        # Convert frame from BGR to HSV colorspace
        HSV = cv2.cvtColor(cameraFeed, cv2.COLOR_BGR2HSV)

        # Filter HSV image between values and store filtered image to threshold matrix
        threshold = cv2.inRange(HSV, (H_MIN, S_MIN, V_MIN), (H_MAX, S_MAX, V_MAX))

        # Perform morphological operations on thresholded image to eliminate noise
        if useMorphOps:
            threshold = morphOps(threshold)

        # Pass in thresholded frame to our object tracking function
        if trackObjects:
            trackFilteredObject(x, y, threshold, cameraFeed)

        # Show frames
        cv2.imshow(windowName2, threshold)
        cv2.imshow(windowName, cameraFeed)
        cv2.imshow(windowName1, HSV)

        # Delay 30ms so that screen can refresh
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()