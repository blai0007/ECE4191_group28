from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)
try: 
    kit.servo[4].angle = 0
    kit.servo[15].angle = 140
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    kit.servo[4].angle = 140
    kit.servo[15].angle = 0