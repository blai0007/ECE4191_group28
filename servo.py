from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)
try: 
    # Arms
    kit.servo[4].angle = 10
    kit.servo[15].angle = 150
    # Gate
    kit.servo[8].angle = 20
    # kit.continuous_servo[5].throttle = 1
    # time.sleep(0.25)
    # kit.continuous_servo[5].throttle = 0
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    # Arms
    kit.servo[4].angle = 140
    kit.servo[15].angle = 0
    # Gate
    kit.servo[8].angle = 100 # Close
    # kit.continuous_servo[5].throttle = -1
    # time.sleep(0.25)
    # kit.continuous_servo[5].throttle = 0