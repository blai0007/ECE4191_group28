import time
import math
import numpy as np

# DRIVE SEQUENCES

def center_ball(Robot, going_back, center):
    if not going_back :
        if center != None: 
            x_coord = center[0]
            if x_coord <=250 or x_coord >= 350:
                # drive_stop()
                if x_coord < 250: #Ball is on left
                    print("On the Left")
                    Robot.left()
                    time.sleep(0.1)
                    Robot.stop()
                if x_coord > 350: #Ball is on right
                    print("On the Right")
                    Robot.right()
                    time.sleep(0.1)
                    Robot.stop()
            else:
                print("Ball is within 250-350 pixels")
                # drive_stop()

def drive_to_ball(Robot, area, going_back):
    if not (going_back) :
        if area > 1000 : 
            if area < 35000 :       # or area > 10000
                Robot.forward()
                return 0

            elif area > 35000 : # or area < 10000
                time.sleep(0.5)
                Robot.stop()
                print("It stopped")
                time.sleep(3)
                
                return 1
            
    if Robot.deg < 0 : 
        Robot.deg = 360 - Robot.deg

    elif Robot.deg > 360 :
        Robot.deg = Robot.deg - 360

def turning_back(Robot, threshold = 15) : 
    print("Turning to origin")
    distance_x = (Robot.x - Robot.starting_x)
    distance_y = -(Robot.y - Robot.starting_y)
    ideal_degree = 0

    # Treat this as cartesian plane
    if (distance_x > 0 ) and (distance_y > 0) : 
        ideal_degree = 270 - math.degrees(math.atan(abs(distance_y/distance_x))) 

    elif (distance_x > 0 ) and (distance_y < 0) : 
        ideal_degree = 270 + math.degrees(math.atan(abs(distance_y/distance_x)))

    elif (distance_x < 0 ) and (distance_y < 0) : 
        ideal_degree = 90 - math.degrees(math.atan(abs(distance_y/distance_x)))

    elif (distance_x < 0 ) and (distance_y > 0) : 
        ideal_degree = 90 + math.degrees(math.atan(abs(distance_y/distance_x)))

    print(f"ideal degree : {ideal_degree}")

    if (Robot.deg < (ideal_degree-threshold)) or (Robot.deg > (ideal_degree+threshold)):           # Not facing centre 
            # robot.deg -= robot.deg_per_iter
            Robot.left()

    else : 
        print("FACING CENTER")
        print(f"ideal_degree:{ideal_degree}")
        return 1
    
def moving_back(Robot) : 
    print("MOVING BACK")
    distance_x = abs(Robot.x - Robot.starting_x)
    distance_y = abs(Robot.y - Robot.starting_y)

    distance_overall = np.sqrt(distance_x**2 + distance_y**2) #units of pixels

    if distance_overall > 50 : 
        Robot.forward()
        return 0

    else : 
        print("reached origin")
        Robot.stop()
        return 1

# FINDING SEQUENCE

def find_ball_step1(Robot, STEP_1_TURN_COMPLETE, center, GOING_BACK):
    # print('Driving to spin point 1')
    if center == None or GOING_BACK == 0:
        if (Robot.x_cartesian < 1800 and STEP_1_TURN_COMPLETE == 0):
            print("Driving to 1st point")
            Robot.forward()
            return 0
        else:
            print("Spinning")
            STEP_1_TURN_COMPLETE == 1
            Robot.right()
            return 0
    else:
        return 1