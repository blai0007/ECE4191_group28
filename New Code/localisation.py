import numpy as np

def localisation(robot, e1_value, e2_value, e1, e2) : 
    distance_moved = 0
    degrees_turned = 0
    robot.ticks_left = e1_value
    robot.ticks_right = e2_value

    left_mag = (e1.rising_edges+e1.falling_edges)/2 #(e1.rising_edges+e1.falling_edges)/2
    robot.left_mag += left_mag
    robot.left_a += e1.rising_edges
    robot.left_b += e1.falling_edges
    
    print(f"left magnitude={left_mag}")
    
    right_mag = (e2.rising_edges+e2.falling_edges)/2

    print(f"right magnitude={right_mag}") 
    robot.right_mag += right_mag

    # MOVE FORWARDS
    if (robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its Forwards")
        distance_moved = (left_mag) * robot.m_per_tick #mm
        
    # MOVE BACKWARDS
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        print("Its Backwards")
        distance_moved = -(left_mag) * robot.m_per_tick

    # MOVE LEFT
    if ( robot.ticks_left < robot.ticks_left_prev ) and ( robot.ticks_right > robot.ticks_right_prev ) : 
        print("Its MOVING LEFT")
        degrees_turned = -(left_mag) * robot.degrees_per_tick
        print(f"Deg turned : {degrees_turned}")
        #deg_turned = rotation_calib 

    # MOVE RIGHT
    if ( robot.ticks_left > robot.ticks_left_prev ) and ( robot.ticks_right < robot.ticks_right_prev ) : 
        degrees_turned = (left_mag) * robot.degrees_per_tick
        print(f"Deg turned : {degrees_turned}")
        print("Its MOVING RIGHT")

    robot.y -= np.cos(np.deg2rad(robot.deg)) * distance_moved
    robot.x += np.sin(np.deg2rad(robot.deg)) * distance_moved
    robot.deg += degrees_turned

    print(f"Moving in x :  {np.sin(np.deg2rad(degrees_turned)) * distance_moved}")
    print(f"Distance Moved : {distance_moved}cm")

    if robot.deg < 0 : 
        robot.deg = 360 + robot.deg

    elif robot.deg > 360 :
        robot.deg = robot.deg - 360

    robot.ticks_left_prev = robot.ticks_left
    robot.ticks_right_prev = robot.ticks_right

    print(f"ROBOT LEFT_TICK : {robot.ticks_left_prev}")
    print(f"ROBOT Right_TICK : {robot.ticks_right_prev}")

    # print(np.sin(degrees_turned) * distance_moved)

    e1.rising_edges = 0
    e2.rising_edges =0
    e1.falling_edges = 0
    e2.falling_edges = 0
    return