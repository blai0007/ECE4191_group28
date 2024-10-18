import numpy
import math


distance_y = 50
distance_x = -50
print(math.degrees(math.atan2(distance_y, abs(distance_x))))
print(math.degrees(math.atan(distance_y/distance_x)))