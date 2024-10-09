from gpiozero import DistanceSensor
ultrasonic = DistanceSensor(echo=10, trigger=9, threshold_distance=0.5)
def inRange():
    print(f"In Range: {ultrasonic.distance}")
def outRange():
    print("Out of Range")
while True:
    # print(ultrasonic.distance)
    ultrasonic.when_in_range = inRange
    ultrasonic.when_out_of_range = outRange