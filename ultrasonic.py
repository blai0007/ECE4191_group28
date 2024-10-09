from gpiozero import DistanceSensor
ultrasonic = DistanceSensor(echo=10, trigger=9, threshold_distance=0.5)

while True:
    ultrasonic.wait_for_in_range()
    print(f"In range: {ultrasonic.distance}")
    ultrasonic.wait_for_out_of_range()
    print("Out of range") 