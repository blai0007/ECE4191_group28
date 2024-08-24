from gpiozero import RotaryEncoder
from time import sleep

# Define the GPIO pins where the encoder is connected
clk_pin = 17  # Replace with the correct GPIO pin for A (CLK)
dt_pin = 18   # Replace with the correct GPIO pin for B (DT)

# Create an instance of the RotaryEncoder class
encoder = RotaryEncoder(clk_pin, dt_pin, max_steps=100)

# Define a function to print the current position and direction
def report():
    direction = "clockwise" if encoder.steps > 0 else "counterclockwise"
    print(f"Steps: {encoder.steps}, Direction: {direction}")

# Attach the function to be called when the encoder is turned
encoder.when_rotated = report

# Main loop to keep the script running
try:
    while True:
        sleep(0.1)
except KeyboardInterrupt:
    print("Exiting program")
finally:
    print("Final Steps:", encoder.steps)