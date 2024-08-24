import RPi.GPIO as GPIO
from time import sleep

# Set up GPIO pins
clk_pin = 17  # Replace with your GPIO pin for A (CLK)
dt_pin = 18   # Replace with your GPIO pin for B (DT)

GPIO.setmode(GPIO.BCM)
GPIO.setup(clk_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Initialize variables
last_clk_state = GPIO.input(clk_pin)
encoder_value = 0

try:
    while True:
        # Read the current state of CLK
        clk_state = GPIO.input(clk_pin)
        dt_state = GPIO.input(dt_pin)

        # If the previous CLK state differs from the current state, a step has been made
        if clk_state != last_clk_state:
            # Determine the direction based on the state of DT
            if dt_state != clk_state:
                encoder_value += 1
                direction = "Clockwise"
            else:
                encoder_value -= 1
                direction = "Counterclockwise"

            print(f"Steps: {encoder_value}, Direction: {direction}")

        # Update the last state
        last_clk_state = clk_state

        # Small delay to prevent CPU overuse
        sleep(0.01)

except KeyboardInterrupt:
    print("Exiting program")

finally:
    GPIO.cleanup()
    print(f"Final Steps: {encoder_value}")