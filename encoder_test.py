# import RPi.GPIO as GPIO   
# # impsort pigpio
# from gpiozero import RotaryEncoder
# from time import sleep
# from Encoder import Encoder

# encoder1_left_pin = 7 # 7
# encoder2_left_pin = 23
# encoder1_right_pin = 8
# encoder2_right_pin = 24

# GPIO.setmode(GPIO.BCM)

# GPIO.setup(23, GPIO.IN)           # pull_up_down=GPIO.PUD_DOWN
# GPIO.setup(24, GPIO.IN) 

# # Initialise Pins



# def get_steps(rotor):
#     print(f"{rotor.steps}")


# rotor = RotaryEncoder(a=23,b=24)

# while True : 
#     print(f"{rotor.steps}")

from gpiozero import RotaryEncoder
from signal import pause

# Initialize the rotary encoder
# Assuming your rotary encoder is connected to GPIO pins 17 and 18
encoder = RotaryEncoder(a=23, b=24, max_steps=100)

# Define the callback function to handle changes in the encoder
def on_rotate():
    print(f"Rotary Encoder value: {encoder.steps}")

# Attach the callback to the 'when_rotated' event
encoder.when_rotated = on_rotate

print("Rotary Encoder is ready. Rotate to see changes...")
pause()  # Keep the program running to capture events

# e1 = Encoder(encoder2_left_pin, encoder2_right_pin)
GPIO.cleanup()

# GPIO.add_event_detect(encoder1_left_pin, GPIO.BOTH, callback=self.transitionOccurred)  
# e1 = Encoder(encoder1_left_pin, encoder1_right_pin)
# # e2 = Encoder(encoder2_left_pin, encoder2_right_pin)

# import lgpio
# import RPi.GPIO as GPIO
# from time import sleep
# from Encoder import Encoder

# # Initialize GPIO
# h = lgpio.gpiochip_open(0)

# # Set Pins
# in1_left = 5 
# in2_left = 6 

# in1_right = 19
# in2_right = 26

# encoder1_left_pin = 7 
# encoder2_left_pin = 23
# encoder1_right_pin = 8
# encoder2_right_pin = 24

# left_speed = 75
# right_speed = 75

# # Set GPIO pins as output
# lgpio.gpio_claim_output(h, in1_left)
# lgpio.gpio_claim_output(h, in2_left)
# lgpio.gpio_claim_output(h, in1_right)
# lgpio.gpio_claim_output(h, in2_right)

# # Reset all pins to LOW
# lgpio.gpio_write(h, in1_left, 0)
# lgpio.gpio_write(h, in2_left, 0)
# lgpio.gpio_write(h, in1_right, 0)
# lgpio.gpio_write(h, in2_right, 0)

# # Initialize encoders
# e1 = Encoder(encoder1_left_pin, encoder1_right_pin)

# # Don't forget to close the GPIO at the end of your program
# lgpio.gpiochip_close(h)