import RPi.GPIO as GPIO    
from time import sleep
from Encoder import Encoder

GPIO.setmode(GPIO.BCM)

encoder1_left_pin = 7 # 7
encoder2_left_pin = 23
encoder1_right_pin = 8
encoder2_right_pin = 24

# Initialise Pins
GPIO.setmode(GPIO.BCM)

e1 = Encoder(encoder2_left_pin, encoder2_right_pin)
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