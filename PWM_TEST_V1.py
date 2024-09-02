import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface
i2c = busio.I2C(SCL, SDA)

# Create a PCA9685 instance
pca = PCA9685(i2c)
pca.frequency = 60  # Frequency for controlling servos or motors

# Select the output channel (e.g., channel 0)
channel = pca.channels[0]

# Function to control the motor
def set_motor_speed(speed):
    # Ensure speed is within 0-100% range
    speed = max(0, min(100, speed))
    
    # Convert speed to PWM duty cycle (12-bit range: 0-4095)
    duty_cycle = int(speed * 4095 / 100)
    
    # Set the PWM duty cycle to control motor speed
    channel.duty_cycle = duty_cycle

speed = 50

try:
    while True:
        # Example: ramp up speed from 0 to 100%
        set_motor_speed(speed)
            
except KeyboardInterrupt:
    # Reset the PWM output on exit
    channel.duty_cycle = 0
    pca.deinit()
    print("Program terminated and PWM stopped.")