import RPi.GPIO as GPIO          
from time import sleep
import numpy as np
from adafruit_pca9685 import PCA9685
from gpiozero import RotaryEncoder
from PI_Controller import PIController
import board
import busio
from matplotlib import pyplot as plt
from IPython import display

# Set Pins
in1_left = 5
in2_left = 6
in1_right = 19
in2_right = 26

encoder1_left_pin = 25
encoder2_left_pin = 23
encoder1_right_pin = 16
encoder2_right_pin = 24

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1_left, GPIO.OUT)
GPIO.setup(in2_left, GPIO.OUT)
GPIO.setup(in1_right, GPIO.OUT)
GPIO.setup(in2_right, GPIO.OUT)

GPIO.output(in1_left, GPIO.LOW)
GPIO.output(in2_left, GPIO.LOW)
GPIO.output(in1_right, GPIO.LOW)
GPIO.output(in2_right, GPIO.LOW)

# Initialize I2C and PWM
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000

pi_controller = PIController(Kp=3e6, Ki=1e9)

def set_motor(in1, in2, motor_num, direction, speed):
    if direction: # forward
        GPIO.output(in1,GPIO.HIGH)  
        GPIO.output(in2,GPIO.LOW)
    else: # back
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
    speed = set_speed(speed)
    pca.channels[motor_num].duty_cycle = speed

# input a percentage 0-100 to set speed
def set_speed(percentage_val):
    speed = int(np.floor((percentage_val/100) * 65535))# CircuitPython apparently converts to 16 bit number 
    print(speed)
    return speed

def drive_forward():
    m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_ticks_per_iter, left_ticks_iter, dt)))
    m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_ticks_per_iter, right_ticks_iter, dt)))

    print(f"M1_SPEED: {m1_speed}")
    print(f"M2_SPEED: {m2_speed}")

    set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
    set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)
    print("forward")

def drive_stop():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.LOW) 

def calc_ticks_per_iter(current, prev, dt):
    abs(current - prev) / dt


e1 = RotaryEncoder(encoder1_left_pin, encoder1_right_pin, max_steps=100000000)
e2 = RotaryEncoder(encoder2_left_pin, encoder2_right_pin, max_steps=100000000)

# PLEASE CHANGE TO SUM OF SLEEP FUNCTIONS
dt = 0.05
expected_duty_cycle = 1
expected_rpm = 180 * (10/12) * expected_duty_cycle # rpm@efficient * motor@10V * duty_cycle
expected_ticks_per_iter = 1300*dt #expected_rpm * (900*dt/60)

# For plotting
plt.figure(figsize=(15, 5))
ticks_left_prev = 0
ticks_right_prev = 0
m1_speed = 0
m2_speed = 0

try:
    for i in range(100):
        # Calculate tick changes
        left_ticks_iter = abs(e1.steps - ticks_left_prev) / dt
        right_ticks_iter = abs(e2.steps - ticks_right_prev) / dt
        print(f"left ticks prev = {ticks_left_prev}")
        print(f"right ticks prev = {ticks_right_prev}")
        # Store values for plotting

        print(f"left ticks iter = {left_ticks_iter}")
        print(f"right ticks iter = {right_ticks_iter}")

        # Motor control logic
        drive_forward()

        # Plotting the values
        plt.subplot(1, 2, 1)
        plt.plot(i, pi_controller.motor_setpoint(expected_ticks_per_iter, left_ticks_iter, dt), 'bo')  # Plot using k as x-axis
        plt.axhline(y=85, color='r', linestyle='-')
        plt.title("Left Motor Ticks")
        plt.xlabel("Time (s)")
        plt.ylabel("Ticks")

        plt.subplot(1, 2, 2)
        plt.plot(i, pi_controller.motor_setpoint(expected_ticks_per_iter, right_ticks_iter, dt), 'bo')  # Plot using k as x-axis
        plt.axhline(y=85, color='r', linestyle='-')
        plt.title("Right Motor Ticks")
        plt.xlabel("Time (s)")
        plt.ylabel("Ticks")

        display.clear_output(wait=True)
        display.display(plt.gcf())  # Update plot

        ticks_left_prev = e1.steps
        ticks_right_prev = e2.steps
        i += 1  # Increment time by 0.1 seconds

        sleep(dt)  # Sleep for the specified time step

    drive_stop()
    plt.show()

except KeyboardInterrupt:
    plt.savefig('motor_ticks_plot.png')  # Save plot when stopping
    drive_stop()
    GPIO.cleanup()
