
# import RPi.GPIO as GPIO          
# from time import sleep
# # import pygame
# import numpy as np
# from adafruit_pca9685 import PCA9685
# from gpiozero import RotaryEncoder
# from PI_Controller import PIController
# import board
# import busio
# from matplotlib import pyplot as plt
# from IPython import display

# # kit = ServoKit(channels=16)

# # Set Pins
# in1_left = 5 # 23
# in2_left = 6 # 24
# # en_left =  11 #25                # Simulating encoder

# in1_right = 19
# in2_right = 26
# # en_right = 13               # simulating encoder

# encoder1_left_pin = 25
# encoder2_left_pin = 23
# encoder1_right_pin = 16
# encoder2_right_pin = 24

# expected_tick_per_sec = 0

# DIRECTION = "S"

# # Initialise Pins
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(in1_left,GPIO.OUT)
# GPIO.setup(in2_left,GPIO.OUT)
# # GPIO.setup(en_left,GPIO.OUT)

# GPIO.setup(in1_right,GPIO.OUT)
# GPIO.setup(in2_right,GPIO.OUT)
# # GPIO.setup(en_right,GPIO.OUT)

# GPIO.output(in1_left,GPIO.LOW)
# GPIO.output(in2_left,GPIO.LOW)
# GPIO.output(in1_right,GPIO.LOW)
# GPIO.output(in2_right,GPIO.LOW)

# i2c = busio.I2C(board.SCL, board.SDA)
# pca = PCA9685(i2c)
# pca.frequency = 1000  # Set PWM frequency for motor control

# e1 = RotaryEncoder(encoder1_left_pin, encoder1_right_pin, max_steps = 100000000)
# e2 = RotaryEncoder(encoder2_left_pin, encoder2_right_pin, max_steps = 100000000)

# # Enable the Motor Drivers
# print("\n")
# print("The default speed & direction of motor is LOW & Forward.....")
# print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
# print("\n")    

# def set_motor(in1, in2, motor_num, direction, speed):
#     if direction: # forward
#         GPIO.output(in1,GPIO.HIGH)
#         GPIO.output(in2,GPIO.LOW)
#     else: # back
#         GPIO.output(in1,GPIO.LOW)
#         GPIO.output(in2,GPIO.HIGH)
#     speed = set_speed(speed)
#     pca.channels[motor_num].duty_cycle = speed

# # input a percentage 0-100 to set speed
# def set_speed(percentage_val):
#     speed = int(np.floor((percentage_val/100) * 65535))# CircuitPython apparently converts to 16 bit number 
#     print(speed)
#     return speed


# def drive_forward():
#     m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
#     m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

#     set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
#     set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)
#     print("forward")

# def drive_backwards():
#     m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
#     m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

#     set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=m1_speed)
#     set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=m2_speed)
#     print("BACKWARDS")

# def drive_left():
#     m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
#     m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

#     set_motor(in1_left, in2_left, motor_num=0, direction=0, speed=m1_speed)
#     set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)
#     print("LEFT")

# def drive_right():
#     m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
#     m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

#     set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
#     set_motor(in1_right, in2_right, motor_num=1, direction=0, speed=m2_speed)
#     print("RIGHT")  

# def drive_stop():
#     GPIO.output(in1_left,GPIO.LOW)
#     GPIO.output(in2_left,GPIO.LOW)
#     GPIO.output(in1_right,GPIO.LOW)
#     GPIO.output(in2_right,GPIO.LOW) 

# # key press
# # rotate bot
# # conrim 360? Y/N
# # print ticks per full rotation 

# # Setpoint
# expected_rpm = 80 # EXPECTED SPEED OF MOTOR 0-100
# expected_tick_per_sec = expected_rpm * (900/60)
# dt = 0.1

# speed = 0
# pi_controller = PIController(Kp=10, Ki=0)
# j = []
# k=0
# ticks_left_prev_array = []
# ticks_right_prev_array = []

# plt.figure(figsize=(15,5)) 
# try:
#     while True:

#         j.append(k)
#         k+= 0.1
#         print(f"Encoder 1 :{e1.steps}")
#         print(f"Encoder 2 :{e2.steps}")

#         ticks_left_prev = e1.steps
#         ticks_right_prev = e2.steps
#         # Ticks per second
#         left_ticks_iter = abs(e1.steps - ticks_left_prev) / dt
#         right_ticks_iter = abs(e2.steps - ticks_right_prev) / dt
#         # ticks_left_prev_array.append(left_ticks_iter)
#         # ticks_right_prev_array.append(right_ticks_iter)

#         print(f"left ticks iter = {left_ticks_iter}")
#         drive_forward()

#         plt.subplot(1,2,1)
#         plt.plot(k, ticks_left_prev,'bo')
#         plt.axhline(y = expected_tick_per_sec, color = 'r', linestyle = '-')

#         plt.subplot(1,2,2)
#         plt.plot(k, ticks_right_prev, 'bo')
#         plt.axhline(y = expected_tick_per_sec, color = 'r', linestyle = '-')

#         plt.show()

#         display.clear_output(wait=True)
#         display.display(plt.gcf())
#         sleep(0.1)

# except KeyboardInterrupt:
#     drive_stop()
#     plt.savefig('sine_wave_plot.png')
    
#     GPIO.cleanup()

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

e1 = RotaryEncoder(encoder1_left_pin, encoder1_right_pin, max_steps=100000000)
e2 = RotaryEncoder(encoder2_left_pin, encoder2_right_pin, max_steps=100000000)

expected_rpm = 80
expected_tick_per_sec = expected_rpm * (900 / 60)
dt = 0.1

pi_controller = PIController(Kp=10, Ki=0)

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
    m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, left_ticks_iter, dt)))
    m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, right_ticks_iter, dt)))

    set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
    set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)
    print("forward")

# For plotting
plt.figure(figsize=(15, 5))
ticks_left_array = []
ticks_right_array = []
ticks_left_prev = 0
ticks_right_prev = 0
k = 0  # Time variable

try:
    while True:
        k += 0.1  # Increment time by 0.1 seconds

        ticks_left_prev = e1.steps
        ticks_right_prev = e2.steps

        # Calculate tick changes
        left_ticks_iter = abs(e1.steps - ticks_left_prev) / dt
        right_ticks_iter = abs(e2.steps - ticks_right_prev) / dt
        print(f"left ticks prev = {ticks_left_prev}")
        print(f"right ticks prev = {ticks_right_prev}")
        # Store values for plotting
        ticks_left_array.append(left_ticks_iter)
        ticks_right_array.append(right_ticks_iter)

        print(f"left ticks iter = {left_ticks_iter}")
        print(f"right ticks iter = {right_ticks_iter}")

        # Motor control logic
        drive_forward()

        # Plotting the values
        plt.subplot(1, 2, 1)
        plt.plot(k, left_ticks_iter, 'bo')  # Plot using k as x-axis
        plt.axhline(y=expected_tick_per_sec, color='r', linestyle='-')
        plt.title("Left Motor Ticks")
        plt.xlabel("Time (s)")
        plt.ylabel("Ticks")

        plt.subplot(1, 2, 2)
        plt.plot(k, right_ticks_iter, 'go')  # Plot using k as x-axis
        plt.axhline(y=expected_tick_per_sec, color='r', linestyle='-')
        plt.title("Right Motor Ticks")
        plt.xlabel("Time (s)")
        plt.ylabel("Ticks")

        display.clear_output(wait=True)
        display.display(plt.gcf())  # Update plot

        sleep(0.1)  # Sleep for the specified time step

except KeyboardInterrupt:
    plt.savefig('motor_ticks_plot.png')  # Save plot when stopping
    drive_stop()
    GPIO.cleanup()
