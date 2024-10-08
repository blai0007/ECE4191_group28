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
from scipy.signal import butter,filtfilt


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

def drive_stop():
    GPIO.output(in1_left,GPIO.LOW)
    GPIO.output(in2_left,GPIO.LOW)
    GPIO.output(in1_right,GPIO.LOW)
    GPIO.output(in2_right,GPIO.LOW) 

def calc_ticks_per_iter(current, prev, dt):
    abs(current - prev) / dt

# pi_controller = PIController(Kp=0.0003,Ki=0.01,Kd=0) #0.043
pi_controller = PIController(Kp=0.00008,Ki=0.01,Kd=0) #0.043

e1 = RotaryEncoder(encoder1_left_pin, encoder1_right_pin, max_steps=100000000)
e2 = RotaryEncoder(encoder2_left_pin, encoder2_right_pin, max_steps=100000000)

# PLEASE CHANGE TO SUM OF SLEEP FUNCTIONS
# dt = 0.05
# expected_duty_cycle = 1
# expected_rpm = 180 * (10/12) * expected_duty_cycle # rpm@efficient * motor@10V * duty_cycle
# expected_ticks_per_iter = 1300*dt #expected_rpm * (900*dt/60)

# 175.93mm == 898 ticks
# 1mm == 5.1 ticks
# approx. 1300 ticks per second
# distance = 1300/898 * 175.03 # distance (mm) per second
# w_expected = distance * 

# For plotting
plt.figure(figsize=(15, 5))
ticks_left_prev = 0
ticks_right_prev = 0
m1_speed = 0
m2_speed = 0

# m_per_tick = 60 / 3300                                  # Nathan and Bryan checked this, measure again if unsure
ticks_per_full_rotation = 900                            # TODO : Change this after wheel calibration
degrees_per_tick = 360 / ticks_per_full_rotation     
rps = 1 / 1.37
deg_per_s = rps*360/500
w_expected = 260
# w_expected = 1300*degrees_per_tick
left_array = []
right_array = []
array = []
dt = 1/500
j=0
try:
    for i in range(2500):
        left_ticks_iter = e1.steps - ticks_left_prev
        print(f"left ticks iter: {left_ticks_iter}")
        right_ticks_iter = e2.steps - ticks_right_prev
        w_left = (left_ticks_iter / dt) * degrees_per_tick 
        w_right = (right_ticks_iter / dt) * degrees_per_tick 

        left_array.append(pi_controller.motor_setpoint(w_right, w_left, dt))#w_left)
        right_array.append(w_right)

        # Motor control logic
        m1_speed = max(0, min(100, pi_controller.motor_setpoint(w_right, w_left, dt)))
        # m2_speed = max(0, min(100, pi_controller.motor_setpoint(w_expected, w_right, dt)))
        m2_speed = 70

        print(f"M1_SPEED: {m1_speed}")
        print(f"M2_SPEED: {m2_speed}")
        if i == 0:
            set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m2_speed)
        else:
            set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=m1_speed)
        set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=m2_speed)
        print("forward")

        # display.clear_output(wait=True)
        # display.display(plt.gcf())  # Update plot

        ticks_left_prev = e1.steps
        ticks_right_prev = e2.steps
        j += 1  # Increment time by 0.1 seconds
        array.append(j)
        sleep(dt)  # Sleep for the specified time step
    # Plotting the values
    plt.subplot(1, 2, 1)
    plt.plot(array, left_array)  # Plot using k as x-axis
    plt.axhline(y=85, color='r', linestyle='-')
    plt.title("Left Motor Ticks")
    plt.xlabel("Time (s)")
    plt.ylabel("Ticks")

    # plt.subplot(1, 2, 2)
    # plt.plot(array, right_array)  # Plot using k as x-axis
    # plt.axhline(y=85, color='r', linestyle='-')
    # plt.title("Right Motor Ticks")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Ticks")
    drive_stop()
    plt.show()

except KeyboardInterrupt:
    plt.savefig('motor_ticks_plot.png')  # Save plot when stopping
    drive_stop()
    GPIO.cleanup()
