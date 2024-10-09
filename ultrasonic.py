from gpiozero import DistanceSensor
import RPi.GPIO as GPIO

# Ultrasonic Pi Pins
echo = 11
trigger = 9

# LEFT MOTOR - WHEELS
in1_left = 5                
in2_left = 6                

# RIGHT MOTOR - WHEELS
in1_right = 19                
in2_right = 26

#Setup of GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(echo,GPIO.IN)
GPIO.setup(trigger,GPIO.OUT)
GPIO.setup(in1_left,GPIO.OUT)
GPIO.setup(in2_left,GPIO.OUT)
GPIO.setup(in1_right,GPIO.OUT)
GPIO.setup(in2_right,GPIO.OUT)

ultrasonic = DistanceSensor(echo=echo,trigger=trigger,threshold_distance=0.3) 

#Intialisation of Motors - Starting ON GOING BACKWARDS 
GPIO.output(in1_left,GPIO.LOW)              
GPIO.output(in2_left,GPIO.HIGH)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.HIGH)

ultrasonic.wait_for_in_range()


#Stop Motors
GPIO.output(in1_left,GPIO.LOW)              
GPIO.output(in2_left,GPIO.LOW)
GPIO.output(in1_right,GPIO.LOW)
GPIO.output(in2_right,GPIO.LOW)


    
