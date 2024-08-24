import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

pin = 17  # Replace with the correct pin

GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def test_callback(channel):
    print("Edge detected on pin", channel)

GPIO.add_event_detect(pin, GPIO.BOTH, callback=test_callback, bouncetime=50)

try:
    while True:
        sleep(1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()