import RPi.GPIO as GPIO

class LEDS():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(13, GPIO.OUT)

    def on(self):
        GPIO.output(13, GPIO.HIGH)
    def off(self):
        GPIO.output(13, GPIO.LOW)
