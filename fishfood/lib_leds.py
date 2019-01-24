"""LED library, provides functions to switch BlueBot's LEDs on and off.
"""
import threading
import RPi.GPIO as GPIO

class LEDS():

    """Functions to switch LEDs on and off.
    """
    
    def __init__(self):
        """All 3 LEDs are on pin 13 and can be operated together only
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(13, GPIO.OUT)

    def on(self):
        """Switch on
        """
        GPIO.output(13, GPIO.HIGH)
    def off(self):
        """Switch off
        """
        GPIO.output(13, GPIO.LOW)
