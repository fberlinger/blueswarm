"""Takes a selfie of visitors

Attributes:
    CAMLED (int): GPIO output that is used to switch between right and left camera
    picam: PiCamera object
"""
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time
import os
import numpy as np

from lib_utils import *
from lib_leds import LEDS


def selfie():
	GPIO.output(CAMLED, False)
	picam.capture('{}_sr.jpg'.format(U_FILENAME))
	GPIO.output(CAMLED, True)
	picam.capture('{}_sl.jpg'.format(U_FILENAME))

def flash_leds(dur):
    """Flashes LEDs 
    """
    leds.on()
    time.sleep(dur)
    leds.off()


CAMLED = 40
GPIO.setup(CAMLED, GPIO.OUT)
from picamera import PiCamera
picam = PiCamera()
picam.resolution = (2592, 1944) # full resolution for calibration
picam.framerate = 15
picam.rotation = 180

leds = LEDS()
flash_leds(4)
selfie()
flash_leds(0.5)
GPIO.cleanup()
