"""Test script to take calibration pictures. Does not require camera library.

Attributes:
    CAMLED (int): GPIO output that is used to switch between right and left camera
    leds: LED object
    picam: PiCamera object
    U_FILENAME (string): Time-stamped filename for logger files
"""
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time
import os
import numpy as np

from lib_utils import *
from lib_leds import LEDS

os.makedirs('./{}/'.format(U_FILENAME))


def calib_pictures(no_pictures):
    """Takes pictures and flashes LEDs before and after
    
    Args:
        no_pictures (int): Number of calibration pictures
    """
    for i in range(no_pictures):
        # status
        print(i)

        # sleep
        time.sleep(3)
        
        # light led to ready people for photo
        flash_leds()
        # take photos
        picam.capture('./{}/calib{}.jpg'.format(U_FILENAME, i))
        flash_leds()

def flash_leds():
    """Flashes LEDs 
    """
    leds.on()
    time.sleep(0.25)
    leds.off()


CAMLED = 40
GPIO.setup(CAMLED, GPIO.OUT)
GPIO.output(CAMLED, False) # right camera
from picamera import PiCamera
picam = PiCamera()
picam.resolution = (2592, 1944) # full resolution for calibration
picam.framerate = 15
picam.rotation = 180

leds = LEDS()
calib_pictures(20) # number of calibration pictures
GPIO.cleanup()
