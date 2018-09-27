from lib_utils import *
import time

import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
CAMLED = 40
GPIO.setup(CAMLED, GPIO.OUT)

from picamera import PiCamera
picam = PiCamera()
picam.resolution = (2592, 1944)
picam.framerate = 15
picam.rotation = 180

# light led to ready people for photo
GPIO.setup(13, GPIO.OUT)
GPIO.output(13, GPIO.HIGH)
time.sleep(4)
GPIO.output(13, GPIO.LOW)

# take photos
GPIO.output(CAMLED, False)
picam.capture('{}_sr.jpg'.format(U_FILENAME))
GPIO.output(CAMLED, True)
picam.capture('{}_sl.jpg'.format(U_FILENAME))

# flash led
GPIO.output(13, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(13, GPIO.LOW)
GPIO.cleanup()
