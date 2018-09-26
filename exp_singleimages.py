from utils import *

import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
CAMLED = 40
GPIO.setup(CAMLED, GPIO.OUT)

from picamera import PiCamera
picam = PiCamera()
picam.resolution = (640, 480)
picam.rotation = 180

GPIO.output(CAMLED, False)
picam.capture('{}_sr.jpg'.format(U_FILENAME), use_video_port=True)

GPIO.output(CAMLED, True)
picam.capture('{}_sl.jpg'.format(U_FILENAME), use_video_port=True)
