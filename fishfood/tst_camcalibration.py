from lib_utils import *
from lib_leds import LEDS
import time
import os

import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
CAMLED = 40
GPIO.setup(CAMLED, GPIO.OUT)
GPIO.output(CAMLED, False)

from picamera import PiCamera
picam = PiCamera()
picam.resolution = (2592, 1944)
picam.framerate = 15
picam.rotation = 180

def flash_leds():
	leds.on()
	time.sleep(0.25)
	leds.off()

def main(no_pictures):
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

leds = LEDS()
os.makedirs('./{}/'.format(U_FILENAME))
main(20)
GPIO.cleanup()
