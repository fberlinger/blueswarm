import RPi.GPIO as GPIO

import time
import numpy as np
from picamera import PiCamera

from camera import Camera
from blob import Blob

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# create camera and choose settings
camera = Camera()
camera.settings()


# analyze right side
img = camera.capture(right)
blobs_right = Blob(img)
blobs_right.blob_detect()
print('blobs_right.blobs')

# analyze left side
img = camera.capture(left)
blobs_left = Blob(img)
blobs_left.blob_detect()
print('blobs_left.blobs')


#if blobs_right && blobs_left:
#	fwd

#else if blobs_right:
#	left

#else right




