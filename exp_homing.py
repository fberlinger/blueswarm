import RPi.GPIO as GPIO

import time
import numpy as np
from picamera import PiCamera

import img
import blob

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

Camera.camera_settings()



img = Camera.img_capture(right)
blobs_right = Blob.blob_detect(img)


img = Camera.img_capture(left)
blobs_left = Blob.blob_detect(img)


if blobs_right && blobs_left:
	fwd

else if blobs_right:
	left

else right




