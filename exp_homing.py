import RPi.GPIO as GPIO

import time
import numpy as np
from picamera import PiCamera

import move
from camera import Camera
from blob import Blob

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# arm motors
move.initialize()

# create camera and choose settings
camera = Camera()
camera.settings()

time_start = time.time()
while time.time() - time_start < 20:
	# analyze right side
	img = camera.capture('right')
	blobs_right = Blob(img)
	blobs_right.blob_detect()
	print(blobs_right.blobs)

	# analyze left side
	img = camera.capture('left')
	blobs_left = Blob(img)
	blobs_left.blob_detect()
	print(blobs_left.blobs)


	if blobs_right.blobs.size and blobs_left.blobs.size:
		print('fwd')
		move.forward()

	elif blobs_right.blobs.size:
		print('left_fin')
		move.cw()

	else:
		print('right_fin')
		move.ccw()

move.stop()
move.terminate()

# This is a useless comment
