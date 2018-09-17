import RPi.GPIO as GPIO

import time
import numpy as np
from picamera import PiCamera


class Camera():

	def __init__(self, x_res=192, y_res=144):
		self.x_res = x_res
		self.y_res = y_res

		# camera settings
		self.CAMLED = 40
		GPIO.setup(self.CAMLED, GPIO.OUT)
		self.picam = PiCamera()
		self.picam.resolution = (self.x_res, self.y_res)
		self.picam.framerate = 60
		self.picam.color_effects = (128, 128) # black and white
		self.picam.awb_mode = 'off'
		self.picam.awb_gains = (1, 1)
		self.picam.iso = 100
		self.picam.brightness = 25
		self.picam.contrast = 100

	def capture(self, side):
		img = np.empty((self.y_res, self.x_res, 3), dtype=np.uint8)

		if side == 'right':
			GPIO.output(self.CAMLED, False) # Set to right cam
			#self.picam.capture('right.jpg', use_video_port=True)

		elif side == 'left':	
			GPIO.output(self.CAMLED, True) # Set to left cam
			#self.picam.capture('left.jpg', use_video_port=True)

		else:
			print('camera error: select btw right and left camera')

		self.picam.capture(img, 'rgb', use_video_port=True)

		return img
