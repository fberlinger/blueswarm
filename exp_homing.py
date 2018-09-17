import RPi.GPIO as GPIO

import time
import datetime
import numpy as np
from picamera import PiCamera

#import utils
import move
from camera import Camera
from blob import Blob




class Homing():	

	def __init__(self, run_time=20, exp_name='unnamed'):

		self.run_time = run_time
		self.t_capture_l = 0
		self.t_capture_r = 0
		self.t_blob_l = 0
		self.t_blob_r = 0
		self.t_loop = 0

		# logger instance
		self.exp_name = exp_name
		with open('{}.log'.format(self.exp_name), 'w') as f:
		    f.truncate()
		    f.write('t_loop    :: t_capture_l :: t_capture_r :: t_blob_l :: t_blob_r ::(ABSOLUTE TIME)\n')



<<<<<<< HEAD



	def log(self):
		with open('{}.log'.format(self.exp_name), 'a+') as f:
			f.write(
				'{:05} :: {:05} :: {:05} :: {:05} :: {:05} :: ({})\n'.format(
					self.t_loop, self.t_capture_l, self.t_capture_r, self.t_blob_l, self.t_blob_r, datetime.datetime.now()
					)
				)

	def run(self):
		time_start = time.time()
		t_loop = 0

		while time.time() - time_start < self.run_time:
			# analyze right side
			t_capture_l = time.time()
			img = camera.capture('right')
			self.t_capture_l = time.time() - t_capture_l
			t_blob_l = time.time()
			blobs_right = Blob(img)
			blobs_right.blob_detect()
			self.t_blob_l = time.time() - t_blob_l
			print(blobs_right.blobs)

			# analyze left side
			img = camera.capture('left')
			blobs_left = Blob(img)
			blobs_left.blob_detect()
			print(blobs_left.blobs)


			if blobs_right.blobs.size and blobs_left.blobs.size:
				print('fwd')
				#move.forward()

			elif blobs_right.blobs.size:
				print('left_fin')
				#move.cw()

			else:
				print('right_fin')
				#move.ccw()

			self.t_loop = time.time() - t_loop
			self.log()
			t_loop = time.time()

		move.stop()
		move.terminate()

if __name__ == "__main__":
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)

	# arm motors
	move.initialize()

	# create camera and choose settings
	camera = Camera()
	
	homing = Homing(10, 'exp_1')
	homing.run()
=======
move.stop()
move.terminate()

# This is a useless comment
>>>>>>> f497b462ad193b924c3ee6a7523534bd4843b6e8
