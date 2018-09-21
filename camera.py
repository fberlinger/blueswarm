import RPi.GPIO as GPIO

from utils import *
import numpy as np
from picamera import PiCamera


class Camera():

    def __init__(self, store_img=False, x_res=U_CAM_XRES, y_res=U_CAM_YRES):
        self.store_img = store_img
        self.x_res = x_res
        self.y_res = y_res

        if self.store_img:
            self.no_r = 0
            self.no_l = 0

        # camera settings
        self.CAMLED = 40
        GPIO.setup(self.CAMLED, GPIO.OUT)
        GPIO.output(self.CAMLED, False)
        self.picam = PiCamera()
        self.picam.resolution = (self.x_res, self.y_res)
        self.picam.framerate = 60
        self.picam.color_effects = (128, 128) # black and white
        self.picam.awb_mode = 'off'
        self.picam.awb_gains = (1, 1)
        self.picam.iso = 100
        self.picam.brightness = 30
        self.picam.contrast = 100

    def capture(self, side):
        img = np.empty((self.y_res, self.x_res, 3), dtype=np.uint8)

        if side == 'right':
            GPIO.output(self.CAMLED, False) # Set to right cam
            if self.store_img:
                self.picam.rotation = 180
                self.picam.capture('{}_r{}.jpg'.format(U_FILENAME, self.no_r), use_video_port=True)
                self.picam.rotation = 0
                self.no_r += 1

        elif side == 'left':    
            GPIO.output(self.CAMLED, True) # Set to left camv
            if self.store_img:
                self.picam.rotation = 180
                self.picam.capture('{}_l{}.jpg'.format(U_FILENAME, self.no_l), use_video_port=True)
                self.picam.rotation = 0
                self.no_l += 1

        else:
            print('camera error: select btw right and left camera')

        self.picam.capture(img, 'rgb', use_video_port=True)

        return img
