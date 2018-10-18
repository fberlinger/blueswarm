import RPi.GPIO as GPIO

from lib_utils import *
import numpy as np
from picamera import PiCamera


class Camera():
    # camera settings
    picam = PiCamera()
    picam.resolution = (U_CAM_NRES, U_CAM_MRES) # (hor, ver)
    picam.framerate = 60
    picam.color_effects = (128, 128) # black and white
    picam.awb_mode = 'off'
    picam.awb_gains = (1, 1)
    picam.iso = 150
    picam.brightness = 50
    picam.contrast = 100
    CAMLED = 40
    GPIO.setup(CAMLED, GPIO.OUT)

    def __init__(self, side, store_img=False):
        self.side = side
        self.store_img = store_img

        if self.store_img:
            self.no_r = 0
            self.no_l = 0
        self.img = np.empty((U_CAM_MRES, U_CAM_NRES, 3), dtype=np.uint8)

    def capture(self):
        if self.side == 'right':
            GPIO.output(self.CAMLED, False) # set to right cam
            if self.store_img:
                self.picam.rotation = 180
                self.picam.capture('./{}/{}_r{}.jpg'.format(U_FILENAME, U_FILENAME, self.no_r), use_video_port=True)
                self.picam.rotation = 0
                self.no_r += 1

        elif self.side == 'left':    
            GPIO.output(self.CAMLED, True) # set to left cam
            if self.store_img:
                self.picam.rotation = 180
                self.picam.capture('./{}/{}_l{}.jpg'.format(U_FILENAME, U_FILENAME, self.no_l), use_video_port=True)
                self.picam.rotation = 0
                self.no_l += 1

        else:
            print('camera error: select btw right and left camera')

        self.picam.capture(self.img, 'rgb', use_video_port=True)
