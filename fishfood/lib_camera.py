"""Camera library, a component of vision library. Takes images.
"""
import RPi.GPIO as GPIO

from lib_utils import *
import numpy as np
from picamera import PiCamera


class Camera():

    """Camera takes images and saves them in arrays to be processed by Blob.
    
    Attributes:
        CAMLED (int): GPIO output that is used to switch between right and left camera
        img (int): Array for raw RGB image, (U_CAM_MRES, U_CAM_NRES, 3)
        no_l (int): Image number taken by the left camera, for labelling stored images
        no_r (int): Image number taken by the right camera, for labelling stored images
        picam (): PiCamera object
        side (string): Camera side, right or left
        store_img (bool): Option to save images
    """
    
    # camera settings
    picam = PiCamera()
    picam.resolution = (U_CAM_NRES, U_CAM_MRES) # (horizontal, vertical)
    picam.framerate = 60
    picam.color_effects = (128, 128) # black and white
    picam.awb_mode = 'off'
    picam.awb_gains = (1, 1)
    picam.iso = 125
    picam.brightness = 35
    picam.contrast = 100
    CAMLED = 40
    GPIO.setup(CAMLED, GPIO.OUT)

    def __init__(self, side, store_img=False):
        """One Camera object is instantiated for each camera, i.e., the right and the left camera.
        
        Args:
            side (string): Camera side, right or left
            store_img (bool, optional): Option to save images
        """
        self.side = side
        self.store_img = store_img

        if self.store_img:
            self.no_r = 0
            self.no_l = 0
        self.img = np.empty((U_CAM_MRES, U_CAM_NRES, 3), dtype=np.uint8)

    def capture(self):
        """Takes an image with the selected camera (right or left) and stores it in self.img. Additionally saves the image if store_img=True.
        """
        if self.side == 'right':
            GPIO.output(self.CAMLED, U_CAM_RIGHT) # set to right cam
            if self.store_img:
                self.picam.rotation = 180
                self.picam.capture('./{}/{}_r{}.jpg'.format(U_FILENAME, U_FILENAME, self.no_r), use_video_port=True)
                self.picam.rotation = 0
                self.no_r += 1

        elif self.side == 'left':    
            GPIO.output(self.CAMLED, U_CAM_LEFT) # set to left cam
            if self.store_img:
                self.picam.rotation = 180
                self.picam.capture('./{}/{}_l{}.jpg'.format(U_FILENAME, U_FILENAME, self.no_l), use_video_port=True)
                self.picam.rotation = 0
                self.no_l += 1

        else:
            print('camera error: select btw right and left camera')

        self.picam.capture(self.img, 'rgb', use_video_port=True)

    def capture_sequence(self, imgs):
        if self.side == 'right':
            GPIO.output(self.CAMLED, U_CAM_RIGHT) # set to right cam
        elif self.side == 'left':    
            GPIO.output(self.CAMLED, U_CAM_LEFT) # set to left cam

        self.picam.capture_sequence(imgs, format='rgb', use_video_port=True)

    def std_settings(self):
        self.picam.framerate = 60
        self.picam.exposure_mode = 'auto'
        self.picam.color_effects = (128, 128) # black and white
        self.picam.awb_gains = (1, 1)
        self.picam.brightness = 35

    def redblue_settings(self):
        self.picam.color_effects = None
        self.picam.awb_gains = (4.0, 1.0)
        self.picam.framerate = 20
        self.picam.shutter_speed = 50000 # us
        self.picam.exposure_mode = 'off'
        self.picam.brightness = 15
