import RPi.GPIO as GPIO

import time
import numpy as np
from picamera import PiCamera


class Camera():
<<<<<<< HEAD
=======
    
>>>>>>> ed75409b20b6d9e81662f01881de68aef3095e14
    def __init__(self, x_res=192, y_res=144):
        self.x_res = x_res
        self.y_res = y_res

<<<<<<< HEAD
        # camera settings
        self.CAMLED = 40
        GPIO.setup(self.CAMLED, GPIO.OUT)
=======
    def settings(self):
        self.CAMLED = 40
        GPIO.setup(self.CAMLED, GPIO.OUT)

>>>>>>> ed75409b20b6d9e81662f01881de68aef3095e14
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
<<<<<<< HEAD

=======
>>>>>>> ed75409b20b6d9e81662f01881de68aef3095e14
        img = np.empty((self.y_res, self.x_res, 3), dtype=np.uint8)

        if side == 'right':
            GPIO.output(self.CAMLED, False) # Set to right cam
<<<<<<< HEAD
            #self.picam.capture('right.jpg', use_video_port=True)

        elif side == 'left':    
            GPIO.output(self.CAMLED, True) # Set to left cam
            #self.picam.capture('left.jpg', use_video_port=True)
=======
            self.picam.capture('right.jpg', use_video_port=True)

        elif side == 'left':    
            GPIO.output(self.CAMLED, True) # Set to left cam
            self.picam.capture('left.jpg', use_video_port=True)
>>>>>>> ed75409b20b6d9e81662f01881de68aef3095e14

        else:
            print('camera error: select btw right and left camera')

        self.picam.capture(img, 'rgb', use_video_port=True)

<<<<<<< HEAD
        return img
=======
        return img
>>>>>>> ed75409b20b6d9e81662f01881de68aef3095e14
