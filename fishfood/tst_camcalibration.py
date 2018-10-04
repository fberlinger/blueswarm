from lib_blob import Blob
from lib_leds import LEDS

import time
import os
import numpy as np

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

def calib_pictures(no_pictures):
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

def known_blobs(detection_threshold):
    # cam settings
    picam.color_effects = (128, 128)
    picam.awb_mode = 'off'
    picam.awb_gains = (1, 1)
    picam.iso = 120
    picam.brightness = 30
    picam.contrast = 100

    # low res 192 by 144
    picam.resolution = (192, 144)
    picam.framerate = 60

    picam.rotation = 180
    picam.capture('low.jpg', use_video_port=True)
    picam.rotation = 0

    img_low = np.empty((144, 192, 3), dtype=np.uint8)
    picam.capture(img_low, 'rgb', use_video_port=True)
    blobs_right_low = Blob(img_low, 'right', detection_threshold, 192, 144)
    blobs_right_low.blob_detect()
    print('low res blob:')
    print(blobs_right_low.blobs)

    # full res 2592 by 1944
    picam.resolution = (2592, 1944)
    picam.framerate = 15

    picam.rotation = 180
    picam.capture('full.jpg')
    picam.rotation = 0


U_FILENAME = time.strftime("%y%m%d_UTC_%H%M%S")
leds = LEDS()
#os.makedirs('./{}/'.format(U_FILENAME))
#calib_pictures(20) # change number of pictures
known_blobs(60) # change detection_threshold based on distance [20-160], comment lines 131, 132, 139 and uncomment lines 135, 136, 142 in lib_blob.py
GPIO.cleanup()
