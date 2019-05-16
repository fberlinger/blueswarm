"""Take an image and average all pixel values to find brightness. Used for sync and swim.

Attributes:
    cam_l (TYPE): Description
    cam_r (TYPE): Description
    dur (TYPE): Description
"""
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
import numpy as np
import time
import math

from lib_utils import *
from lib_leds import LEDS
from lib_camera import Camera


def observe():
    flash = False
    thresh_flash = 100

    cam_r.capture()
    avg_bright_r = np.mean(cam_r.img)
    print('avg_bright_r = {}'.format(avg_bright_r))

    cam_l.capture()
    avg_bright_l = np.mean(cam_l.img)
    print('avg_bright_l = {}'.format(avg_bright_l))

    if avg_bright_r > thresh_flash or avg_bright_l > thresh_flash:
        flash = True

    return flash


cam_r = Camera('right')
cam_l = Camera('left')

dur = np.zeros((100,1))

for i in range(100):
	t_start = time.time()
	observe()
	dur[i] = time.time() - t_start

print(np.mean(dur))
print(np.max(dur))
