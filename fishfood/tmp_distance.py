
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time
import numpy as np
from picamera import PiCamera

from lib_utils import *
from lib_vision import Vision

def main(run_time=60):
    t_start = time.time()

    while time.time() - t_start < run_time:
        # check environment and find blob centroids of leds
        try:
            vision.update()
        except:
            continue

        if not vision.xyz_r.size:
            print('cannot see robot')
        else:
            rel_pos = (vision.xyz_r[:,0] + vision.xyz_r[:,1]) / 2
            print('distance {}'.format(np.linalg.norm(rel_pos)))

vision = Vision(2)
main(20)
