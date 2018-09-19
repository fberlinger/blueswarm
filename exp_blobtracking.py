import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import csv
import time
import datetime
import math
import numpy as np
from picamera import PiCamera

from utils import *
from camera import Camera
from blob import Blob


def initialize(exp_name='unnamed'):
    # logger instance for overall status
    with open('{}.log'.format(exp_name), 'w') as f:
        f.truncate()
        f.write('t_passed    :: t_loop      :: t_observe_r :: status\n')

    # logger for blob centroids    
    with open('{}.csv'.format('blobs'), 'w') as f:
        f.truncate()

def log_status(t_passed, t_loop, t_observe_r, blob_size):
    with open('{}.log'.format(exp_name), 'a') as f:
        f.write(
            '{:.3f} :: {:.3f} :: {:.3f} :: {}\n'.format(
                t_passed, t_loop, t_observe_r, blob_size
                )
            )

def log_blobs(t_passed, blobs_right):
    print(blobs_right)
    print(blobs_right.shape)
    blob_list = 255 * np.ones((5, 2)) # max 5 blobs, remaining values are 255
    blob_list[:blobs_right.shape[0], :blobs_right.shape[1]] = blobs_right
    blob_list = blob_list.reshape((1, blob_list.size))

    with open('{}.csv'.format('blobs'), 'a') as f:
        writer = csv.writer(f, delimiter=',')
        row = []
        row.append(t_passed)
        for i in range(blob_list.size):
            row.append(blob_list[0, i])
        writer.writerow(row)

def main(run_time=60):
    # loop
    time_start = time.time()
    t_loop_prev = time.time()
    while time.time() - time_start < run_time:
        # observe right side of environment and measure time for logging
        t_observe_r = time.time()
        img = camera.capture('right')
        blobs_right = Blob(img)
        blobs_right.blob_detect()
        t_observe_r = time.time() - t_observe_r

        # log status and blobs
        t_now = time.time()
        t_passed = t_now - t_start
        t_loop = t_now - t_loop_prev
        t_loop_prev = time.time()
        log_status(t_passed, t_loop, t_observe_r, blobs_right.blob_size)
        log_blobs(t_passed, blobs_right.blobs)

# homing plus orbiting
# create camera and choose settings
camera = Camera()
exp_name='unnamed'
initialize()
main(40)
