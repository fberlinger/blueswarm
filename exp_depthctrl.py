import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import csv
import time
import datetime
import math
import numpy as np
from picamera import PiCamera

import utils
import move
from camera import Camera
from blob import Blob


def initialize(exp_name='unnamed'):
    # arm motors
    move.initialize()

    # create camera and choose settings
    camera = Camera()

    # logger instance for overall status
    with open('{}.log'.format(exp_name), 'w') as f:
        f.truncate()
        f.write('t_now :: t_loop :: t_observe_r :: status')

    # logger for blob centroids    
    with open('{}.csv'.format('blobs'), 'w') as f:
        f.truncate()

def terminate():
    move.stop()
    move.terminate()

def log_status(t_now, t_loop, t_observe_r, status):
    with open('{}.log'.format(self.exp_name), 'a') as f:
        f.write(
            '{:05} :: {:05} :: {:05} :: ({})\n'.format(
                t_now, t_loop, t_observe_r, status
                )
            )

def log_blobs(blobs_right):
    blob_list = 255 * np.ones((3, 2)) # max 3 blobs, remaining values are 255
    blob_list[:blobs_right.shape[0], :blobs_right.shape[1]] = blobs_right
    blob_list = blob_list.reshape((1, blob_list.size))

    with open('{}.csv'.format('blobs'), 'a') as f:
        writer = csv.writer(f, delimiter=',')
        row = []
        for i in range(blob_list.size):
            row.append(blob_list[0, i])
        writer.writerow(row)

def depth_ctrl_from_cam(blobs_right, blobs_left):
    if (blobs_right[0, 1] + blobs_left[0, 1]) > 0:
        print('move down')
        #move.down()

def main(run_time=60):
    # loop
    time_start = time.time()
    t_loop = time.time()
    while time.time() - time_start < run_time:
        # observe right side of environment and measure time for logging
        t_observe_r = time.time()
        img = camera.capture('right')
        blobs_right = Blob(img)
        blobs_right.blob_detect()
        t_observe_r = time.time() - t_observe_r

        # log blobs on right side of environment
        log_blobs(blobs_right.blobs)

        # observe left side of environment
        img = camera.capture('left')
        blobs_left = Blob(img)
        blobs_left.blob_detect()

        total_blob_pixels = blobs_left.blob_size + blobs_right.blob_size

        # ctrl depth
        depth_ctrl_from_cam(blobs_right.blobs, blobs_left.blobs)

        # log status
        time_now = time.time()
        t_loop = time_now - t_loop
        log_status(time_now, t_loop, t_observe_r, status)
        t_loop = time.time()

    terminate()


# homing plus orbiting
initialize('exp_1')
main(20)
