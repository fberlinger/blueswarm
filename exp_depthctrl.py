import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import csv
import time
import math
import numpy as np
from picamera import PiCamera

from utils import *
from camera import Camera
from blob import Blob
import move


def initialize():
    # logger instance for overall status
    with open('{}.log'.format(U_FILENAME), 'w') as f:
        f.truncate()
        f.write('t_passed :: t_loop :: t_observe_r :: #blob_r_pix :: #blob_l_pix\n')

    move.initialize()

def terminate():
    move.stop()
    move.terminate()

def log_status(t_passed, t_loop, t_observe_r, blob_r_size, blob_l_size):
    with open('{}.log'.format(U_FILENAME), 'a') as f:
        f.write(
            '  {:6.3f} :: {:6.3f} ::      {:6.3f} ::      {:6} ::      {:6}\n'.format(
                t_passed, t_loop, t_observe_r, blob_r_size, blob_l_size
                )
            )

def log_blobs(t_passed, blobs, side):
    #print(blobs)
    #print(blobs.shape)
    blob_list = U_CAM_YRES * np.ones((10, 2)) # max 10 blobs, remaining values U_CAM_YRES
    if blobs.size:
        blob_list[:blobs.shape[0], :blobs.shape[1]] = blobs
    blob_list = blob_list.reshape((1, blob_list.size))

    with open('{}_{}.csv'.format(U_FILENAME, side), 'a') as f:
        writer = csv.writer(f, delimiter=',')
        row = []
        row.append(t_passed)
        #for i in range(blob_list.size):
        for i in range(10):
            row.append(blob_list[0, i])
        writer.writerow(row)

def depth_ctrl_from_cam(blobs_right, blobs_left):
    if not blobs_right.size and not blobs_left.size:
        return

    if not blobs_right.size:
        blobs_right = blobs_left
    elif not blobs_left.size:
        blobs_left = blobs_right

    # discard blobs that are reflected on the surface
    blobs_r_ind = np.where(blobs_right == min(blobs_right[:, 1]))
    blobs_l_ind = np.where(blobs_left == min(blobs_left[:, 1]))
    blobs_r = blobs_right[blobs_r_ind[0], :]
    blobs_l = blobs_left[blobs_l_ind[0], :]

    #print(blobs_l)

    if ((blobs_right[0, 1] + blobs_left[0, 1]) / 2) < 0:
        #print('move down')
        move.down()
    else:
        move.stop()

def main(run_time=60):
    # loop
    t_start = time.time()
    t_loop_prev = time.time()
    while time.time() - t_start < run_time:
        # observe right side of environment and measure time for logging
        t_observe_r = time.time()
        img = camera.capture('right')
        blobs_right = Blob(img, 'right')
        blobs_right.blob_detect()
        t_observe_r = time.time() - t_observe_r

        # observe left side of environment
        img = camera.capture('left')
        blobs_left = Blob(img, 'left')
        blobs_left.blob_detect()

        total_blob_pixels = blobs_left.blob_size + blobs_right.blob_size

        # ctrl depth
        depth_ctrl_from_cam(blobs_right.blobs, blobs_left.blobs)

        # log status and blobs
        t_now = time.time()
        t_passed = t_now - t_start
        t_loop = t_now - t_loop_prev
        t_loop_prev = time.time()
        log_status(t_passed, t_loop, t_observe_r, blobs_right.blob_size, blobs_left.blob_size)
        log_blobs(round(t_passed, 3), blobs_right.blobs, 'right')
        log_blobs(round(t_passed, 3), blobs_left.blobs, 'left')

    terminate()


# homing plus orbiting
camera = Camera()
initialize()
main(180)
