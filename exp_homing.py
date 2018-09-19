import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import csv
import time
import datetime
import math
import numpy as np
from picamera import PiCamera

form utils import *
import move
from camera import Camera
from blob import Blob


status = ['home', 'orbit', 'terminate']

def initialize():
    # initial status
    status = 'home'

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

def log_status():
    with open('{}.log'.format(exp_name), 'a') as f:
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

def home(blobs_right, blobs_left, total_blob_pixels):
    # control
    thresh_orbit = 0.4 # [%], blob_pixels / total_no_pixels
    total_no_pixels = (U_CAM_YRES / 2)**2 * math.pi # pixels in spherical FOV

    # blob in front
    if blobs_right.size and blobs_left.size:
        print('move fwd')
        #move.forward()
        
        # initialize orbiting?
        if orbit:
            blob_ratio =  total_blob_pixels / (2 * total_no_pixels)
            if blob_ratio > thresh_orbit:
                status = 'orbit'

    # blob to the right
    elif blobs_right.size:
        print('turn cw')
        #move.cw()

    # blob to the left or behind
    else:
        print('turn ccw')
        #move.ccw()

def orbit(blobs_right):
    thresh_heading = 0.2
    horizontal_offset = blobs_right[0, 0] / (U_CAM_YRES / 2)

    if horizontal_offset > thresh_heading:
        print('turn ccw')
        #move.ccw()
    elif horizontal_offset < thresh_heading:
        print('turn cw')
        #move.cw()
    else:
        print('move fwd')    
        #move.forward()

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
       
        # act based on status
        if status == 'home':
            home(blobs_right.blobs, blobs_left.blobs, total_blob_pixels)
        elif status == 'orbit':
            orbit(blobs_right.blobs)
        elif status == 'terminate':
            terminate()

        # ctrl depth
        if depth_ctrl:
            depth_ctrl_from_cam(blobs_right.blobs, blobs_left.blobs)

        # log status
        t_now = time.time()
        t_loop = t_now - t_loop
        log_status(t_now, t_loop, t_observe_r, status)
        t_loop = time.time()

    terminate()


# homing plus orbiting, 2D or 3D
exp_name='unnamed'
orbit = False
depth_ctrl = False
initialize()
main(20)
