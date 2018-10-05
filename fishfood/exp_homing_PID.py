import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import csv
import time
import math
import threading
import numpy as np
from picamera import PiCamera

from lib_utils import *
from lib_camera import Camera
from lib_blob import Blob
from lib_fin import Fin
from lib_leds import LEDS



status = ['home', 'orbit', 'terminate']

def initialize():
    threading.Thread(target=caudal.run).start()
    threading.Thread(target=dorsal.run).start()
    threading.Thread(target=pectol.run).start()
    threading.Thread(target=pector.run).start()

    # logger instance for overall status
    with open('{}.log'.format(U_FILENAME), 'w') as f:
        f.truncate()
        f.write('t_passed :: t_loop :: t_observe_r :: #blob_r_pix :: #blob_l_pix\n')

    leds.on()
    time.sleep(1)
    leds.off()

def terminate():
    caudal.terminate()
    dorsal.terminate()
    pectol.terminate()
    pector.terminate()

    leds.on()
    time.sleep(1)
    leds.off()

    GPIO.cleanup()

def log_status(t_passed, t_loop, t_observe_r, blob_r_size, blob_l_size, status):
    with open('{}.log'.format(U_FILENAME), 'a') as f:
        f.write(
            '  {:6.3f} :: {:6.3f} ::      {:6.3f} ::      {:6} ::      {:6} ::      {}\n'.format(
                t_passed, t_loop, t_observe_r, blob_r_size, blob_l_size, status
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

def home(blobs_right, blobs_left, total_blob_pixels):
    # control
    thresh_orbit = 0.4 # [%], blob_pixels / total_no_pixels
    total_no_pixels = (U_CAM_YRES / 2)**2 * math.pi # pixels in spherical FOV

    # blob in front
    if blobs_right.size and blobs_left.size:
        caudal.on()
        
        # keep centered
        if (blobs_right[0, 0] > blobs_left[0, 0] + 5):
            pector.set_frequency(2.5)
            pector.on()
            pectol.off()
            print('move fwd and ccw')
        elif (blobs_left[0, 0] > blobs_right[0, 0] + 5):
            pectol.set_frequency(2.5)
            pectol.on()
            pector.off()
            print('move fwd and cw')
        else:
            print('move fwd')
            pector.off()
            pectol.off()


        # initialize orbiting?
        if orbit:
            blob_ratio =  total_blob_pixels / (2 * total_no_pixels)
            if blob_ratio > thresh_orbit:
                status = 'orbit'

    # blob to the right
    elif blobs_right.size:
        freq_l = 2 + 6 * (U_CAM_YRES/2 - blobs_right[0, 0]) / U_CAM_YRES
        pectol.set_frequency(freq_l)
        #print('freq_l is {}'.format(freq_l))

        print('turn cw')
        pectol.on()
        pector.off()
        caudal.off()

    # blob to the left
    elif blobs_left.size:
        freq_r = 2 + 6 * (U_CAM_YRES/2 - blobs_left[0, 0]) / U_CAM_YRES
        pector.set_frequency(freq_r)
        #print('freq_r is {}'.format(freq_r))

        print('turn ccw')
        pector.on()
        pectol.off()
        caudal.off()

    # blob behind or lost
    else:
        print('lost blob, wait')
        pector.off()
        pectol.off()
        caudal.off()

def orbit(blobs_right):
    thresh_heading = 0.2
    horizontal_offset = blobs_right[0, 0] / (U_CAM_YRES / 2)

    if horizontal_offset > thresh_heading:
        print('turn ccw')
        #pector.on()
        #pectol.off()
        #caudal.off()
    elif horizontal_offset < thresh_heading:
        print('turn cw')
        #pectol.on()
        #pector.off()
        #caudal.off()
    else:
        print('move fwd')
        #caudal.on()
        #pector.off()
        #pectol.off()

def depth_ctrl_from_cam(blobs_right, blobs_left):
    if not blobs_right.size and not blobs_left.size:
        print('move up')
        dorsal.off()
        return

    if not blobs_right.size:
        blobs_right = blobs_left
    elif not blobs_left.size:
        blobs_left = blobs_right

    if ((blobs_right[0, 1] + blobs_left[0, 1]) / 2) < 0:
        print('move down')
        dorsal.on()
    else:
        print('move up')
        dorsal.off()

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
        blobs_r = blobs_right.blobs
        t_observe_r = time.time() - t_observe_r

        # observe left side of environment
        img = camera.capture('left')
        blobs_left = Blob(img, 'left')
        blobs_left.blob_detect()
        blobs_l = blobs_left.blobs

        total_blob_pixels = blobs_left.blob_size + blobs_right.blob_size

        # discard blobs that are reflected on the surface
        if blobs_r.size:
            blobs_r_ind = np.where(blobs_r == min(blobs_r[:, 1]))
            blobs_r = blobs_r[blobs_r_ind[0], :]
        if blobs_l.size:
            blobs_l_ind = np.where(blobs_l == min(blobs_l[:, 1]))
            blobs_l = blobs_l[blobs_l_ind[0], :]

        # act based on status
        if status == 'home':
            home(blobs_r, blobs_l, total_blob_pixels)
        elif status == 'orbit':
            orbit(blobs_r)
        elif status == 'terminate':
            terminate()

        # ctrl depth
        if depth_ctrl:
            depth_ctrl_from_cam(blobs_r, blobs_l)

        # log status and blobs
        t_now = time.time()
        t_passed = t_now - t_start
        t_loop = t_now - t_loop_prev
        t_loop_prev = time.time()
        log_status(t_passed, t_loop, t_observe_r, blobs_right.blob_size, blobs_left.blob_size, status)
        log_blobs(round(t_passed, 3), blobs_r, 'right')
        log_blobs(round(t_passed, 3), blobs_l, 'left')

    terminate()


# homing plus orbiting, 2D or 3D
status = 'home'
orbit = False
depth_ctrl = True

caudal = Fin(20, 21, 5)
dorsal = Fin(19, 26, 5)
pectol = Fin(18, 23, 6)
pector = Fin(4, 17, 6)
camera = Camera()
leds = LEDS()


time.sleep(5)
initialize()
main(70)
