import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import os
import csv
import time
import math
import threading
import numpy as np
from picamera import PiCamera

from lib_utils import *
from lib_camera import Camera
from lib_globalblob import GBlob
from lib_fin import Fin
from lib_leds import LEDS
from lib_depthsensor import DepthSensor

os.makedirs('./{}/'.format(U_FILENAME))


def initialize():
    threading.Thread(target=caudal.run).start()
    threading.Thread(target=dorsal.run).start()
    threading.Thread(target=pecto_l.run).start()
    threading.Thread(target=pecto_r.run).start()

    # logger instance for overall status
    with open('./{}/{}_status.log'.format(U_FILENAME, U_FILENAME), 'w') as f:
        f.truncate()
        f.write('t_passed :: distance :: status\n')

    leds.on()
    time.sleep(1)
    leds.off()

def terminate():
    caudal.terminate()
    dorsal.terminate()
    pecto_l.terminate()
    pecto_r.terminate()

    leds.on()
    time.sleep(1)
    leds.off()

    GPIO.cleanup()

def log_status(t_passed, distance, status):
    with open('./{}/{}_status.log'.format(U_FILENAME, U_FILENAME), 'a') as f:
        f.write(
            '  {:6.3f} ::     {:4} ::   {}\n'.format(t_passed, distance, status
                )
            )

def log_blobs(t_passed, side):
    #print(blobs)
    #print(blobs.shape)
    if (side == 'right'):
        blobs = blob_r.blobs
    elif (side == 'left'):
        blobs = blob_l.blobs

    max_blobs = 10
    blob_list = U_CAM_YRES * np.ones((max_blobs, 2)) # non-blob entries are set to U_CAM_YRES
    if blobs.size:
        blob_list[:blobs.shape[0], :blobs.shape[1]] = blobs
    blob_list = blob_list.reshape((1, blob_list.size))

    with open('./{}/{}_centroids_{}.csv'.format(U_FILENAME, U_FILENAME, side), 'a') as f:
        writer = csv.writer(f, delimiter=',')
        row = []
        row.append(t_passed)
        #for i in range(blob_list.size):
        for i in range(max_blobs):
            row.append(blob_list[0, i])
        writer.writerow(row)

def home():
    blobs_right = blob_r.blobs
    blobs_left = blob_l.blobs

    # blob in front
    if blobs_right.size and blobs_left.size:
        caudal.on()
        
        # keep centered
        if (blobs_right[0, 0] > blobs_left[0, 0] + 5):
            pecto_r.set_frequency(2.5)
            pecto_r.on()
            pecto_l.off()
            print('move fwd and ccw')
        elif (blobs_left[0, 0] > blobs_right[0, 0] + 5):
            pecto_l.set_frequency(2.5)
            pecto_l.on()
            pecto_r.off()
            print('move fwd and cw')
        else:
            print('move fwd')
            pecto_r.off()
            pecto_l.off()

    # blob to the right
    elif blobs_right.size:
        freq_l = 2 + 8 * (U_CAM_XRES/2 - blobs_right[0, 0]) / U_CAM_XRES
        pecto_l.set_frequency(abs(freq_l))

        print('turn cw')
        pecto_l.on()
        pecto_r.off()

        if (blobs_right[0, 0] > 55):
            caudal.on()
        else:
            caudal.off()

    # blob to the left
    elif blobs_left.size:
        freq_r = 2 + 8 * (U_CAM_XRES/2 - blobs_left[0, 0]) / U_CAM_XRES
        pecto_r.set_frequency(abs(freq_r))

        print('turn ccw')
        pecto_r.on()
        pecto_l.off()

        if (blobs_left[0, 0] > 55):
            caudal.on()
        else:
            caudal.off()

    # blob behind or lost
    else:
        print('lost blob, wait')
        pecto_r.off()
        pecto_l.off()
        caudal.off()

def orbit():
    thresh_heading = 0.2
    horizontal_offset = blobs_right[0, 0] / (U_CAM_YRES / 2)

    if horizontal_offset > thresh_heading:
        print('turn ccw')
        #pecto_r.on()
        #pecto_l.off()
        #caudal.off()
    elif horizontal_offset < thresh_heading:
        print('turn cw')
        #pecto_l.on()
        #pecto_r.off()
        #caudal.off()
    else:
        print('move fwd')
        #caudal.on()
        #pecto_r.off()
        #pecto_l.off()

def depth_ctrl_from_cam():
    blobs_right = blob_r.blobs
    blobs_left = blob_l.blobs

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

    # pressure sensor takeover. is not distance invariant, so start only when orbiting at fixed distance
    if status == 'orbit' and abs(((blobs_right[0, 1] + blobs_left[0, 1]) / 2)) < 10:
        depth_sensor.update()
        lock_depth = depth_sensor.depth_mm # i.e., lock_depth not false anymore
        depth_ctrl = False

def depth_ctrl_from_depthsensor(thresh=10):
    depth_sensor.update()

    if depth_sensor.depth_mm > (lock_depth + thresh):
        dorsal.off()
    elif depth_sensor.depth_mm < (lock_depth - thresh):
        dorsal.on()

def observe():
    # observe right side
    cam_r.capture()
    blob_r.run(cam_r.img)

    # observe left side
    cam_l.capture()
    blob_l.run(cam_l.img)

    # discard reflected blobs (run only for luring with single blob)
    blob_r.reflections()
    blob_l.reflections()

def main(run_time=60, target_dist=500): # [s, mm]
    t_start = time.time()
    
    while time.time() - t_start < run_time:
        # check environment and find blob centroids of leds
        observe()

        # todo: calculate distance of blob pair
        distance = 1000

        # update status
        if distance < target_dist:
            global status
            status = 'orbit'

        # act based on status
        if status == 'home':
            home()
        elif status == 'orbit':
            orbit()

        # control depth
        if depth_ctrl:
            depth_ctrl_from_cam()
        elif lock_depth:
            depth_ctrl_from_depthsensor()

        # log status and centroids
        t_passed = time.time() - t_start
        log_status(t_passed, distance, status)
        log_blobs(round(t_passed, 3), 'right')
        log_blobs(round(t_passed, 3), 'left')


# homing plus orbiting, 2D or 3D
status = 'home' # ['home', 'orbit']
depth_ctrl = True # 2D or 3D
lock_depth = False # use depth sensor once at target depth, set to mm value

caudal = Fin(U_FIN_C1, U_FIN_C2, 5) # freq
dorsal = Fin(U_FIN_D1, U_FIN_D2, 5) # freq
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 6) # freq
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 6) # freq
leds = LEDS()
cam_r = Camera('right')
cam_l = Camera('left')
blob_r = GBlob('right', 40) # detection threshold
blob_l = GBlob('left', 40) # detection threshold
depth_sensor = DepthSensor()

time.sleep(0.1)
initialize()
main(5) # run time
terminate()
