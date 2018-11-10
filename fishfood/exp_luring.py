import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import os
import csv
import time
import threading
import numpy as np
from math import *
from picamera import PiCamera

from lib_utils import *
from lib_fin import Fin
from lib_leds import LEDS
from lib_vision import Vision
from lib_depthsensor import DepthSensor
from lib_ema import EMA

os.makedirs('./{}/'.format(U_FILENAME))

def initialize():
    threading.Thread(target=caudal.run).start()
    threading.Thread(target=dorsal.run).start()
    threading.Thread(target=pecto_l.run).start()
    threading.Thread(target=pecto_r.run).start()

    '''
    # logger instance for overall status
    with open('./{}/{}_status.log'.format(U_FILENAME, U_FILENAME), 'w') as f:
        f.truncate()
        #f.write('t_passed :: t_capture::   t_blob ::    t_uvw ::    t_pqr ::    t_xyz :: distance ::    x_pos :: status\n')
        f.write('t_passed :: distance ::    x_pos :: status\n')
    '''

    leds.on()
    time.sleep(1)
    leds.off()
    time.sleep(1)

def terminate():
    caudal.terminate()
    dorsal.terminate()
    pecto_l.terminate()
    pecto_r.terminate()

    leds.on()
    time.sleep(1)
    leds.off()

    GPIO.cleanup()

def log_status(t_passed, distance, x_pos, status):
    with open('./{}/{}_status.log'.format(U_FILENAME, U_FILENAME), 'a') as f:
        f.write(
            '  {:6.3f} ::     {:4.0f} ::     {:4.0f} ::   {}\n'.format(t_passed, distance, x_pos, status
                )
            )

def log_centroids(t_passed, side):
    if (side == 'right'):
        centroids = vision.xyz_r
    elif (side == 'left'):
        centroids = vision.xyz_l

    max_centroids = 3 * 3
    centroid_list = U_CAM_NRES * np.ones((3, max_centroids)) # non-blob entries are set to U_CAM_NRES
    if centroids.size:
        centroid_list[:centroids.shape[0], :centroids.shape[1]] = centroids
    centroid_list = np.transpose(centroid_list)
    centroid_list = centroid_list.reshape((1, centroid_list.size))

    with open('./{}/{}_centroids_{}.csv'.format(U_FILENAME, U_FILENAME, side), 'a') as f:
        writer = csv.writer(f, delimiter=',')
        row = []
        row.append(t_passed)
        #for i in range(blob_list.size):
        for i in range(max_centroids):
            row.append(centroid_list[0, i])
        writer.writerow(row)

def depth_ctrl_from_cam():
    right = vision.pqr_r
    left = vision.pqr_l

    if not right.size and not left.size:
        print('cant see blob')
        dorsal.off()
        return

    if not right.size:
        pitch_l = np.arctan2(left[2, 0], sqrt(left[0, 0]**2 + left[1, 0]**2)) * 180 / pi
        pitch_r = pitch_l
    elif not left.size:
        pitch_r = np.arctan2(right[2, 0], sqrt(right[0, 0]**2 + right[1, 0]**2)) * 180 / pi
        pitch_l = pitch_r
    else:
        pitch_r = np.arctan2(right[2, 0], sqrt(right[0, 0]**2 + right[1, 0]**2)) * 180 / pi
        pitch_l = np.arctan2(left[2, 0], sqrt(left[0, 0]**2 + left[1, 0]**2)) * 180 / pi

    pitch = (pitch_r + pitch_l) / 2
    print(pitch)

    if pitch > 1:
        print('move down')
        dorsal.on()
    elif pitch < -1:
        print('move up')
        dorsal.off()

def home():
    right = vision.pqr_r
    left = vision.pqr_l

    # blob behind or lost
    if not right.size and not left.size:
        #print('cant see blob')
        pecto_r.set_frequency(6)
        pecto_r.on()
        pecto_l.off()
        caudal.off()
        return

    if not right.size:
        heading_l = np.arctan2(left[1, 0], left[0, 0]) * 180 / pi
        heading_r = heading_l
    elif not left.size:
        heading_r = np.arctan2(right[1, 0], right[0, 0]) * 180 / pi
        heading_l = heading_r
    else:
        heading_r = np.arctan2(right[1, 0], right[0, 0]) * 180 / pi
        heading_l = np.arctan2(left[1, 0], left[0, 0]) * 180 / pi

    heading = (heading_r + heading_l) / 2

    # blob to the right
    if heading > 0:
        freq_l = 5 + 5 * abs(heading) / 180
        pecto_l.set_frequency(freq_l)

        #print('turn cw')
        pecto_l.on()
        pecto_r.off()

        if heading < 20:
            caudal.on()
        else:
            caudal.off()

    # blob to the left
    elif heading < 0:
        freq_r = 5 + 5 * abs(heading) / 180
        pecto_r.set_frequency(freq_r)

        #print('turn ccw')
        pecto_r.on()
        pecto_l.off()

        if heading > -20:
            caudal.on()
        else:
            caudal.off()

def main(run_time=60): # [s]
    t_start = time.time()
    
    while time.time() - t_start < run_time:
        # check environment and find blob centroids of leds
        vision.update()

        # control depth
        depth_ctrl_from_cam()

        # move towards light
        home()

        # log status and centroids
        t_passed = time.time() - t_start
        #log_status(t_passed, dist, x_pos, status)
        #log_centroids(round(t_passed, 3), 'right')
        #log_centroids(round(t_passed, 3), 'left')


caudal = Fin(U_FIN_C1, U_FIN_C2, 5) # freq
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 8) # freq
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 8) # freq
leds = LEDS()
vision = Vision()
depth_sensor = DepthSensor()

time.sleep(10)
initialize()
leds.on()
main(180) # run time
leds.off()
terminate()