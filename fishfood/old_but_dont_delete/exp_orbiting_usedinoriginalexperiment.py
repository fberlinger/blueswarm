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

    # logger instance for overall status
    with open('./{}/{}_status.log'.format(U_FILENAME, U_FILENAME), 'w') as f:
        f.truncate()
        #f.write('t_passed :: t_capture::   t_blob ::    t_uvw ::    t_pqr ::    t_xyz :: distance ::    x_pos :: status\n')
        f.write('t_passed :: distance ::    x_pos :: status\n')

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

'''
def log_status(t_passed, t_capture, t_blob, t_uvw, t_pqr, t_xyz, distance, x_pos, status):
    with open('./{}/{}_status.log'.format(U_FILENAME, U_FILENAME), 'a') as f:
        f.write(
            '  {:6.3f} ::   {:6.3f} ::   {:6.3f} ::   {:6.3f} ::   {:6.3f} ::   {:6.3f} ::     {:4.0f} ::     {:4.0f} ::   {}\n'.format(t_passed, t_capture, t_blob, t_uvw, t_pqr, t_xyz, distance, x_pos, status
                )
            )
'''

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
    right = vision.xyz_r
    left = vision.xyz_l

    if not right.size and not left.size:
        print('move up')
        dorsal.off()
        return

    if not right.size:
        right = left
    elif not left.size:
        left = right

    if ((right[2, 1] + left[2, 1]) / 2) > 5:
        print('move down')
        dorsal.on()
    elif ((right[2, 1] + left[2, 1]) / 2) < -5:
        print('move up')
        dorsal.off()

    '''
    # pressure sensor takeover. is not distance invariant, so start only when orbiting at fixed distance
    if status == 'orbit' and abs(((blobs_right[1, 0] + blobs_left[1, 0]) / 2) - (U_CAM_MRES / 2)) < 15:
        depth_sensor.update()
        global lock_depth
        lock_depth = depth_sensor.depth_mm # i.e., lock_depth not false anymore
        global depth_ctrl
        depth_ctrl = False
    '''

def depth_ctrl_from_depthsensor(thresh=5):
    depth_sensor.update()

    if depth_sensor.depth_mm > (lock_depth + thresh):
        dorsal.off()
    elif depth_sensor.depth_mm < (lock_depth - thresh):
        dorsal.on()

def home():
    blobs_right = vision._blob_r.blobs
    blobs_left = vision._blob_l.blobs

    # blob in front
    if blobs_right.size and blobs_left.size:
        caudal.on()
        
        '''
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
        '''

    # blob to the right
    elif blobs_right.size:
        freq_l = 2 + 8 * blobs_right[1, 0] / U_CAM_NRES
        pecto_l.set_frequency(abs(freq_l))

        #print('turn cw')
        pecto_l.on()
        pecto_r.off()

        if (blobs_right[1, 0] < 75):
            caudal.on()
        else:
            caudal.off()

    # blob to the left
    elif blobs_left.size:
        freq_r = 2 + 8 * (U_CAM_NRES - blobs_left[1, 0]) / U_CAM_NRES
        pecto_r.set_frequency(abs(freq_r))

        #print('turn ccw')
        pecto_r.on()
        pecto_l.off()

        if (blobs_left[1, 0] > (U_CAM_NRES - 75)):
            caudal.on()
        else:
            caudal.off()

    # blob behind or lost
    else:
        #print('lost blob, wait')
        pecto_r.set_frequency(4)
        pecto_r.on()
        pecto_l.off()
        caudal.off()

def transition():
    caudal.off()
    pecto_l.off()
    pecto_r.set_frequency(8)
    pecto_r.on()

    if vision.xyz_r.size:
        heading = np.arctan2(vision.xyz_r[1, 1], vision.xyz_r[0, 1]) * 180 / pi
    else:
        heading = 0

    #print(heading)

    if heading > 45:
        pecto_r.off()
        global status
        status = 'orbit'

def orbit(target_dist):
    dist = np.linalg.norm(vision.xyz_r[:, 1]) # 2D, ignoring z
    x_pos = vision.xyz_r[0, 1]
    if dist > target_dist:
        if x_pos > 0:
            #print('fwd')
            caudal.set_frequency(2.2)
            pecto_r.off()
            pecto_l.off()
        else:
            #print('cw')
            caudal.set_frequency(1.4)
            pecto_l.set_frequency(8)
            pecto_l.on()
            pecto_r.off()
    else:
        if x_pos > 0:
            #print('ccw')
            caudal.set_frequency(2.2)
            pecto_r.set_frequency(8)
            pecto_r.on()
            pecto_l.off()
        else:
            #print('fwd')
            caudal.set_frequency(2.2)
            pecto_r.off()
            pecto_l.off()

def main(run_time=60, target_dist=500): # [s, mm]
    t_start = time.time()
    
    while time.time() - t_start < run_time:
        # check environment and find blob centroids of leds
        vision.update()

        # control depth
        if depth_ctrl:
            depth_ctrl_from_cam()
        elif lock_depth:
            depth_ctrl_from_depthsensor()

        # orbit if 2 blobs are visible
        if vision.xyz_r.size:
            dist = np.linalg.norm(vision.xyz_r[:, 1])
            x_pos = vision.xyz_r[0, 1]
        elif vision.xyz_l.size:
            dist = np.linalg.norm(vision.xyz_l[:, 1])
            x_pos = vision.xyz_l[0, 1]
        else:
            caudal.off()
            pecto_r.off()
            pecto_l.off()
            dist = 9999
            x_pos = 9999

        # act based on status
        global status
        if status == 'home':
            dist_filtered = ema.update_ema(dist)
            if dist_filtered < target_dist * 1.6:
                status = 'transition'
            else:
                #print(home)
                home()
        elif status == 'transition':
            transition()
        elif status == 'orbit':
            caudal.on()
            orbit(target_dist)    

        #print(status)

        # log status and centroids
        t_passed = time.time() - t_start
        log_status(t_passed, dist, x_pos, status)
        log_centroids(round(t_passed, 3), 'right')
        log_centroids(round(t_passed, 3), 'left')


# homing plus orbiting, 2D or 3D
status = 'home' # ['home', 'transition', 'orbit']
depth_ctrl = True # 2D or 3D
lock_depth = False # 320 # use depth sensor once at target depth, set to mm value

caudal = Fin(U_FIN_C1, U_FIN_C2, 2.2) # freq, [Hz]
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 8) # freq, [Hz]
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 8) # freq, [Hz]
leds = LEDS()
vision = Vision()
depth_sensor = DepthSensor()
ema = EMA(0.3)

time.sleep(12)
initialize()
leds.on()
main(120, 400) # run time, target distance
leds.off()
terminate()
