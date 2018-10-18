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
from lib_fin import Fin
from lib_leds import LEDS
from lib_vision import Vision
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

def log_centroids(t_passed, side):
    if (side == 'right'):
        centroids = vision.pqr_r
    elif (side == 'left'):
        centroids = vision.pqr_l

    max_centroids = 3 * 5
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


def main(run_time=60, target_dist=500): # [s, mm]
    t_start = time.time()
    
    while time.time() - t_start < run_time:
        # check environment and find blob centroids of leds
        vision.update()

        # log status and centroids
        distance = 1000
        t_passed = time.time() - t_start
        log_status(t_passed, distance, status)
        log_centroids(round(t_passed, 3), 'right')
        log_centroids(round(t_passed, 3), 'left')


# homing plus orbiting, 2D or 3D
status = 'home' # ['home', 'orbit']
depth_ctrl = True # 2D or 3D
lock_depth = False # use depth sensor once at target depth, set to mm value

caudal = Fin(U_FIN_C1, U_FIN_C2, 5) # freq
dorsal = Fin(U_FIN_D1, U_FIN_D2, 5) # freq
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 6) # freq
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 6) # freq
leds = LEDS()
vision = Vision()
depth_sensor = DepthSensor()

time.sleep(0.1)
initialize()
main(5) # run time
terminate()