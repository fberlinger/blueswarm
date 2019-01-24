"""Makes BlueBot lead another BlueBot.

Conscious random walk checks for distance to follower and adjusts speed accordingly.

Attributes:
    caudal: Fin object for caudal fin
    depth_sensor: DepthSensor object
    dorsal: Fin object for dorsal fin
    ema: EMA filter object
    leds: LED object
    max_centroids (int): Number of LEDs that are expected in environment
    pecto_l: Fin object for pectoral left fin
    pecto_r: Fin object for pectoral right fin
    vision: Vision object
"""
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import os
import csv
import time
import threading
import numpy as np
import random
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
    """Initializes all threads which are running fins and a logger instance for the overall status
    """
    threading.Thread(target=caudal.run).start()
    threading.Thread(target=dorsal.run).start()
    threading.Thread(target=pecto_l.run).start()
    threading.Thread(target=pecto_r.run).start()

    # logger instance for overall status
    with open('./{}/{}_status.log'.format(U_FILENAME, U_FILENAME), 'w') as f:
        f.truncate()
        #f.write('t_passed :: t_capture::   t_blob ::    t_uvw ::    t_pqr ::    t_xyz :: distance ::    heading :: status\n')
        f.write('t_passed :: distance ::  heading :: status\n')

    leds.on()
    time.sleep(1)
    leds.off()
    time.sleep(1)

def terminate():
    """Terminates all threads which are running fins
    """
    caudal.terminate()
    dorsal.terminate()
    pecto_l.terminate()
    pecto_r.terminate()

    leds.on()
    time.sleep(1)
    leds.off()

    GPIO.cleanup()

'''
def log_status(t_passed, t_capture, t_blob, t_uvw, t_pqr, t_xyz, distance, heading, status):
    with open('./{}/{}_status.log'.format(U_FILENAME, U_FILENAME), 'a') as f:
        f.write(
            '  {:6.3f} ::   {:6.3f} ::   {:6.3f} ::   {:6.3f} ::   {:6.3f} ::   {:6.3f} ::     {:4.0f} ::     {:4.0f} ::   {}\n'.format(t_passed, t_capture, t_blob, t_uvw, t_pqr, t_xyz, distance, heading, status
                )
            )
'''

def log_status(t_passed, distance, heading, status):
    """Logs the overall status of BlueBot
    
    Args:
        t_passed (float): Time since the beginning of the experiment, [s]
        distance (float): Distance to LED pair, [mm]
        heading (float): x-position of an LED pair, [mm]
        status (string): Status in the finite state machine
    """
    with open('./{}/{}_status.log'.format(U_FILENAME, U_FILENAME), 'a') as f:
        f.write(
            '  {:6.3f} ::     {:4.0f} ::     {:4.0f} ::   {}\n'.format(t_passed, distance, heading, status
                )
            )

def log_centroids(t_passed, side, max_centroids):
    """Logs the (xyz) centroid positions observed in the last vision.update. If fewer than max_centroids are observed, remaining values will be padded with U_CAM_NRES.
    
    Args:
        t_passed (float): Time since the beginning of the experiment, [s]
        side (string): Right or left robot side
        max_centroids (int): Number of LEDs that are expected in environment
    """
    if (side == 'right'):
        centroids = vision.xyz_r
    elif (side == 'left'):
        centroids = vision.xyz_l

    centroid_list = U_CAM_NRES * np.ones((3, 3 * max_centroids)) # non-blob entries are set to U_CAM_NRES
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

def depth_ctrl_from_depthsensor(target_depth=400, thresh=2):
    """Controls the diving depth to a preset level
    
    Args:
        target_depth (int, optional): Nominal diving depth
        thresh (int, optional): Threshold below which dorsal fin is not controlled, [mm]
    """
    depth_sensor.update()

    if depth_sensor.depth_mm > (target_depth + thresh):
        dorsal.off()
    elif depth_sensor.depth_mm < (target_depth - thresh):
        dorsal.on()

def move(self):
    """Conscious random walk. Wait, whenever follower is too far away. Otherwise, go forward, up/down, and turn left/right with respective probabilities.
    """
    target_depth = 600

    global state_switch # [0, 1] = [spin, forward]
    global state_dir # [0, 1] = [clockwise, counterclockwise]

    prob_switch = 0.2
    prob_dir = 0.5
    prob_depth = 0.05

    if random.random() <= prob_switch:
        state_switch = 1 - state_switch

    if state_switch == 0:
        caudal.off()
        if state_dir == 0:
            pecto_r.off()
            pecto_l.on() 
        else:
            pecto_l.off()
            pecto_r.on()
    else:
        pecto_r.off()
        pecto_l.off()       
        caudal.on() 
        if random.random() <= prob_dir:
            state_dir = 1 - state_dir

    if random.random() <= prob_depth:
        target_depth += random.randint(-250, 250)
        target_depth = np.clip(target_depth, 100, 1200)
    depth_ctrl_from_depthsensor(target_depth)

def wait(self):
    """Spin in circle and wait to see follower again
    """
    caudal.off()
    pecto_r.off()
    pecto_l.on()
    depth_ctrl_from_depthsensor(600)

def main(max_centroids, run_time=60, target_dist=500):
    """Runs vision update, depth control, status-based action, and logging iteratively
    
    Args:
        max_centroids (int): Maximum expected centroids in environment
        run_time (int, optional): Experiment time [s]
        target_dist (int, optional): Max distance to follower, [mm]
    """
    t_start = time.time()
    
    while time.time() - t_start < run_time:
        # check environment and find blob centroids of leds
        try:
            vision.update()
        except:
            continue

        # lead if 2 blobs are visible
        if vision.xyz_r.size:
            dist = np.linalg.norm(vision.xyz_r[:2, 0])
        elif vision.xyz_l.size:
            dist = np.linalg.norm(vision.xyz_l[:2, 0])
        else:
        	wait()
        	continue

        # act based on distance to follower
        dist_filtered = ema.update_ema(dist)
        if dist_filtered < target_dist:
            move()
        else:
            wait()

        # log status and centroids
        t_passed = time.time() - t_start
        log_status(t_passed, dist, heading, status)
        log_centroids(round(t_passed, 3), 'right', max_centroids)
        log_centroids(round(t_passed, 3), 'left', max_centroids)


max_centroids = 2 # maximum expected centroids in environment

caudal = Fin(U_FIN_C1, U_FIN_C2, 2.0) # freq, [Hz]
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 8) # freq, [Hz]
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 8) # freq, [Hz]
leds = LEDS()
vision = Vision(max_centroids)
depth_sensor = DepthSensor()
ema = EMA(0.3)

time.sleep(12)
initialize()
leds.on()
main(max_centroids, 120, 500) # run time, target distance
leds.off()
terminate()
