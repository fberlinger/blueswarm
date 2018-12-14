"""Makes BlueBot around two vertically stacked lights

Contains generic vision based functions that can be used elsewhere including homing and depth control. Also contains depth control using the depth sensor and logger functions.

Attributes:
    caudal (): Fin object for caudal fin
    depth_ctrl (bool): Depth control from camera, [y/n]
    depth_sensor (): DepthSensor object
    dorsal (): Fin object for dorsal fin
    ema (): EMA filter object
    leds (): LED object
    lock_depth (int): Depth control from depth sensor, 0=false, int=target_depth
    pecto_l (): Fin object for pectoral left fin
    pecto_r (): Fin object for pectoral right fin
    status (str): BlueBot status in finite state machine
    vision (): Vision object
"""
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

def depth_ctrl_from_cam():
    """Controls the diving depth to stay level with an observed object using both cameras. Swithes to depth sensor based depth control when on level with object.

    The "pitch" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(r, sqrt(p^2 + q^2)). A positive angle switches the dorsal fin on to move down. A negative angles switches the dorsal fin off to move up.
    
    Returns:
        (): Floats to the surface if no object observed
    """
    pitch_range = 1 # abs(pitch) below which dorsal fin is not controlled 

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

    if pitch > pitch_range:
        print('move down')
        dorsal.on()
    elif pitch < -pitch_range:
        print('move up')
        dorsal.off()

    # pressure sensor takeover. is not distance invariant, so start only when orbiting at fixed distance
    if status == 'orbit' and abs(pitch) < pitch_range:
        depth_sensor.update()
        global lock_depth
        lock_depth = depth_sensor.depth_mm # i.e., lock_depth not false anymore
        global depth_ctrl
        depth_ctrl = False

def depth_ctrl_from_depthsensor(thresh=2):
    """Controls the diving depth to a preset level
    
    Args:
        thresh (int, optional): Threshold below which dorsal fin is not controlled, [mm]
    """
    depth_sensor.update()

    if depth_sensor.depth_mm > (lock_depth + thresh):
        dorsal.off()
    elif depth_sensor.depth_mm < (lock_depth - thresh):
        dorsal.on()

def home():
    """Controls the pectoral fins to follow an object using both cameras

    The "heading" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(r, sqrt(q^2 + p^2)). A positive angle switches the pectoral left fin on turn clockwise. A negative angles switches the pectoral right fin on to turn counterclockwise.
    
    Returns:
        (): Floats to the surface and turns on the spot if no object observed
    """
    caudal_range = 20 # abs(heading) below which caudal fin is swithed on

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

    # calculate headings
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

        if heading < caudal_range:
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

        if heading > -caudal_range:
            caudal.on()
        else:
            caudal.off()

def transition():
    """Transitions between homing and orbiting. Uses pectoral right fin to align tangentially with the orbit.
    """
    caudal.off()
    pecto_l.off()
    pecto_r.set_frequency(8)
    pecto_r.on()

    right = vision.pqr_r

    try:
        heading = np.arctan2(right[1, 0], right[0, 0]) * 180 / pi
    except:
        return

    if heading > 45:
        pecto_r.off()
        global status
        status = 'orbit'

def orbit(target_dist):
    """Orbits an object, e.g. two vertically stacked LEDs, at a predefined radius

    Uses four zones to control the orbit with pectoral and caudal fins. The problem is reduced to 2D and depth control is handled separately.

    Could make fin frequencies dependent on distance and heading, i.e., use proportianl control.
    
    Args:
        target_dist (int): Target orbiting radius, [mm]
    """
    try:
        dist = np.linalg.norm(vision.xyz_r[:2, 0]) # 2D, ignoring z
        heading = np.arctan2(vision.pqr_r[1, 0], vision.pqr_r[0, 0]) * 180 / pi
    except:
        return

    if dist > target_dist:
        if heading < 90:
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
        if heading < 90:
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

def main(max_centroids, run_time=60, target_dist=500):
    """Runs vision update, depth control, status-based action, and logging iteratively
    
    Args:
        max_centroids (int): Maximum expected centroids in environment
        run_time (int, optional): Experiment time [s]
        target_dist (int, optional): Orbit radius, [mm]
    """
    t_start = time.time()
    
    while time.time() - t_start < run_time:
        # check environment and find blob centroids of leds
        try:
            vision.update()
        except:
            continue

        # control depth
        if depth_ctrl:
            depth_ctrl_from_cam()
        elif lock_depth:
            depth_ctrl_from_depthsensor()

        # orbit if 2 blobs are visible
        if vision.xyz_r.size:
            dist = np.linalg.norm(vision.xyz_r[:1, 0])
            heading = np.arctan2(vision.pqr_r[1, 0], vision.pqr_r[0, 0]) * 180 / pi
        elif vision.xyz_l.size:
            dist = np.linalg.norm(vision.xyz_l[:1, 0])
            heading = np.arctan2(vision.pqr_l[1, 0], vision.pqr_l[0, 0]) * 180 / pi
        else:
            caudal.off()
            pecto_r.off()
            pecto_l.off()
            dist = 9999
            heading = 9999

        # act based on status
        global status
        if status == 'home':
            dist_filtered = ema.update_ema(dist)
            if dist_filtered < target_dist * 1.6:
                status = 'transition'
            else:
                home()
        elif status == 'transition':
            transition()
        elif status == 'orbit':
            caudal.on()
            orbit(target_dist)

        # log status and centroids
        t_passed = time.time() - t_start
        log_status(t_passed, dist, heading, status)
        log_centroids(round(t_passed, 3), 'right', max_centroids)
        log_centroids(round(t_passed, 3), 'left', max_centroids)


# homing plus orbiting, 2D or 3D
status = 'home' # ['home', 'transition', 'orbit']
depth_ctrl = True # 2D or 3D, [False, True]
lock_depth = False # use depth sensor once at target depth, set to mm value

max_centroids = 2 # maximum expected centroids in environment

caudal = Fin(U_FIN_C1, U_FIN_C2, 2.2) # freq, [Hz]
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
main(max_centroids, 120, 400) # run time, target distance
leds.off()
terminate()
