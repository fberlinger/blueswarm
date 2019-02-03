"""Makes BlueBot follow the light, e.g., a stick with an LED on its tip, moved by a visitor.

Can be extended to a BlueBot following another BlueBot.

Contains generic vision based functions that can be used elsewhere including homing and depth control. Also contains logger functions.

Attributes:
    caudal (): Fin object for caudal fin
    depth_sensor (): DepthSensor object
    dorsal (): Fin object for dorsal fin
    leds (): LED object
    pecto_l (): Fin object for pectoral left fin
    pecto_r (): Fin object for pectoral right fin
    vision (): Vision object
"""
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time
import threading
import numpy as np
from math import *
from picamera import PiCamera

from lib_utils import *
from lib_photodiode import Photodiode
from lib_fin import Fin
from lib_leds import LEDS
from lib_vision import Vision
from lib_ema import EMA


def initialize():
    """Initializes all threads which are running fins and a logger instance for the overall status
    """
    threading.Thread(target=caudal.run).start()
    threading.Thread(target=dorsal.run).start()
    threading.Thread(target=pecto_l.run).start()
    threading.Thread(target=pecto_r.run).start()

    leds.on()
    time.sleep(1)
    leds.off()

def idle():
    """Waiting for starting signal
    """
    thresh_photodiode = 50 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        photodiode.update()

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

def depth_ctrl_from_cam():
    """Controls the diving depth to stay level with an observed object using both cameras

    The "pitch" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(r, sqrt(p^2 + q^2)). A positive angle switches the dorsal fin on to move down. A negative angles switches the dorsal fin off to move up.
    
    Returns:
        (): Floats to the surface if no object observed
    """
    right = vision.pqr_r
    left = vision.pqr_l

    if not right.size and not left.size:
        #print('cant see blob')
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
    #print(pitch)

    if pitch > 1:
        #print('move down')
        dorsal.on()
    elif pitch < -1:
        #print('move up')
        dorsal.off()

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
    else:
        freq_r = 5 + 5 * abs(heading) / 180
        pecto_r.set_frequency(freq_r)

        #print('turn ccw')
        pecto_r.on()
        pecto_l.off()

        if heading > -caudal_range:
            caudal.on()
        else:
            caudal.off()

def wait():
    """Spin in circle and wait to see follower again
    """
    caudal.off()
    pecto_r.on()
    pecto_l.on()
    depth_ctrl_from_cam()

def main(max_centroids, run_time=60, target_dist=500):
    """Runs vision update, depth control, homing, and logging iteratively
    
    Args:
        max_centroids (int): Maximum expected centroids in environment
        run_time (int, optional): Experiment time [s]
    """
    t_start = time.clock()
    
    while time.clock() - t_start < run_time:
        # check environment and find blob centroids of leds
        try:
            vision.update()
        except:
            continue

        # follow if 2 blobs are visible
        if vision.xyz_r.size:
            dist = np.linalg.norm(vision.xyz_r[:2, 0])
        elif vision.xyz_l.size:
            dist = np.linalg.norm(vision.xyz_l[:2, 0])
        else:
            wait()
            continue

        # control depth
        depth_ctrl_from_cam()

        # act based on distance to follower
        dist_filtered = ema.update_ema(dist)
        if dist_filtered < target_dist:
            wait()
        else:
            home()

        # log status and centroids
        t_passed = time.clock() - t_start
        #log_status(t_passed, dist, x_pos, status)
        #log_centroids(round(t_passed, 3), 'right', max_centroids)
        #log_centroids(round(t_passed, 3), 'left', max_centroids)

max_centroids = 2 # maximum expected centroids in environment

caudal = Fin(U_FIN_C1, U_FIN_C2, 2.2) # freq, [Hz]
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 8) # freq, [Hz]
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 8) # freq, [Hz]
leds = LEDS()
photodiode = Photodiode()
vision = Vision(max_centroids)
ema = EMA(0.3)

initialize()
idle()
time.sleep(1)
leds.on()
main(max_centroids, 180, 130) # run time
leds.off()
terminate()
