"""Makes BlueBot aggregate.
"""
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import os
import time
import threading
import numpy as np
from math import *
from picamera import PiCamera

from lib_utils import *
from lib_photodiode import Photodiode
from lib_depthsensor import DepthSensor
from lib_fin import Fin
from lib_leds import LEDS
from lib_vision import Vision

os.makedirs('./data/{}/'.format(U_FILENAME))

'''
from PIL import Image
os.makedirs('./data/{}/'.format(U_FILENAME))
os.makedirs('./data/{}/imgs_r'.format(U_FILENAME))
os.makedirs('./data/{}/imgs_l'.format(U_FILENAME))
'''

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
    thresh_photodiode = 40 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        photodiode.update()

    time.sleep(3)
    t_blink = time.time()
    for blink in range(U_UUID):
        leds.on()
        time.sleep(0.2)
        leds.off()
        time.sleep(0.2)
    elapsed_time = time.time() - t_blink
    sleep_time = 10 - elapsed_time
    time.sleep(sleep_time)

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

def log_status(t_passed, depth_mm):
    """Logs the overall status of BlueBot
    """
    with open('./data/{}/{}_status.log'.format(U_FILENAME, U_UUID), 'a') as f:
        f.write('{:.2f},{}\n'.format(t_passed, depth_mm))

def depth_ctrl_from_cam(target):
    """Controls the diving depth to stay level with an observed object using both cameras. Swithes to depth sensor based depth control when on level with object.

    The "pitch" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(r, sqrt(p^2 + q^2)). A positive angle switches the dorsal fin on to move down. A negative angles switches the dorsal fin off to move up.
    
    Returns:
        (): Floats to the surface if no object observed
    """
    if not target.size:
        dorsal.off()
        return

    pitch_range = 2 # abs(pitch) below which dorsal fin is not controlled 

    pitch = np.arctan2(target[2], sqrt(target[0]**2 + target[1]**2)) * 180 / pi

    if pitch > pitch_range:
        dorsal.on()
    elif pitch < -pitch_range:
        dorsal.off()

def center(sign):
    right = vision.pqr_r
    left = vision.pqr_l

    # compute center
    if right.size and left.size:
        center = sign/(right.shape[1] + left.shape[1]) * (np.sum(right, axis=1) + np.sum(left, axis=1))
        return center
    elif right.size:
        center = sign/right.shape[1] * np.sum(right, axis=1)
        return center
    elif left.size:
        center = sign/left.shape[1] * np.sum(left, axis=1)
        return center
    else:
        return np.zeros(0)

def home(target):
    """Controls the pectoral fins to follow an object using both cameras

    The "heading" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(r, sqrt(q^2 + p^2)). A positive angle switches the pectoral left fin on turn clockwise. A negative angles switches the pectoral right fin on to turn counterclockwise.
    
    Returns:
        (): Floats to the surface and turns on the spot if no object observed
    """
    caudal_range = 50 # abs(heading) below which caudal fin is switched on

    # blob behind or lost
    if not target.size:
        #print('cant see blob')
        pecto_r.set_frequency(6)
        pecto_r.on()
        pecto_l.off()
        caudal.off()
        return

    # calculate heading
    heading = np.arctan2(target[1], target[0]) * 180 / pi

    # target to the right
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

    # target to the left
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

def main(run_time=60):
    iteration = 0
    t_start = time.time()
    
    while time.time() - t_start < run_time:
        try:
            vision.update()
        except:
            continue

        '''
        # SAVE IMAGES
        temp = Image.fromarray(vision._cam_r.img)
        temp.save('./data/{}/imgs_r/img_r_{}.png'.format(U_FILENAME,iteration))
        temp = Image.fromarray(vision._cam_l.img)
        temp.save('./data/{}/imgs_l/img_l_{}.png'.format(U_FILENAME,iteration))
        iteration += 1
        '''

        time_passed = time.time() - t_start
        if (time_passed < run_time/4) or (1/2*run_time <= time_passed < 3/4*run_time):
            sign = -1
        else:
            sign = 1

        target = center(sign)
        home(target)
        depth_ctrl_from_cam(target)

        depth_sensor.update()
        depth_mm = max(0, (depth_sensor.pressure_mbar - surface_pressure) * 10.197162129779)
        log_status(time_passed, depth_mm)


max_centroids = 3 # maximum expected centroids in environment

caudal = Fin(U_FIN_C1, U_FIN_C2, 3) # freq, [Hz]
dorsal = Fin(U_FIN_D1, U_FIN_D2, 5) # freq, [Hz]
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 8) # freq, [Hz]
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 8) # freq, [Hz]
photodiode = Photodiode()
leds = LEDS()
vision = Vision(max_centroids, True)
vision._cam_r.colorbot_settings()
vision._cam_l.colorbot_settings()
depth_sensor = DepthSensor()

depth_sensor.update()
surface_pressure = depth_sensor.pressure_mbar

initialize()
idle()
main(80) # run time
terminate()
