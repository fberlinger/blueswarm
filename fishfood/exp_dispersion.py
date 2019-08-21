"""Makes BlueBot aggregate.
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
#from lib_depthsensor import DepthSensor


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

def depth_ctrl_from_cam(target):
    """Controls the diving depth to stay level with an observed object using both cameras. Swithes to depth sensor based depth control when on level with object.

    The "pitch" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(r, sqrt(p^2 + q^2)). A positive angle switches the dorsal fin on to move down. A negative angles switches the dorsal fin off to move up.
    
    Returns:
        (): Floats to the surface if no object observed
    """
    pitch_range = 1 # abs(pitch) below which dorsal fin is not controlled 

    pitch = np.arctan2(target[2], sqrt(target[0]**2 + target[1]**2)) * 180 / pi

    if pitch > pitch_range:
        print('move down')
        dorsal.on()
    elif pitch < -pitch_range:
        print('move up')
        dorsal.off()

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

def center():
    right = vision.pqr_r
    left = vision.pqr_l

    # compute center
    if right.size and left.size:
        center = 1/(right.shape[1] + left.shape[1]) * (np.sum(right, axis=1) + np.sum(left, axis=1))
        return -center
    elif right.size:
        center = 1/right.shape[1] * np.sum(right, axis=1)
        return -center
    elif left.size:
        center = 1/left.shape[1] * np.sum(left, axis=1)
        return -center
    else:
        return np.zeros(0)

def home(target):
    """Controls the pectoral fins to follow an object using both cameras

    The "heading" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(r, sqrt(q^2 + p^2)). A positive angle switches the pectoral left fin on turn clockwise. A negative angles switches the pectoral right fin on to turn counterclockwise.
    
    Returns:
        (): Floats to the surface and turns on the spot if no object observed
    """
    caudal_range = 20 # abs(heading) below which caudal fin is switched on

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

def main(max_centroids, run_time=60):
    t_start = time.time()
    
    while time.time() - t_start < run_time:
        # check environment and find blob centroids of leds
        try:
            vision.update()
        except:
            continue

        target = center()

        #depth_ctrl_from_depthsensor(200)
        #depth_ctrl_from_cam(target)

        home(target)


max_centroids = 4 # maximum expected centroids in environment

caudal = Fin(U_FIN_C1, U_FIN_C2, 3) # freq, [Hz]
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 8) # freq, [Hz]
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 8) # freq, [Hz]
photodiode = Photodiode()
leds = LEDS()
vision = Vision(max_centroids)
#depth_sensor = DepthSensor()

initialize()
idle()
leds.on()
main(max_centroids, 40) # run time
leds.off()
terminate()
