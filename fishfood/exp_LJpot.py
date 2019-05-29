

import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import os
import time
import threading
import numpy as np
from picamera import PiCamera

from lib_utils import *
from lib_photodiode import Photodiode
from lib_depthsensor import DepthSensor
from lib_fin import Fin
from lib_leds import LEDS
from lib_vision import Vision

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
        f.write('t_passed ::     depth_mm ::  target_dist ::     target_x ::     target_y ::     target_z :: no_neighbors\n') # 13 long

    leds.on()
    time.sleep(1)
    leds.off()

def idle():
    """Waiting for starting signal, then signal UUID
    """
    thresh_photodiode = 50 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        photodiode.update()
        time.sleep(0.1)

    t_start = time.time()
    time.sleep(6)
    for blink in range(U_UUID):
        leds.on()
        time.sleep(0.2)
        leds.off()
        time.sleep(0.2)
    elapsed_time = time.time() - t_start
    sleep_time = 12 - elapsed_time
    time.sleep(sleep_time) # wait such that all robots leave idle before LEDs are on

    return t_start

def terminate():
    """Terminates all threads which are running fins
    """
    caudal.terminate()
    dorsal.terminate()
    pecto_l.terminate()
    pecto_r.terminate()

    time.sleep(1)
    leds.on()
    time.sleep(1)
    leds.off()

    GPIO.cleanup()

def log_status(t_passed, depth_mm, target_dist, target_x, target_y, target_z, no_neighbors):
    """Logs the overall status of BlueBot
    
    Args:
        t_passed (float): Time since the beginning of the experiment, [s]
        distance (float): Distance to LED pair, [mm]
        heading (float): x-position of an LED pair, [mm]
        status (string): Status in the finite state machine
    """
    with open('./{}/{}_status.log'.format(U_FILENAME, U_FILENAME), 'a') as f:
        f.write('{:8.1f} :: {:12.0f} :: {:12.0f} :: {:12.3f} :: {:12.3f} :: {:12.3f} :: {:12.0f}\n'.format(t_passed, depth_mm, target_dist, target_x, target_y, target_z, no_neighbors))

def avoid_duplicates_by_angle():
    """Use right and left cameras just up to the xz-plane such that the overlapping camera range disappears and there are no duplicates.

    Returns:
        tuple: all_blobs (that are valid, i.e. not duplicates) and their all_angles
    """
    right = np.transpose(vision.pqr_r)
    left = np.transpose(vision.pqr_l)

    all_blobs = np.empty((3,0))
    all_angles = np.empty(0)

    for r in right:
        angle = np.arctan2(r[1], r[0]) * 180 / pi
        if angle > -5:
            all_blobs = np.append(all_blobs, [[r[0]], [r[1]], [r[2]]], axis=1)
            all_angles = np.append(all_angles, angle)

    for l in left:
        angle = np.arctan2(l[1], l[0]) * 180 / pi
        if angle < 5:
            all_blobs = np.append(all_blobs, [[l[0]], [l[1]], [l[2]]], axis=1)
            all_angles = np.append(all_angles, angle)

    return (all_blobs, all_angles)

def parse(all_blobs, all_angles):
    """Assigns duos of blobs to single robots

    Idea: Sort all blobs by the angles/directions they are coming from. Pair duos of blobs that have most similar angles.
    
    Args:
        all_blobs (np.array): all valid blobs from both images
        all_angles (np.array): all angles in xy-plane of all valid blobs
    
    Returns:
        tuple: set of neighbors and dict with their relative positions
    """
    neighbors = set()
    rel_pos = {}

    no_blobs = len(all_angles)
    if no_blobs < 2: # 0 robots
        return (neighbors, rel_pos)

    sorted_indices = np.argsort(all_angles)
    angle_thresh = 5 # below which 2 blobs are considered a duo

    i = 0 # blob_ind
    neighbor_ind = 0
    while i < no_blobs-1: # iterate through all blobs and fill dict
        # if 2 blobs are too far apart, ignore first one and check next 2
        dangle = abs(all_angles[sorted_indices[i+1]] - all_angles[sorted_indices[i]])
        if dangle > angle_thresh:
            i += 1
            continue

        # else, add neighbor with averaged xyz coordinates
        b1 = all_blobs[:,sorted_indices[i]]
        b2 = all_blobs[:,sorted_indices[i+1]]

        pqr = np.transpose(np.vstack((b1, b2)))
        xyz = vision._pqr_to_xyz(pqr)

        neighbors.add(neighbor_ind)
        rel_pos[neighbor_ind] = (xyz[:,0] + xyz[:,1]) / 2

        i += 2
        neighbor_ind += 1

    return(neighbors, rel_pos)

def lj_force(neighbors, rel_pos):
    """Derives the Lennard-Jones potential and force based on the relative positions of all neighbors and the desired target_dist to neighbors. The force is a gain factor, attracting or repelling a fish from a neighbor. The center is a point in space toward which the fish will move, based on the sum of all weighted neighbor positions.

    Args:
        neighbors (set): Visible neighbors
        rel_pos (dict): Relative positions of visible neighbors

    Returns:
        np.array: Weighted 3D direction based on visible neighbors
    """
    center = np.zeros((3,))
    magn = 0

    if not neighbors:
        return (center, magn)

    # (a=12,b=6) is standard and ratio has to be 2:1, lower numbers for less aggressive repulsion, e.g. (a=6,b=3)
    a = 8
    b = 4
    # epsilon and gamma are only scaling factors and without effect after normalization
    epsilon = 100 # depth of potential well, V_LJ(r_target) = epsilon
    gamma = 1 # force gain
    r_target = target_dist
    r_const = r_target + 4*BL #xx

    for neighbor in neighbors:
        r = np.clip(np.linalg.norm(rel_pos[neighbor]), 0.001, r_const)
        f_lj = -gamma * epsilon /r * (a * (r_target / r)**a - 2 * b * (r_target / r)**b)
        center += f_lj * rel_pos[neighbor]

    center /= len(neighbors)
    magn = np.linalg.norm(center) # normalize
    center /= magn # normalize

    return (center, magn)

def home(target, magnitude):
    """Controls the pectoral fins to follow an object using both cameras

    The "heading" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(r, sqrt(q^2 + p^2)). A positive angle switches the pectoral left fin on turn clockwise. A negative angles switches the pectoral right fin on to turn counterclockwise.
    
    Returns:
        (): Floats to the surface and turns on the spot if no object observed
    """
    caudal_range = 25 # abs(heading) below which caudal fin is switched on
    freq_c = min(1.5 + 1.5/100 * magnitude, 3)
    caudal.set_frequency(freq_c)

    # blob behind or lost
    if not target.size:
        #pecto_r.set_frequency(6)
        #pecto_r.on()
        pecto_r.off()
        pecto_l.off()
        caudal.off()
        return

    # calculate heading
    heading = np.arctan2(target[1], target[0]) * 180 / pi

    # target behind
    if heading > 155 or heading < -155:
        caudal.off()
        pecto_r.set_frequency(6)
        pecto_r.on()
        pecto_l.set_frequency(6)
        pecto_l.on()

    # target to the right
    elif heading > 0:
        freq_l = 4 + 4 * abs(heading) / 180
        pecto_l.set_frequency(freq_l)

        pecto_l.on()
        pecto_r.off()

        if heading < caudal_range:
            caudal.on()
        else:
            caudal.off()

    # target to the left
    elif heading < 0:
        freq_r = 4 + 4 * abs(heading) / 180
        pecto_r.set_frequency(freq_r)

        pecto_r.on()
        pecto_l.off()

        if heading > -caudal_range:
            caudal.on()
        else:
            caudal.off()

def depth_ctrl_from_cam(target):
    """Controls the diving depth to stay level with an observed object using both cameras. Swithes to depth sensor based depth control when on level with object.

    The "pitch" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(r, sqrt(p^2 + q^2)). A positive angle switches the dorsal fin on to move down. A negative angles switches the dorsal fin off to move up.
    
    Returns:
        (): Floats to the surface if no object observed
    """
    pitch_range = 10 # abs(pitch) below which dorsal fin is not controlled 

    pitch = np.arctan2(target[2], sqrt(target[0]**2 + target[1]**2)) * 180 / pi

    if pitch > pitch_range:
        dorsal.on()
    elif pitch < -pitch_range:
        dorsal.off()


def main(run_time=60):
    t_passed = 0
    iteration = 0
    t_main = time.time() - t_start
    t_change = t_main
    
    while t_passed < run_time + t_main:
        # check environment and find blob centroids of leds
        try:
            vision.update()
        except:
            continue
        # find all valid blobs and their respective angles
        all_blobs, all_angles = avoid_duplicates_by_angle()
        # match blob duos by angle
        neighbors, rel_pos = parse(all_blobs, all_angles)
        # find target move with lj force
        target, magnitude = lj_force(neighbors, rel_pos)
        # move
        home(target, magnitude)
        depth_ctrl_from_cam(target)

        # switch behavior
        if t_passed - t_change > run_time / 4:
            leds.off()
            global target_dist
            if target_dist == upper_thresh:
                target_dist = lower_thresh
            else:
                target_dist = upper_thresh
            t_change = time.time() - t_start
            time.sleep(1.5)
            leds.on()

        # update counters
        t_passed = time.time() - t_start
        iteration += 1

        # log status
        if iteration % 2 == 0: 
            depth_sensor.update()
            depth_mm = max(0, (depth_sensor.pressure_mbar - surface_pressure) * 10.197162129779)
            log_status(t_passed, depth_mm, target_dist, target[0], target[1], target[2], len(neighbors))


BL = 160 # body length, [mm]
max_centroids = 8 # (robots-1)*2, excess centroids are reflections
lower_thresh = 0.5*BL
upper_thresh = 1.5*BL
target_dist = lower_thresh # distance to neighbors, [mm]

caudal = Fin(U_FIN_C1, U_FIN_C2, 2.5) # freq, [Hz]
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 8) # freq, [Hz]
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 8) # freq, [Hz]
photodiode = Photodiode()
leds = LEDS()
vision = Vision(max_centroids)
depth_sensor = DepthSensor()

depth_sensor.update()
surface_pressure = depth_sensor.pressure_mbar

initialize()
t_start = idle()
leds.on()
main(180) # run time, [s]
leds.off()
terminate()
