

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
from lib_leds import LEDS
from lib_vision import Vision


def idle():
    """Waiting for starting signal
    """
    thresh_photodiode = 50 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        print(photodiode.brightness)
        photodiode.update()

    time.sleep(1)

def terminate():
    """Terminates all threads which are running fins
    """
    leds.on()
    time.sleep(1)
    leds.off()

    GPIO.cleanup()

def LED_angle():
    """Test angle manually
    
	Calculates angle for single blob in image, switches leds on/off according to some threshold
    """
    right = np.transpose(vision.pqr_l)
    print(right)
    if not right.size:
        return
    else:
        angle = np.arctan2(right[0, 1], right[0, 0]) * 180 / pi
        if angle > -15:
            leds.off()
        else:
            leds.on()


def remove_duplicates_by_matching():
    """Remove blobs that are observed by both right and left cameras by matching them via the dot product.

    Idea: All blobs in the right image with an angle smaller than 15deg are in the overlapping camera range in front of BlueBot. For each blob in the left image with an angle larger -15deg, check whether its dot product with one of the right blobs is close to 1. A close to 1 dot product means that their rays are coming from the same location, i.e. they are duplicates.

    Note: Not optimized for speed.
    
    Returns:
        tuple: all_blobs (that are valid, i.e. not duplicates) and their all_angles
    """
    # 1) and 2)
    all_blobs = vision.pqr_r
    all_angles = np.zeros(0)
    right = np.transpose(vision.pqr_r)
    left = np.transpose(vision.pqr_l)

    if not right.size and not left.size:
        return (0, 0)

    if not right.size:
        for l in left:
            angle = np.arctan2(l[1], l[0]) * 180 / pi
            all_angles = np.append(all_angles, angle)
        return (vision.pqr_l, all_angles)

    if not left.size:
        for r in right:
            angle = np.arctan2(r[1], r[0]) * 180 / pi
            all_angles = np.append(all_angles, angle)
        return (vision.pqr_r, all_angles)


    for r in right:
        angle = np.arctan2(r[1], r[0]) * 180 / pi
        all_angles = np.append(all_angles, angle)
        cand_r = np.zeros((3,1))
        if angle < 15:
            cand_r = np.append(cand_r, [[r[0]], [r[1]], [r[2]]], axis=1)
    cand_r = np.delete(cand_r, 0, axis=1)
    cand_r = np.transpose(cand_r)

    for l in left:
        angle = np.arctan2(l[1], l[0]) * 180 / pi
        dot = 0
        if angle > -15:
            dl = max(0.001, np.linalg.norm(l))
            for r in cand_r:
                dr = max(0.001, np.linalg.norm(r))
                dot = np.dot(r, l) / (dr * dl)
                print(dot)
                if dot > 0.9:
                    continue
        
        if dot <= 0.9:
            all_blobs = np.append(all_blobs, [[l[0]], [l[1]], [l[2]]], axis=1)
            all_angles = np.append(all_angles, angle)

    # make even number of blobs if necessary
    #if all_blobs.shape[1] % 2:
    #    all_blobs = np.delete(all_blobs, 0, axis=1)
    #    all_angles = np.delete(all_angles, 0)



    return (all_blobs, all_angles)


def avoid_duplicates_by_angle():
    """Use right and left cameras just up to the xz-plane such that the overlapping camera range disappears and there are no duplicates.

    Returns:
        tuple: all_blobs (that are valid, i.e. not duplicates) and their all_angles
    """
    right = np.transpose(vision.pqr_r)
    left = np.transpose(vision.pqr_l)
    
    #all_blobs = np.zeros((3,1))
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

    #all_blobs = np.delete(all_blobs, 0, axis=1) # necessary because cannot create empty 3x1 numpy array

    return (all_blobs, all_angles)

def parse(all_blobs, all_angles):
	"""Assignes duos of blobs to single robots

	Idea: Sort all blobs by the angles/directions they are coming from. Pair duos of blobs that have most similar angles.
	
	Args:
	    all_blobs (np.array): all valid blobs from both images
	    all_angles (np.array): all angles in xy-plane of all valid blobs
	
	Returns:
	    tuple: set of neighbors and dict with their relative positions

	Todo:
	- reduce rel pos to single vector
	- deal with number of robots
	- deal with edge cases, i.e. number of blobs that is not multiple of 2
	"""


    neighbors = set()
    rel_pos = {}

    print(all_blobs)
    print('all_angles {}'.format(all_angles))
    sorted_indices = np.argsort(all_angles)
    print('sorted_indices {}'.format(sorted_indices))

    print('len(all_angles) {}'.format(len(all_angles)))

    if len(all_angles) < 2: # 0 robots
        return (neighbors, rel_pos)

    if len(all_angles) < 6: # 1-2 robots
        for i in range(0, all_blobs.shape[1]-1, 2):
            b1 = all_blobs[:,sorted_indices[i]]
            b2 = all_blobs[:,sorted_indices[i+1]]

            pqr = np.transpose(np.vstack((b1, b2)))
            xyz = vision._pqr_to_xyz(pqr)

            neighbors.add(i/2)
            rel_pos[i/2] = xyz

        return (neighbors, rel_pos)


    # 3)
    # sort by angle, pair by angle
    

    # start where?
    dangle01 = abs(all_angles[sorted_indices[1]] - all_angles[sorted_indices[0]])
    dangle12 = abs(all_angles[sorted_indices[2]] - all_angles[sorted_indices[1]]) 

    if dangle01 < dangle12:
        for i in range(0, all_blobs.shape[1]-1, 2):
            b1 = all_blobs[:,sorted_indices[i]]
            b2 = all_blobs[:,sorted_indices[i+1]]

            pqr = np.transpose(np.append(b1, b2))
            xyz = vision._pqr_to_xyz(pqr)

            neighbors.add(i/2)
            rel_pos[i/2] = xyz
    else:
        b1 = all_blobs[:,sorted_indices[0]]
        b2 = all_blobs[:,sorted_indices[-1]]

        pqr = np.transpose(np.append(b1, b2))
        xyz = vision._pqr_to_xyz(pqr)

        neighbors.add(0)
        rel_pos[0] = xyz

        for i in range(1, all_blobs.shape[1]-2, 2):
            b1 = all_blobs[:,sorted_indices[i]]
            b2 = all_blobs[:,sorted_indices[i+1]]

            pqr = np.transpose(np.append(b1, b2))
            xyz = vision._pqr_to_xyz(pqr)

            neighbors.add((i+1)/2)
            rel_pos[(i+1)/2] = xyz # format of xyz, check

    return(neighbors, rel_pos)



    # 0) check reflections() in lib_blob for inspiration
    # 1) combine left and right images
    # 2) remove duplicates
    # 3) find pqr duos
    # 4) feed them into vision._pqr_to_xyz(self, pqr)
    # 5) get back xyz with distance np.linalg.norm(xyz)

    # I would also generate a set of neighbors and a dict with their xyz positions


def main(run_time=60):
    t_start = time.time()
    
    while time.time() - t_start < run_time:
        # check environment and find blob centroids of leds
        try:
            vision.update()
        except:
            continue

        #LED_angle() # PASSED
        
        #all_blobs, all_angles = remove_duplicates_by_matching() # PASSED

        #print('vision.pqr_r {}'.format(vision.pqr_r))
        #print('vision.pqr_l {}'.format(vision.pqr_l))
        all_blobs, all_angles = avoid_duplicates_by_angle() # PASSED
        #print('all_blobs {}'.format(all_blobs))
        #print('all_angles {}'.format(all_angles))

        neighbors, rel_pos = parse(all_blobs, all_angles) # todo
        print('neighbors {}'.format(neighbors))
        print('rel_pos {}'.format(rel_pos))


max_centroids = 10

photodiode = Photodiode()
leds = LEDS()
vision = Vision(max_centroids)

idle()
leds.on()
main(20) # run time, [s]
leds.off()
terminate()
