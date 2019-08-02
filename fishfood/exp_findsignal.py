
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

from lib_imgmatch import ImgMatch
from lib_flashdetector import FlashDetector

os.makedirs('../data/{}/'.format(U_FILENAME))

def initialize():
    """Initializes all threads which are running fins and a logger instance for the overall status
    """
    threading.Thread(target=caudal.run).start()
    threading.Thread(target=dorsal.run).start()
    threading.Thread(target=pecto_l.run).start()
    threading.Thread(target=pecto_r.run).start()

    threading.Thread(target=leds.flash).start()

    leds.on()
    time.sleep(1)
    leds.off()

def idle():
    """Waiting for starting signal, then signal UUID
    """
    thresh_photodiode = 50 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        photodiode.update()
    '''
    time.sleep(4)

    t_blink = time.time()
    for blink in range(U_UUID):
        leds.on()
        time.sleep(0.2)
        leds.off()
        time.sleep(0.2)
    elapsed_time = time.time() - t_blink
    sleep_time = 12 - elapsed_time
    time.sleep(sleep_time) # wait such that all robots leave idle before LEDs are on
    '''
    t_start = time.time()

    return t_start

def terminate():
    """Terminates all threads which are running fins
    """
    caudal.terminate()
    dorsal.terminate()
    pecto_l.terminate()
    pecto_r.terminate()

    leds.terminate()

    time.sleep(1)
    leds.on()
    time.sleep(1)
    leds.off()

    GPIO.cleanup()

def log_status(t_passed, status):
    """Logs the overall status of BlueBot
    
    Args:
        t_passed (float): Time since the beginning of the experiment, [s]
        status (string): Status in the finite state machine
    """
    with open('../data/{}/{}_status.log'.format(U_FILENAME, U_UUID), 'a') as f:
        f.write('{},{}\n'.format(t_passed, status))

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

        # else, add 2 blobs
        b1 = all_blobs[:,sorted_indices[i]]
        b2 = all_blobs[:,sorted_indices[i+1]]

        # check for reflections
        ref = 0
        if i+2 < no_blobs-1:
            dangle = abs(all_angles[sorted_indices[i+2]] - all_angles[sorted_indices[i]])
            if dangle < angle_thresh: # a 3rd blob from same direction?
                ref = 1
                b3 = all_blobs[:,sorted_indices[i+2]]
                # who is closest to the surface?
                pitch1 = (np.arctan2(b1[2], sqrt(b1[0]**2 + b1[1]**2)) * 180 / pi, 1)
                pitch2 = (np.arctan2(b2[2], sqrt(b2[0]**2 + b2[1]**2)) * 180 / pi, 2)
                pitch3 = (np.arctan2(b3[2], sqrt(b3[0]**2 + b3[1]**2)) * 180 / pi, 3)
                min_pitch = min(pitch1, pitch2, pitch3)[1] # smallest angle (negative) is closest to surface and will be discarded
                if min_pitch == 1:
                    b1 = b3
                elif min_pitch == 2:
                    b2 = b3

                if i+3 < no_blobs-1:
                    dangle = abs(all_angles[sorted_indices[i+3]] - all_angles[sorted_indices[i]])
                    if dangle < angle_thresh: # a 4th blob from same direction?
                        ref = 2
                        b4 = all_blobs[:,sorted_indices[i+3]]
                        # who is closest to the surface?
                        pitch4 = (np.arctan2(b4[2], sqrt(b4[0]**2 + b4[1]**2)) * 180 / pi, 4)
                        min_pitch = min(pitch1, pitch2, pitch4)[1] # smallest angle (negative)
                        if min_pitch == 1:
                            b1 = b4
                        elif min_pitch == 2:
                            b2 = b4

        # add final duo as neighbor with averaged xyz coordinates
        pqr = np.transpose(np.vstack((b1, b2)))
        xyz = vision._pqr_to_xyz(pqr)

        neighbors.add(neighbor_ind)
        rel_pos[neighbor_ind] = (xyz[:,0] + xyz[:,1]) / 2

        i += 2 + ref
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
    a = 12
    b = 6
    # epsilon and gamma are only scaling factors and without effect after normalization
    epsilon = 100 # depth of potential well, V_LJ(r_target) = epsilon
    gamma = 1 # force gain
    r_target = target_dist
    r_const = r_target + 2*160 #xx

    for neighbor in neighbors:
        r = np.clip(np.linalg.norm(rel_pos[neighbor]), 0.001, r_const)
        f_lj = -gamma * epsilon /r * (a * (r_target / r)**a - 2 * b * (r_target / r)**b)
        center += f_lj * rel_pos[neighbor]

    center /= len(neighbors)
    magn = np.linalg.norm(center) # normalize
    center /= magn # normalize

    return (center, magn)

def home(target, magnitude=0):
    """Controls the pectoral fins to follow an object using both cameras

    The "heading" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(q, p). A positive angle switches the pectoral left fin on turn clockwise. A negative angles switches the pectoral right fin on to turn counterclockwise.
    
    Returns:
        (): Floats to the surface and turns on the spot if no object observed
    """
    caudal_range = 30 # abs(heading) below which caudal fin is switched on
    freq_c = min(2 + 1/250 * magnitude, 3)
    caudal.set_frequency(freq_c)

    # blob behind or lost
    if not target.size:
        pecto_r.set_frequency(6)
        pecto_r.on()
        #pecto_r.off()
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
    pitch_range = 5 # abs(pitch) below which dorsal fin is not controlled 

    pitch = np.arctan2(target[2], sqrt(target[0]**2 + target[1]**2)) * 180 / pi

    if pitch > pitch_range:
        dorsal.on()
    elif pitch < -pitch_range:
        dorsal.off()

def check_flash():
    flash = False
    imgs = [np.empty((U_CAM_MRES, U_CAM_NRES, 3), dtype=np.uint8) for _ in range(no_images)]

    # check right side and return if flash is detected
    vision._cam_r.capture_sequence(imgs)
    outliers = greedymatch.find_outliers(imgs, 'right')
    no_flashes = flashdetector.find_max_flashes(outliers)
    #print('no flashes right {}'.format(no_flashes))

    if no_flashes >= thresh_flash:
        flash = True
        return flash

    # check left side in case right side didn't detect flash
    vision._cam_l.capture_sequence(imgs)
    outliers = greedymatch.find_outliers(imgs, 'left')
    no_flashes = flashdetector.find_max_flashes(outliers)
    #print('no flashes left {}'.format(no_flashes))  
    if no_flashes >= thresh_flash:
        flash = True
    return flash

def check_signal():
    found_source = False
    source_location = 0
    no_pix = 4
    neighborhood = 2 # no_pix = (2*neighborhood+1)**2

    vision._cam_r.redblue_settings()
    vision._cam_r.capture()
    vision._blob_r.detect(vision._cam_r.img)
    colors, blob_ind = vision._blob_r.color_intensities(vision._cam_r.img, no_pix, neighborhood)
    vision._cam_r.std_settings()

    if colors:
        if max(colors) > 1:
            found_source = True
            ind = blob_ind[colors.index(max(colors))]
            mn_r = np.zeros((2,1))
            mn_r[0] = vision._blob_r.blobs[0,ind]
            mn_r[1] = vision._blob_r.blobs[1,ind]
            uvw_r = vision._mn_to_uvw(mn_r)
            pqr_r = vision._uvw_to_pqr_r(uvw_r)
            source_location = pqr_r
            return (found_source, source_location) # bool, np.zeros((3,)) or any list such that source_location[0] is p, [1] is q, [2] is r

    vision._cam_l.redblue_settings()
    vision._cam_l.capture()
    vision._blob_l.detect(vision._cam_l.img)
    colors, blob_ind = vision._blob_l.color_intensities(vision._cam_l.img, no_pix, neighborhood)
    vision._cam_l.std_settings()

    if colors:
        if max(colors) > 1:
            found_source = True
            ind = blob_ind[colors.index(max(colors))]
            mn_l = np.zeros((2,1))
            mn_l[0] = vision._blob_l.blobs[0,ind]
            mn_l[1] = vision._blob_l.blobs[1,ind]
            uvw_l = vision._mn_to_uvw(mn_l)
            pqr_l = vision._uvw_to_pqr_l(uvw_l)
            source_location = pqr_l
    
    return (found_source, source_location) # bool, np.zeros((3,)) or any list such that source_location[0] is p, [1] is q, [2] is r



def main(run_time):
    d_status = {'search': 0, 'approach': 1, 'signal': 2} # status

    while (time.time() - t_start) < run_time:
        # UPDATE STATUS
        found_source, source_location = check_signal()

        if found_source:
            status = 'signal'
        else:
            flash = check_flash()
            if flash:
                status = 'approach'
            else:
                status = 'search'

        # ACT BASED ON STATUS
        if status == 'signal':
            # flash leds
            leds.off()
            leds.flash_on()
            # stay at source
            home(source_location)
            depth_ctrl_from_cam(source_location)
        
        else:
            leds.flash_off()
            global target_dist
            if status == 'approach':
                leds.off()
                target_dist = 50
            elif status == 'search':
                leds.on()
                target_dist = 500

            # move for 4 seconds
            t_move = time.time()
            while (time.time() - t_move) < 4:
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

        #### LOG STATUS ####
        print(status)
        log_status(time.time()-t_start, d_status[status])


max_centroids = 12
no_images = 30
thresh_distance = 5
thresh_flash = 10

greedymatch = ImgMatch(no_images, max_centroids, thresh_distance)
flashdetector = FlashDetector(thresh_distance)

caudal = Fin(U_FIN_C1, U_FIN_C2, 2.5) # freq, [Hz]
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 8) # freq, [Hz]
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 8) # freq, [Hz]
photodiode = Photodiode()
leds = LEDS()
vision = Vision(max_centroids) # 0 disables reflections() in lib_blob
depth_sensor = DepthSensor()

depth_sensor.update()
surface_pressure = depth_sensor.pressure_mbar

initialize()
t_start = idle()
leds.on()
main(90) # run time, [s]
leds.off()
terminate()
