
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

os.makedirs('./data/{}/'.format(U_FILENAME))

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
    '''
    thresh_photodiode = 50 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        photodiode.update()
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

def log_status(t_passed, status, depth_mm, max_colors, no_flashes, found_source, flash):
    """Logs the overall status of BlueBot
    
    Args:
        t_passed (float): Time since the beginning of the experiment, [s]
        status (string): Status in the finite state machine
    """
    with open('./data/{}/{}_status.log'.format(U_FILENAME, U_UUID), 'a') as f:
        f.write('{:.2f}\t{}\t{:.2f}\t{:.2f}\t{}\t{}\t{}\n'.format(t_passed, status, depth_mm, max_colors, no_flashes, found_source, flash))

def center():
    """Summary
    
    Returns:
        TYPE: Description
    """
    right = vision.pqr_r
    left = vision.pqr_l

    # compute center
    if right.size and left.size:
        center = 1/(right.shape[1] + left.shape[1]) * (np.sum(right, axis=1) + np.sum(left, axis=1))
        return center
    elif right.size:
        center = 1/right.shape[1] * np.sum(right, axis=1)
        return center
    elif left.size:
        center = 1/left.shape[1] * np.sum(left, axis=1)
        return center
    else:
        return np.zeros(0)

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
        #pecto_r.set_frequency(6)
        #pecto_r.on()
        pecto_r.off()
        pecto_l.off()
        caudal.off()
        return

    # calculate heading
    heading = np.arctan2(target[1], target[0]) * 180 / pi
    print('heading {}'.format(heading))

    # target behind
    if heading > 155 or heading < -155:
        caudal.off()
        pecto_r.set_frequency(6)
        pecto_r.on()
        pecto_l.set_frequency(6)
        pecto_l.on()

    # target to the right
    elif heading > 0:
        freq_l = 2 + 4 * abs(heading) / 180
        pecto_l.set_frequency(freq_l)

        pecto_l.on()
        pecto_r.off()

        if heading < caudal_range:
            caudal.on()
        else:
            caudal.off()

    # target to the left
    elif heading < 0:
        freq_r = 2 + 4 * abs(heading) / 180
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
    flash_location = np.zeros(0)

    # take images right
    imgs_r = [np.empty((U_CAM_MRES, U_CAM_NRES, 3), dtype=np.uint8) for _ in range(no_images)]
    vision._cam_r.capture_sequence(imgs_r)
    # take images left
    imgs_l = [np.empty((U_CAM_MRES, U_CAM_NRES, 3), dtype=np.uint8) for _ in range(no_images)]
    vision._cam_l.capture_sequence(imgs_l)

    # check right side and return in case of flash
    outliers = greedymatch.find_outliers(imgs_r, 'right')
    no_flashes, location = flashdetector.find_max_flashes(outliers)
    if no_flashes >= thresh_flash:
        flash = True
        uvw_r = vision._mn_to_uvw(location)
        pqr_r = vision._uvw_to_pqr_r(uvw_r)
        flash_location = np.zeros((3,))
        flash_location[0] = pqr_r[0,0]
        flash_location[1] = pqr_r[1,0]
        flash_location[2] = pqr_r[2,0]
        return (flash, flash_location, no_flashes)

    # check left side in case right side didn't detect flash
    outliers = greedymatch.find_outliers(imgs_l, 'left')
    no_flashes, location = flashdetector.find_max_flashes(outliers)
    if no_flashes >= thresh_flash:
        flash = True
        uvw_l = vision._mn_to_uvw(location)
        pqr_l = vision._uvw_to_pqr_l(uvw_l)
        flash_location = np.zeros((3,))
        flash_location[0] = pqr_l[0,0]
        flash_location[1] = pqr_l[1,0]
        flash_location[2] = pqr_l[2,0]
        return (flash, flash_location, no_flashes)

    return (flash, flash_location, no_flashes)

def check_signal():
    found_source = False
    source_location = np.zeros(0)
    no_pix = 4
    neighborhood = 2 # no_pix = (2*neighborhood+1)**2

    vision._cam_r.redblue_settings()
    vision._cam_r.capture()
    vision._blob_r.detect(vision._cam_r.img)
    colors, blob_ind = vision._blob_r.color_intensities(vision._cam_r.img, no_pix, neighborhood)
    vision._cam_r.std_settings()

    if colors:
        if max(colors) > 1.4:
            found_source = True
            ind = blob_ind[colors.index(max(colors))]
            mn_r = np.zeros((2,1))
            mn_r[0] = vision._blob_r.blobs[0,ind]
            mn_r[1] = vision._blob_r.blobs[1,ind]
            print('mn_r {}'.format(mn_r))
            uvw_r = vision._mn_to_uvw(mn_r)
            pqr_r = vision._uvw_to_pqr_r(uvw_r)
            source_location = np.zeros((3,))
            source_location[0] = pqr_r[0,0]
            source_location[1] = pqr_r[1,0]
            source_location[2] = pqr_r[2,0]
            return (found_source, source_location, max(colors)) # bool, np.zeros((3,)) or any list such that source_location[0] is p, [1] is q, [2] is r

    vision._cam_l.redblue_settings()
    vision._cam_l.capture()
    vision._blob_l.detect(vision._cam_l.img)
    colors, blob_ind = vision._blob_l.color_intensities(vision._cam_l.img, no_pix, neighborhood)
    vision._cam_l.std_settings()

    if colors:
        if max(colors) > 1.4:
            found_source = True
            ind = blob_ind[colors.index(max(colors))]
            mn_l = np.zeros((2,1))
            mn_l[0] = vision._blob_l.blobs[0,ind]
            mn_l[1] = vision._blob_l.blobs[1,ind]
            print('mn_l {}'.format(mn_l))
            uvw_l = vision._mn_to_uvw(mn_l)
            pqr_l = vision._uvw_to_pqr_l(uvw_l)
            source_location = np.zeros((3,))
            source_location[0] = pqr_l[0,0]
            source_location[1] = pqr_l[1,0]
            source_location[2] = pqr_l[2,0]
            return (found_source, source_location, max(colors))

    return (found_source, source_location, 9999) # bool, np.zeros((3,)) or any list such that source_location[0] is p, [1] is q, [2] is r



def main(run_time):
    d_status = {'search': 0, 'approach': 1, 'signal': 2} # status
    signal_counter = 0
    status = 'search'

    while (time.time() - t_start) < run_time:
        flash = 9999
        no_flashes = 9999
        # FOUND SOURCE?
        found_source, source_location, max_colors = check_signal()
        if found_source:
            status = 'signal'
            signal_counter = 0
            # flash leds
            leds.off()
            leds.flash_on()
            # stay at source
            home(source_location)
            #depth_ctrl_from_cam(source_location) #xx
        elif status == 'signal' and signal_counter < 3:
            signal_counter += 1
        
        # OBSERVED FLASH?
        else:
            leds.flash_off()
            flash, flash_location, no_flashes = check_flash()
            if flash:
                status = 'approach'
                leds.off()
                home(flash_location)
                #depth_ctrl_from_cam(flash_location) #xx
            else:
                status = 'search'
                leds.on()
                try:
                    vision.update()
                except:
                    continue
                target = center()
                home(-target)
                #depth_ctrl_from_cam(-target) #xx
        
        #### LOG STATUS ####
        print(status)
        depth_sensor.update()
        depth_mm = max(0, (depth_sensor.pressure_mbar - surface_pressure) * 10.197162129779)
        log_status(time.time()-t_start, status, depth_mm, max_colors, no_flashes, found_source, flash)


max_centroids = 0
no_images = 30
thresh_distance = 4
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
main(40) # run time, [s]
leds.off()
terminate()
