
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

def log_status(t_passed, status, found_source, max_colors, found_flash, no_flashes, depth_mm):
    """Logs the overall status of BlueBot
    
    Args:
        t_passed (float): Time since the beginning of the experiment, [s]
        status (string): Status in the finite state machine
    """
    with open('./data/{}/{}_status.log'.format(U_FILENAME, U_UUID), 'a') as f:
        f.write('{:.2f},{},{:.2f},{:.2f},{},{},{}\n'.format(t_passed, status, found_source, max_colors, found_flash, no_flashes, depth_mm))

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

def home(target, signal_freq=1.5):
    """Controls the pectoral fins to follow an object using both cameras

    The "heading" angle towards an object is calculated based on (pqr) coordinates as follows: atan2(q, p). A positive angle switches the pectoral left fin on turn clockwise. A negative angles switches the pectoral right fin on to turn counterclockwise.
    
    Returns:
        (): Floats to the surface and turns on the spot if no object observed
    """
    caudal_range = 35 # abs(heading) below which caudal fin is switched on
    freq_c = min(signal_freq, 1.5)
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
        pecto_r.set_frequency(2.5)
        pecto_r.on()
        pecto_l.set_frequency(2.5)
        pecto_l.on()

    # target in front
    elif heading < 10 and heading > -10:
        pecto_r.off()
        pecto_l.off()
        caudal.on()

    # target to the right
    elif heading > 10:
        freq_l = 1 + 1.5 * abs(heading) / 155
        pecto_l.set_frequency(freq_l)

        pecto_l.on()
        pecto_r.off()

        if heading < caudal_range:
            caudal.on()
        else:
            caudal.off()

    # target to the left
    elif heading < -10:
        freq_r = 1 + 1.5 * abs(heading) / 155
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
    pitch_range = 2 # abs(pitch) below which dorsal fin is not controlled 

    pitch = np.arctan2(target[2], sqrt(target[0]**2 + target[1]**2)) * 180 / pi

    if pitch > pitch_range:
        dorsal.on()
    elif pitch < -pitch_range:
        dorsal.off()

def check_flash():
    found_flash = False
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
        found_flash = True
        uvw_r = vision._mn_to_uvw(location)
        pqr_r = vision._uvw_to_pqr_r(uvw_r)
        flash_location = np.zeros((3,))
        flash_location[0] = pqr_r[0,0]
        flash_location[1] = pqr_r[1,0]
        flash_location[2] = pqr_r[2,0]
        return (found_flash, flash_location, no_flashes)

    # check left side in case right side didn't detect flash
    outliers = greedymatch.find_outliers(imgs_l, 'left')
    no_flashes, location = flashdetector.find_max_flashes(outliers)
    if no_flashes >= thresh_flash:
        found_flash = True
        uvw_l = vision._mn_to_uvw(location)
        pqr_l = vision._uvw_to_pqr_l(uvw_l)
        flash_location = np.zeros((3,))
        flash_location[0] = pqr_l[0,0]
        flash_location[1] = pqr_l[1,0]
        flash_location[2] = pqr_l[2,0]
        return (found_flash, flash_location, no_flashes)

    return (found_flash, flash_location, no_flashes)

def check_signal():
    found_source = False
    source_location = np.zeros(0)
    no_pix = 3 # was 3 for first successful experiment
    neighborhood = 2 # no_pix = (2*neighborhood+1)**2

    vision._cam_r.capture()
    vision._blob_r.detect(vision._cam_r.img)
    colors, blob_ind = vision._blob_r.color_intensities(vision._cam_r.img, no_pix, neighborhood)

    if colors:
        if max(colors) > 1.2:
            found_source = True
            ind = blob_ind[colors.index(max(colors))]
            mn_r = np.zeros((2,1))
            mn_r[0] = vision._blob_r.blobs[0,ind]
            mn_r[1] = vision._blob_r.blobs[1,ind]
            uvw_r = vision._mn_to_uvw(mn_r)
            pqr_r = vision._uvw_to_pqr_r(uvw_r)
            source_location = np.zeros((3,))
            source_location[0] = pqr_r[0,0]
            source_location[1] = pqr_r[1,0]
            source_location[2] = pqr_r[2,0]
            return (found_source, source_location, max(colors)) # bool, np.zeros((3,)) or any list such that source_location[0] is p, [1] is q, [2] is r

    vision._cam_l.capture()
    vision._blob_l.detect(vision._cam_l.img)
    colors, blob_ind = vision._blob_l.color_intensities(vision._cam_l.img, no_pix, neighborhood)

    if colors:
        if max(colors) > 1.2:
            found_source = True
            ind = blob_ind[colors.index(max(colors))]
            mn_l = np.zeros((2,1))
            mn_l[0] = vision._blob_l.blobs[0,ind]
            mn_l[1] = vision._blob_l.blobs[1,ind]
            uvw_l = vision._mn_to_uvw(mn_l)
            pqr_l = vision._uvw_to_pqr_l(uvw_l)
            source_location = np.zeros((3,))
            source_location[0] = pqr_l[0,0]
            source_location[1] = pqr_l[1,0]
            source_location[2] = pqr_l[2,0]
            return (found_source, source_location, max(colors))

    return (found_source, source_location, 9999) # bool, np.zeros((3,)) or any list such that source_location[0] is p, [1] is q, [2] is r



def main(run_time):
    vision._cam_r.redblue_settings()
    vision._cam_l.redblue_settings()
    time.sleep(0.25)

    d_status = {'search': 0, 'home': 1, 'signal': 2} # status
    signal_counter = 0
    flash_counter = 0
    status = 'search'

    while (time.time() - t_start) < run_time:
        found_flash = False
        no_flashes = 9999
        # FOUND SOURCE?
        try:
            found_source, source_location, max_colors = check_signal()
        except:
            continue
        if found_source:
            status = 'signal'
            signal_counter = 0
            # flash leds
            leds.off()
            leds.flash_on()
            # stay at source
            home(source_location, 1.5)
            depth_ctrl_from_cam(source_location)
        elif status == 'signal' and signal_counter < 6:
            signal_counter += 1
        
        # OBSERVED FLASH?
        else:
            vision._cam_r.std_settings()
            vision._cam_l.std_settings()
            time.sleep(0.25)

            leds.flash_off()
            try:
                found_flash, flash_location, no_flashes = check_flash()
            except:
                vision._cam_r.redblue_settings()
                vision._cam_l.redblue_settings()
                time.sleep(0.25)
                continue
            if found_flash:
                vision._cam_r.redblue_settings()
                vision._cam_l.redblue_settings()
                status = 'home'
                flash_counter = 0
                leds.off()
                home(flash_location)
                depth_ctrl_from_cam(flash_location)
            elif status == 'home' and flash_counter < 6:
                vision._cam_r.redblue_settings()
                vision._cam_l.redblue_settings()
                flash_counter += 1
            else:
                status = 'search'
                leds.on()
                try:
                    vision.update()
                except:
                    vision._cam_r.redblue_settings()
                    vision._cam_l.redblue_settings()
                    time.sleep(0.25)
                    continue
                vision._cam_r.redblue_settings()
                vision._cam_l.redblue_settings()
                target = center()
                home(-target)
                depth_ctrl_from_cam(-target)
        
        #### LOG STATUS ####
        #print(status, time.time()-t_start)
        depth_sensor.update()
        depth_mm = max(0, (depth_sensor.pressure_mbar - surface_pressure) * 10.197162129779)
        log_status(time.time()-t_start, d_status[status], found_source, max_colors, found_flash, no_flashes, depth_mm)


max_centroids = 0 # how many blobs you expect to see in one image, surplus will be remove by blob.reflections(), 0 disables blob.reflections()
no_images = 30 # 30 no of images in sequence for flash detection
thresh_flash = 10 # 10 no of observed flashes to trigger flash detection (flashing at 15Hz, so nominally 15 flashes)
thresh_distance = 6 # distance a blob may move between successive images
ligth_sens = 32 # thresholding for flash pixels
cont_pix = 3 # gap size for continuity of flash pixels
rec_depth = 0 # recursion depth in blob detection, 0 disables recursion
nhood_size = 5 # size of neighborhood that gets probed for disappeared blob in flash detection
soft_thresh = 6 # light sensitivity threshold for a missing pixel

greedymatch = ImgMatch(no_images, max_centroids, thresh_distance, ligth_sens, cont_pix, rec_depth, nhood_size, soft_thresh)
flashdetector = FlashDetector(thresh_distance)

caudal = Fin(U_FIN_C1, U_FIN_C2, 1.5) # freq, [Hz]
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
main(240) # run time, [s]
leds.off()
terminate()
