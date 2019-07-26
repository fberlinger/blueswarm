
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

#os.makedirs('./{}/'.format(U_FILENAME))

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

    time.sleep(1)
    leds.on()
    time.sleep(1)
    leds.off()

    GPIO.cleanup()

def waste_time():
    time.sleep(1)
    print('waste time')

def check_flash():
    flash = False
    imgs = [np.empty((U_CAM_MRES, U_CAM_NRES, 3), dtype=np.uint8) for _ in range(no_images)]

    # check right side and return if flash is detected
    vision._cam_r.capture_sequence(imgs)
    outliers = greedymatch.find_outliers(imgs, 'right')
    no_flashes = flashdetector.find_max_flashes(outliers)
    print('no flashes right {}'.format(no_flashes))

    if no_flashes >= thresh_flash:
        flash = True
        return flash

    # check left side in case right side didn't detect flash
    vision._cam_l.capture_sequence(imgs)
    outliers = greedymatch.find_outliers(imgs, 'left')
    no_flashes = flashdetector.find_max_flashes(outliers)
    print('no flashes left {}'.format(no_flashes))  
    if no_flashes >= thresh_flash:
        flash = True
    return flash

def main(run_time):
    t_passed = 0
    while t_passed < run_time:

        waste_time()
        t_flash = time.time()
        flash = check_flash()
        dur_flash = time.time() - t_flash
        print('dur check_flash {}s\n'.format(dur_flash))
        if flash:
            leds.off()
        else:
            leds.on()

        # update counters
        t_passed = time.time() - t_start


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
