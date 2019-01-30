"""Makes BlueBots sync their LED flashing.

Attributes:
    depth_sensor (Class instance): Instance of depth sensor for depth control.
    dorsal (Class instance): Instance of dorsal fin for vertical movement.
    sync (Class instance): Instance of synchronization for LED sync.
"""
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time
import threading
import random
import numpy as np
from math import *

from lib_utils import *
from lib_sync import Sync
from lib_fin import Fin
from lib_leds import LEDS
from lib_depthsensor import DepthSensor
from lib_photodiode import Photodiode


def initialize():
    """Initializes all threads which are running the clock and flash_LED functions for synchronization, and the dorsal fin.
    """
    threading.Thread(target=dorsal.run).start()

    leds.on()
    time.sleep(1)
    leds.off()
    time.sleep(1)

def terminate():
    """Terminates synchronization and dorsal fin thread.
    """
    sync.is_started = False
    dorsal.terminate()

    GPIO.cleanup()

def depth_ctrl_from_depthsensor(target_depth=400, thresh=2): # change depth
    """Controls the diving depth to a preset level
    
    Args:
        target_depth (int, optional): Nominal diving depth
        thresh (int, optional): Threshold below which dorsal fin is not controlled, [mm]
    """
    t_start = time.time()

    while time.time() - t_start < 148:
        depth_sensor.update()

        if depth_sensor.depth_mm > (target_depth + thresh):
            dorsal.off()
        elif depth_sensor.depth_mm < (target_depth - thresh):
            dorsal.on()
        
        # radom diving for broken psensor
        #dorsal.on()
        #time.sleep(10)
        #dorsal.off()
        #time.sleep(15)
        
    dorsal.off()


def idle():
    """Waiting for starting signal
    """
    thresh_photodiode = 50 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        photodiode.update()

    leds.on()
    time.sleep(1)
    leds.off()
    time.sleep(4) # change to 1 or 7 for other fishes

    threading.Thread(target=sync.clock).start()
    threading.Thread(target=sync.flash_LEDs).start()
    threading.Thread(target=depth_ctrl_from_depthsensor).start()

def main(run_time):
    """Runs experiment
    
    Args:
        run_time (int): Duration of synchronization
    """
    time.sleep(38) # watch desync flashing
    sync.update(run_time) # sync flashing
    sync.is_started = False # terminate

dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]

depth_sensor = DepthSensor()
photodiode = Photodiode()
sync = Sync()
leds = LEDS()

initialize()
idle()
main(110) # run time
terminate()
