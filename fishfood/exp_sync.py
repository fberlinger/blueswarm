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
import numpy as np
from math import *

from lib_utils import *
from lib_sync import Sync
from lib_fin import Fin
from lib_depthsensor import DepthSensor


def initialize():
    """Initializes all threads which are running the clock and flash_LED functions for synchronization, and the dorsal fin.
    """
    threading.Thread(target=sync.clock).start()
    threading.Thread(target=sync.flash_LEDs).start()
    threading.Thread(target=dorsal.run).start()

def terminate():
    """Terminates synchronization and dorsal fin thread.
    """
    sync.is_started = False
    dorsal.terminate()

    GPIO.cleanup()

def depth_ctrl_from_depthsensor(target_depth=400, thresh=2):
    """Controls the diving depth to a preset level
    
    Args:
        target_depth (int, optional): Nominal diving depth
        thresh (int, optional): Threshold below which dorsal fin is not controlled, [mm]
    """
    depth_sensor.update()

    if depth_sensor.depth_mm > (target_depth + thresh):
        dorsal.off()
    elif depth_sensor.depth_mm < (target_depth - thresh):
        dorsal.on()

def main(run_time):
    """Runs experiment
    
    Args:
        run_time (int): Duration of synchronization
    """
    time.sleep(22) # watch desync flashing
    sync.update(run_time) # sync flashing
    sync.is_started = False # terminate

dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]

depth_sensor = DepthSensor()
sync = Sync()

time.sleep(2)
initialize()
main(92) # run time
terminate()
