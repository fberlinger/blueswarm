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

from lib_utils import *
from lib_fin import Fin
from lib_depthsensor import DepthSensor
from lib_leds import LEDS
from lib_photodiode import Photodiode
from lib_sync import Sync


def initialize():
    """Initializes all threads which are running the clock and flash_LED functions for synchronization, and the dorsal fin.
    """
    threading.Thread(target=dorsal.run).start()

    leds.on()
    time.sleep(1)
    leds.off()

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

    while True:
        depth_sensor.update()
        depth_mm = max(0, (depth_sensor.pressure_mbar - surface_pressure) * 10.197162129779)

        if depth_mm > (target_depth + thresh):
            dorsal.off()
        elif depth_mm < (target_depth - thresh):
            dorsal.on()

    dorsal.off()

def idle():
    """Waiting for starting signal
    """
    thresh_photodiode = 50 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        photodiode.update()
    time.sleep(2)

    leds.on()
    time.sleep(1)
    leds.off()

    t_start = time.time()

    sleep_time = 12*random.random()
    time.sleep() # sleep different times for initial desynced flashings

    sync.t_start = t_start
    threading.Thread(target=sync.clock).start()
    threading.Thread(target=sync.flash_LEDs).start()
    threading.Thread(target=depth_ctrl_from_depthsensor).start()

    return t_start

def main(run_time):
    """Runs experiment
    
    Args:
        run_time (int): Duration of synchronization
    """
    t_main = time.time()
    t_desync = 50 - (t_main-t_start)
    time.sleep(t_desync) # watch desync flashing, 3 rounds of 15s plus buffer
    sync.update(run_time-t_desync) # sync flashing
    sync.is_started = False # terminate


target_depth = -149 + U_UUID*83 # 100-600mm if using robots 3-9
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]
depth_sensor = DepthSensor()
depth_sensor.update()
surface_pressure = depth_sensor.pressure_mbar

leds = LEDS()
photodiode = Photodiode()
sync = Sync()

initialize()
t_start = idle()
main(190) # run time, allows for 12 rounds of 15s each plus buffer
terminate()
