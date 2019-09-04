
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time
import random
import threading

from lib_utils import *
from lib_fin import Fin
from lib_leds import LEDS
from lib_depthsensor import DepthSensor

def initialize():
    """Initializes all threads which are running fins and a logger instance for the overall status
    """
    threading.Thread(target=caudal.run).start()
    threading.Thread(target=dorsal.run).start()
    threading.Thread(target=pecto_l.run).start()
    threading.Thread(target=pecto_r.run).start()

def terminate():
    """Terminates all threads which are running fins
    """
    caudal.terminate()
    dorsal.terminate()
    pecto_l.terminate()
    pecto_r.terminate()
    GPIO.cleanup()

def main(run_time=60):
    while (time.time() - t_start) < run_time:
        rand = random.random()
        
        # depth clipping
        depth_sensor.update()
        depth_mm = max(0, (depth_sensor.pressure_mbar - surface_pressure) * 10.197162129779)
        if depth_mm > 550:
            dorsal.off()
        elif depth_mm < 150:
            dorsal.on()
        # down
        elif rand > 0.5:
            dorsal.on()
        # up
        else:
            dorsal.off()

        # right
        if rand < 0.33:
            pecto_r.off()
            pecto_l.on()
        # left
        elif rand > 0.66:
            pecto_l.off()
            pecto_r.on()
        # straight
        else:
            pecto_r.off()
            pecto_l.off()
        
        # forward
        if rand < 0.5:
            caudal.on()
        # caudal off
        elif rand > 0.75:
            caudal.off()
        # backward
        else:
            caudal.off()
            pecto_r.on()
            pecto_l.on()

        time.sleep(10)


caudal = Fin(U_FIN_C1, U_FIN_C2, 2.5) # freq, [Hz]
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 3) # freq, [Hz]
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 3) # freq, [Hz]
leds = LEDS()

depth_sensor = DepthSensor()
depth_sensor.update()
surface_pressure = depth_sensor.pressure_mbar

initialize()
t_start = time.time()
leds.on()
main(70)
leds.off()
terminate()
