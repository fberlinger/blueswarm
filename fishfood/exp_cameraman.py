
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import os
import time
import threading
import numpy as np
from picamera import PiCamera

from lib_utils import *
from lib_depthsensor import DepthSensor
from lib_fin import Fin

os.makedirs('./data/{}/'.format(U_FILENAME))



class Video():
    picam = PiCamera()
    picam.resolution = (1296, 972) # max values, have to be 4:3
    picam.rotation = 180
    CAMLED = 40
    GPIO.setup(CAMLED, GPIO.OUT)
    GPIO.output(CAMLED, U_CAM_RIGHT) # set to right cam

    def start(self):
        self.picam.start_recording('./data/{}/video.h264'.format(U_FILENAME))

    def stop(self):
        self.picam.stop_recording()


def initialize():
    """Initializes all threads which are running fins and a logger instance for the overall status
    """
    threading.Thread(target=caudal.run).start()
    threading.Thread(target=dorsal.run).start()
    threading.Thread(target=pecto_l.run).start()
    threading.Thread(target=pecto_r.run).start()

    threading.Thread(target=video.start).start()

def terminate():
    """Terminates all threads which are running fins
    """
    caudal.terminate()
    dorsal.terminate()
    pecto_l.terminate()
    pecto_r.terminate()

    video.stop()

    GPIO.cleanup()

def depth_ctrl_from_depthsensor(target_depth):
    """Controls the diving depth to a preset level
    
    Args:
        thresh (int, optional): Threshold below which dorsal fin is not controlled, [mm]
    """
    thresh = 2

    depth_sensor.update()
    depth_mm = max(0, (depth_sensor.pressure_mbar - surface_pressure) * 10.197162129779)

    if depth_mm > (target_depth + thresh):
        dorsal.off()
    elif depth_mm < (target_depth - thresh):
        dorsal.on()

def main(run_time=60):
    while (time.time() - t_start) < run_time:
        depth_ctrl_from_depthsensor(250)

        # move forward
        if 0 < (time.time() - t_start) < 10:
            caudal.on()

        # turn right
        elif 10 < (time.time() - t_start) < 20:
            caudal.off()
            pecto_l.on()

        # move forward
        elif 20 < (time.time() - t_start) < 30:
            pecto_l.off()
            caudal.on()

        # move backward
        elif 30 < (time.time() - t_start) < 40:
            caudal.off()
            pecto_r.on()
            pecto_l.on()

        # turn left
        elif 40 < (time.time() - t_start) < 50:
            pecto_l.off()

        # move forward
        elif 50 < (time.time() - t_start) < 60:
            pecto_r.off()
            caudal.on()
        

caudal = Fin(U_FIN_C1, U_FIN_C2, 2) # freq, [Hz]
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 3) # freq, [Hz]
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 3) # freq, [Hz]

video = Video()

depth_sensor = DepthSensor()
depth_sensor.update()
surface_pressure = depth_sensor.pressure_mbar


initialize()
t_start = time.time()
main(60)
terminate()
