"""Synchronization library. Flashes LEDs at given time intervals, uses cameras to observe other robots, and adjusts flashing to synchronize with them.
"""
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import numpy as np
import time
import math
import threading

from lib_utils import *
from lib_leds import LEDS
from lib_camera import Camera

class Sync():

    """Summary
    
    Attributes:
        clockspeed (float): Update period for clock that checks whether it's time to flash again.
        flash_dur (int): Duration of LED flashing
        flash_led (bool): Status for flash_LEDs function that runs and checks continuously on a thread
        flash_period (int): Time between LED flashings
        is_started (bool): Status for synchronization
        last_flash (int): Time of last flash
        obs_dur (float): Duration of a camera observation
        observed (int): Counter that starts at refractory whenever another flashing was observed and goes down to zero. No observations are made during while counter is active.
        refractory (TYPE): refractory period. After an observed LED flashing, no observations are made for the duration of the refractory period.
        timestamp (int): Time in flashing cycle.
    """
    
    def __init__(self):
        """Uses LED and camera classes.
        """
        self.is_started = True
        self.flash_led = False
        self.observed = 0

        self.timestamp = 0
        self.last_flash = 0
        self.clockspeed = 0.05

        self.flash_period = 10
        self.flash_dur = 1
        self.obs_dur = 0.35
        self.refractory = math.ceil(self.flash_dur / self.obs_dur)

        self._leds = LEDS()
        self._cam_r = Camera('right')
        self._cam_l = Camera('left')


    def update(self, run_time):
        """Runs observation if not in refractory period and updates timestamp if flashing is observed.
        
        Args:
            run_time (TYPE): Description
        """
        t_start = time.time()
        t_start_loop = time.time()

        while time.time() - t_start < run_time:
            if self.observed > 0:
                self.observed -= 1
            elif self._observe():
                self.observed = self.refractory
                dt = time.time() - self.last_flash
                self.timestamp += math.sqrt(dt)
            
            t_elapsed = time.time() - t_start_loop
            time.sleep(self.obs_dur - t_elapsed)
            t_start_loop = time.time()

    def _observe(self):
        """Uses cameras to check for LED flashing, which is assumed if the mean image brightness is above thresh_flash.
        
        Returns:
            TYPE: Description
        """
        flash = False
        thresh_flash = 0.01

        self._cam_r.capture()
        avg_bright_r = np.mean(self._cam_r.img)
        #print('avg_bright_r = {}'.format(avg_bright_r))

        self._cam_l.capture()
        avg_bright_l = np.mean(self._cam_l.img)

        if avg_bright_r > thresh_flash or avg_bright_l > thresh_flash:
            flash = True
        
        return flash

    def clock(self):
        """Proceeds timestamp within a flashing period. Resets timestamp and causes LEDs to flash (fires!) at the end of each flashing period. Runs on a separate thread.
        """
        self.last_flash = time.time()
        t_start_loop = time.time()
        last_update = time.time()
        
        while self.is_started:
            if self.timestamp > self.flash_period:
                self.observed = self.refractory
                self.flash_led = True
                self.timestamp = 0
                self.last_flash = time.time()
            else:
                now = time.time()
                dt = now - last_update
                last_update = now
                self.timestamp += dt

            t_elapsed = time.time() - t_start_loop
            time.sleep(self.clockspeed - t_elapsed)
            t_start_loop = time.time()

    def flash_LEDs(self):
        """Flashes LEDs for flash_dur whenever flash_led is set True by the clock. Runs on a separate thread.
        """
        while self.is_started:
            if self.flash_led:
                self._leds.on()
                time.sleep(self.flash_dur)
                self._leds.off()
                self.flash_led = False
