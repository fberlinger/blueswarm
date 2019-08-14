import math
import threading
import time
import RPi.GPIO as GPIO
import numpy as np

class Home():
    def __init__(self, pecto_r, pecto_l, caudal):
        self._is_started = False
        self._is_terminated = False

        self.freq_right = 0 # freq pectoral right
        self.freq_left = 0 # freq pectoral left

        self.error = 0
        self.kp = 6/180 # 6Hz (max freq) / 180deg (max error)
        self.omega = 0
        self.alpha = 6 # turning rate is 36deg/s at 6Hz

        self.pecto_r = pecto_r
        self.pecto_l = pecto_l
        self.caudal = caudal

        self.dt = 0.25

    def terminate(self):
        self._is_terminated = True

    def on(self):
        self._is_started = True

    def off(self):
        self._is_started = False

    def set_target(self, target):
        if not target.size:
            return
        # calculate heading
        self.error = np.arctan2(target[1], target[0]) * 180 / math.pi

    def run(self):
        """Times the switching of directions. Runs in a loop on a thread.
        """
        while not self._is_terminated:
            if self._is_started:
                start_time = time.time()

                self._eval()
            
                elapsed_time = time.time() - start_time
                sleep_time = max(0, self.dt - elapsed_time)
                time.sleep(sleep_time)
            else:
                time.sleep(0.1)

    def _eval(self):
        if abs(self.error) < 35:
            self.caudal.on()
        else:
            self.caudal.off()

        ctrl_output = self.kp * self.error
        if ctrl_output < 0:
            self.freq_left = 0
            self.pecto_l.off()
            self.freq_right = abs(ctrl_output)
            if self.freq_right < 0.1:
				self.freq_right = 0
                self.pecto_r.off()
            else:
                self.pecto_r.set_frequency(self.freq_right)
                self.pecto_r.on()
        else:
            self.freq_right = 0
            self.pecto_r.off()
            self.freq_left = ctrl_output
            if self.freq_left < 0.1:
                self.freq_left = 0
                self.pecto_l.off()
            else:
                self.pecto_l.set_frequency(self.freq_left)
                self.pecto_l.on()

        self.omega = self.alpha * (self.freq_left - self.freq_right)
        self.error = self.error - self.omega * self.dt
        