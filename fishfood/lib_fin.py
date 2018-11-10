"""Fin library, controls fins.
"""
import threading
import time
import RPi.GPIO as GPIO

class Fin():

    """Fin contains functions to switch fins on and off, and to change their flapping frequencies.
    """
    
    def __init__(self, pin_1, pin_2, frequency):
        """A Fin object has two designated pins and a starting frequency
        
        Args:
            pin_1 (int): First GPIO pin of Fin object
            pin_2 (int): Second GPIO pin of Fin object
            frequency (float): Fin flapping frequency
        """
        self._period = 1 / (2 * frequency)
        self._pin_1 = pin_1
        self._pin_2 = pin_2
        self._is_started = False
        self._is_terminated = False
        self._state = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin_1, GPIO.OUT)
        GPIO.setup(self._pin_2, GPIO.OUT)

    def on(self):
        """Switches a fin on (temporarily)
        """
        self._is_started = True

    def off(self):
        """Switches a fin off (temporarily)
        """
        GPIO.output(self._pin_1, GPIO.LOW)
        GPIO.output(self._pin_2, GPIO.LOW)
        self._is_started = False

    def terminate(self):
        """Shuts a fin down
        """
        self._is_terminated = True

    def set_frequency(self, frequency):
        """Sets a new flapping frequency
        
        Args:
            frequency (float): Fin flapping frequency, [Hz]
        """
        self._period = 1 / (2 * frequency)

    def run(self):
        """Times the switching of directions. Runs in a loop on a thread.
        """
        while not self._is_terminated:
            if self._is_started:
                start_time = time.time()

                self._eval()
            
                elapsed_time = time.time() - start_time
                sleep_time = self._period - elapsed_time
                time.sleep(sleep_time)

    def _eval(self):
        """Switches directions of oscillation
        """
        if self._state == 0:
            GPIO.output(self._pin_1, GPIO.LOW)
            GPIO.output(self._pin_2, GPIO.HIGH)
        else:
            GPIO.output(self._pin_2, GPIO.LOW)
            GPIO.output(self._pin_1, GPIO.HIGH)

        self._state = 1 - self._state
