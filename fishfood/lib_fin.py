import threading
import time
import RPi.GPIO as GPIO

class Fin():
    def __init__(self, pin_1, pin_2, frequency):
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
        self._is_started = True

    def off(self):
        GPIO.output(self._pin_1, GPIO.LOW)
        GPIO.output(self._pin_2, GPIO.LOW)
        self._is_started = False

    def terminate(self):
        self._is_terminated = True

    def set_frequency(self, frequency):
        self._period = 1 / (2 * frequency)

    def run(self):
        while not self._is_terminated:
            if self._is_started:
                start_time = time.time()

                self._eval()
            
                elapsed_time = time.time() - start_time
                sleep_time = self._period - elapsed_time
                time.sleep(sleep_time)

    def _eval(self):
        if self._state == 0:
            GPIO.output(self._pin_1, GPIO.LOW)
            GPIO.output(self._pin_2, GPIO.HIGH)
        else:
            GPIO.output(self._pin_2, GPIO.LOW)
            GPIO.output(self._pin_1, GPIO.HIGH)

        self._state = 1 - self._state
