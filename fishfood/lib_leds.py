"""LED library, provides functions to switch BlueBot's LEDs on and off.
"""
import threading
import time
import RPi.GPIO as GPIO

class LEDS():

    """Functions to switch LEDs on and off.
    """
    
    def __init__(self):
        """All 3 LEDs are on pin 13 and can be operated together only
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(13, GPIO.OUT)

        self._is_terminated = False

        self._flash = False
        self.frequency = 0

    def on(self):
        """Switch on
        """
        GPIO.output(13, GPIO.HIGH)

    def off(self):
        """Switch off
        """
        GPIO.output(13, GPIO.LOW)

    def flash_on(self, frequency=15):
        self.frequency = frequency
        self._flash = True

    def flash_off(self):
        self._flash = False

    def flash(self):
        """Flash leds
        """
        t_off = time.time()
        while not self._is_terminated:
            if self._flash:
                half_period = 1/(self.frequency*2)
                t_elapsed = time.time() - t_off
                time.sleep(max(0, half_period - t_elapsed))
                #self.on()
                GPIO.output(13, GPIO.HIGH) # might be faster than fct call
                time.sleep(half_period)
                #self.off()
                GPIO.output(13, GPIO.LOW) # might be faster than fct call
                t_off = time.time()
            else:
                time.sleep(0.1)
                t_off = time.time()

    def terminate(self):
        """Terminates flash thread
        """
        self._is_terminated = True
