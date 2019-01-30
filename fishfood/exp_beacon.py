import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time

from lib_utils import *
from lib_photodiode import Photodiode
from lib_leds import LEDS

def idle():
    """Waiting for starting signal
    """
    thresh_photodiode = 50 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        photodiode.update()

photodiode = Photodiode()
leds = LEDS()

idle()
leds.on()
time.sleep(30)
leds.off()
