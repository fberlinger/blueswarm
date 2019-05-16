import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time
from lib_utils import *
from lib_leds import LEDS

leds = LEDS()

leds.on()
time.sleep(40)
leds.off()