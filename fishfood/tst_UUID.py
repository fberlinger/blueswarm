import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time
from lib_utils import *
from lib_leds import LEDS

leds = LEDS()

start_time = time.time()
for blink in range(U_UUID):
    leds.on()
    time.sleep(0.2)
    leds.off()
    time.sleep(0.2)
elapsed_time = time.time() - start_time
sleep_time = 8 - elapsed_time
time.sleep(sleep_time) # wait such that all robots leave idle before LEDs are on

leds.on()
time.sleep(1)
leds.off()