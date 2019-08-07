import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

from lib_utils import *
from lib_leds import LEDS
import time
import threading

leds = LEDS()
threading.Thread(target=leds.flash).start()
leds.on()

t_start = time.time()
while time.time() - t_start < 30:
    time.sleep(0.5)
leds.off()

leds.terminate()
GPIO.cleanup()
