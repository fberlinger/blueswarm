
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time
from picamera import PiCamera

from lib_utils import *
from lib_photodiode import Photodiode

photodiode = Photodiode()
t_start = time.time()

while time.time() - t_start < 30:
    photodiode.update()
    print(photodiode.brightness)
    time.sleep(1)
