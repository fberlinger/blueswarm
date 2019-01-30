"""Test script to calibrate a BlueBot for straight swimming

Attributes:
    caudal (): Fin object for caudal fin
"""
from lib_utils import *
import threading
from time import sleep
import RPi.GPIO as GPIO

from lib_fin import Fin

caudal = Fin(U_FIN_C1, U_FIN_C2, 4) # freq

threading.Thread(target=caudal.run).start()

sleep(4)
caudal.on()
sleep(30)
caudal.off()

caudal.terminate()
GPIO.cleanup()
