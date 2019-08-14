
from lib_utils import *
import threading
from time import sleep
import RPi.GPIO as GPIO

from lib_fin import Fin

dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq
threading.Thread(target=dorsal.run).start()

dorsal.on()
sleep(8)
dorsal.off()

dorsal.terminate()
GPIO.cleanup()
