from time import sleep
import threading
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

from lib_leds import LEDS
from lib_depthsensor import DepthSensor
from lib_fin import Fin

leds = LEDS()
depth_sensor = DepthSensor()

caudal = Fin(20, 21, 4)
dorsal = Fin(19, 26, 4)
pectol = Fin(18, 23, 6)
pector = Fin(4, 22, 6)

threading.Thread(target=caudal.run).start()
threading.Thread(target=dorsal.run).start()
threading.Thread(target=pectol.run).start()
threading.Thread(target=pector.run).start()

# WRITE YOUR CODE HERE

caudal.terminate()
dorsal.terminate()
pectol.terminate()
pector.terminate()
GPIO.setwarnings(False)
GPIO.cleanup()
