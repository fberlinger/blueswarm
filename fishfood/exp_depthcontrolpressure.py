from lib_depthsensor import DepthSensor
from lib_fin import Fin
from lib_leds import LEDS

import RPi.GPIO as GPIO
import threading
import time

caudal = Fin(20, 21, 1)
dorsal = Fin(19, 26, 4)
pectol = Fin(18, 23, 1)
pector = Fin(4, 22, 1)

threading.Thread(target=caudal.run).start()
threading.Thread(target=dorsal.run).start()
threading.Thread(target=pectol.run).start()
threading.Thread(target=pector.run).start()

depth_sensor = DepthSensor()
leds = LEDS()

run_time = 30
thresh_lo = 150
thresh_hi = 400

start_time = time.time()

state = 0

leds.on()

while time.time() - start_time < run_time:
    depth_sensor.update()
    depth_mm = depth_sensor.depth_mm

    if state == 1 and depth_mm > thresh_hi:
        dorsal.off()
        state = 0

    if state == 0 and depth_mm < thresh_lo:
        dorsal.on()
        state = 1

dorsal.off()
leds.off()

caudal.terminate()
dorsal.terminate()
pectol.terminate()
pector.terminate()

GPIO.cleanup()
