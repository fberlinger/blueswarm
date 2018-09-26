from depth_sensor import DepthSensor
from fin import Fin

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

target_depth = 200
thresh_lo = 198
thresh_hi = 202

depth_sensor = DepthSensor()

start_time = time.time()

state = 0

while time.time() - start_time < 60:
    depth_sensor.update()
    depth_mm = depth_sensor.depth_mm

    if state == 1 and depth_mm > thresh_hi:
        dorsal.stop()
        state = 0

    if state == 0 and depth_mm < thresh_lo:
        dorsal.start()
        state = 1

dorsal.stop()

caudal.terminate()
dorsal.terminate()
pectol.terminate()
pector.terminate()

GPIO.cleanup()
