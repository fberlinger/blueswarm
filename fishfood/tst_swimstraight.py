import threading
from time import sleep
import RPi.GPIO as GPIO

from lib_fin import Fin

caudal = Fin(20, 21, 6)

threading.Thread(target=caudal.run).start()

sleep(4)
caudal.on()
sleep(30)
caudal.off()

caudal.terminate()

GPIO.cleanup()
