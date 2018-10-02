import threading
from time import sleep
import RPi.GPIO as GPIO

from lib_fin import Fin

caudal = Fin(20, 21, 4)
dorsal = Fin(19, 26, 4)
pectol = Fin(18, 23, 6)
pector = Fin(4, 17, 6)

threading.Thread(target=caudal.run).start()
threading.Thread(target=dorsal.run).start()
threading.Thread(target=pectol.run).start()
threading.Thread(target=pector.run).start()

pectol.on()
sleep(5)
pectol.off()
pector.on()
sleep(5)
pector.off()
dorsal.on()
sleep(5)
dorsal.off()
caudal.on()
sleep(5)
caudal.off()

caudal.terminate()
dorsal.terminate()
pectol.terminate()
pector.terminate()

GPIO.cleanup()
