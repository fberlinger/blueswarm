import threading
from time import sleep
import RPi.GPIO as GPIO

from fin import Fin

caudal = Fin(20, 21, 4)
dorsal = Fin(19, 26, 4)
pectol = Fin(18, 23, 6)
pector = Fin(4, 22, 6)

threading.Thread(target=caudal.run).start()
threading.Thread(target=dorsal.run).start()
threading.Thread(target=pectol.run).start()
threading.Thread(target=pector.run).start()

pectol.start()
sleep(5)
pectol.stop()
pector.start()
sleep(5)
pector.stop()
dorsal.start()
sleep(5)
dorsal.stop()
caudal.start()
sleep(5)
caudal.stop()

caudal.terminate()
dorsal.terminate()
pectol.terminate()
pector.terminate()

GPIO.cleanup()
