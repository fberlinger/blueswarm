import threading
from time import sleep
from Fin import Fin
import RPi.GPIO as GPIO

caudal = Fin(20, 21, 1)
dorsal = Fin(19, 26, 1)
pectol = Fin(18, 23, 1)
pector = Fin(4, 22, 1)

threading.Thread(target=caudal.run).start()
threading.Thread(target=dorsal.run).start()
threading.Thread(target=pectol.run).start()
threading.Thread(target=pector.run).start()

pectol.start()
sleep(5)
pector.start()
sleep(5)
pector.set_frequency(5)
sleep(5)
pectol.stop()
pector.stop()

caudal.terminate()
dorsal.terminate()
pectol.terminate()
pector.terminate()

GPIO.cleanup()