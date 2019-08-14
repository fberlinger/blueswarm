"""Test script that tests all fins during assemly

Attributes:
    caudal (): Fin object for caudal fin
    dorsal (): Fin object for dorsal fin
    pectol (): Fin object for pectoral left fin
    pector (): Fin object for pectoral right fin
"""
from lib_utils import *
import threading
from time import sleep
import RPi.GPIO as GPIO

from lib_fin import Fin

caudal = Fin(U_FIN_C1, U_FIN_C2, 4) # freq
dorsal = Fin(U_FIN_D1, U_FIN_D2, 4) # freq
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 6) # freq
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 6) # freq

threading.Thread(target=caudal.run).start()
threading.Thread(target=dorsal.run).start()
threading.Thread(target=pecto_r.run).start()
threading.Thread(target=pecto_l.run).start()

pecto_l.on()
sleep(5)
pecto_l.off()
'''
pecto_l.on()
sleep(2.5)
pecto_l.off()
dorsal.on()
sleep(2.5)
dorsal.off()
caudal.on()
sleep(2.5)
caudal.off()
'''

caudal.terminate()
dorsal.terminate()
pecto_r.terminate()
pecto_l.terminate()
GPIO.cleanup()
