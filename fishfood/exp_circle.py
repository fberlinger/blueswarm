import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

from lib_utils import *
from lib_camera import Camera

from lib_leds import LEDS

import numpy as np

import threading
from lib_fin import Fin

from timeit import default_timer as timer

caudal = Fin(U_FIN_C1, U_FIN_C2, 3) # freq, [Hz]
dorsal = Fin(U_FIN_D1, U_FIN_D2, 6) # freq, [Hz]
pecto_r = Fin(U_FIN_PR1, U_FIN_PR2, 8) # freq, [Hz]
pecto_l = Fin(U_FIN_PL1, U_FIN_PL2, 8) # freq, [Hz]

threading.Thread(target=caudal.run).start()
threading.Thread(target=dorsal.run).start()
threading.Thread(target=pecto_l.run).start()
threading.Thread(target=pecto_r.run).start()



leds = LEDS()

cam_l = Camera('left')
cam_r = Camera('right')

mask_l = np.loadtxt("mask_l.txt", dtype=np.int32)
mask_r = np.loadtxt("mask_r.txt", dtype=np.int32)

print("Starting")

caudal.on()

start = timer()
for i in range(50):
	print(i)
	cam_l.capture()
	cam_r.capture()

	L = cam_l.img	
	R = cam_r.img

	LM = np.multiply(L[:, :, 2], mask_l)
	RM = np.multiply(R[:, :, 2], mask_r)

	if ((np.sum(LM) > 100) or (np.sum(RM) > 100)):
		leds.on()
		pecto_r.off()
		pecto_l.on()
	else:
		leds.off()
		pecto_r.on()
		pecto_l.off()

#	print(np.sum(AA))

caudal.off()

end = timer()

leds.off()

print(end - start)

caudal.terminate()
dorsal.terminate()
pecto_l.terminate()
pecto_r.terminate()