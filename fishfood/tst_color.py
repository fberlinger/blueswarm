import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import time
import os
import numpy as np
#import matplotlib.pyplot as plt

from lib_utils import *
from lib_camera import Camera


cam_l = Camera('left')
#cam_r = Camera('right')
cam_l.colorbot_settings()
#cam_r.colorbot_settings()

mask_l = np.loadtxt("/home/pi/fishfood/mask_l.txt", dtype=np.int32)
#mask_r = np.loadtxt("/home/pi/fishfood/mask_r.txt", dtype=np.int32)
mask_cb = np.loadtxt("/home/pi/fishfood/mask_cb.txt", dtype=np.int32)

cam_l.capture()
#cam_r.capture()

L = cam_l.img
#R = cam_r.img
np.savetxt('L.txt', L[:, :, 2])
#np.savetxt('R.txt', R[:, :, 2])

img_dark = 255*np.ones((192, 256))

LM = np.multiply(img_dark-L[:, :, 2], mask_cb)
#RM = np.multiply(img_dark, mask_cb) - np.multiply(R[:, :, 2], mask_cb)

LS = np.multiply(LM, mask_l)

np.savetxt('LM.txt', LM)
#np.savetxt('RM.txt', RM)

#plt.imshow(LM, cmap="gray")
#plt.imshow(RM, cmap="gray")
#plt.show()