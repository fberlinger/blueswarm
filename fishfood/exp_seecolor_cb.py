
from time import sleep

import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import os
import numpy as np
from lib_camera import Camera
import threading
from lib_leds import LEDS
from lib_utils import *
from lib_photodiode import Photodiode

from PIL import Image
os.makedirs('./data/{}/'.format(U_FILENAME))
os.makedirs('./data/{}/imgs_r'.format(U_FILENAME))
os.makedirs('./data/{}/imgs_l'.format(U_FILENAME))


leds = LEDS()

photodiode = Photodiode()




cam_l = Camera('left')
cam_r = Camera('right')
cam_l.colorbot_settings()
cam_r.colorbot_settings()

mask_l = np.loadtxt("/home/pi/fishfood/mask_l.txt", dtype=np.int32)
mask_r = np.loadtxt("/home/pi/fishfood/mask_r.txt", dtype=np.int32)
mask_cb = np.loadtxt("/home/pi/fishfood/mask_cb.txt", dtype=np.int32)
img_dark = 255*np.ones((192, 256))

sensor_value = 0


def update_sensor_local(iteration):
    global sensor_value
    cam_l.capture()
    cam_r.capture()

    L = cam_l.img
    R = cam_r.img

    temp = Image.fromarray(L)
    temp.save('./data/{}/imgs_l/img_l_{}.png'.format(U_FILENAME,iteration))
    temp = Image.fromarray(R)
    temp.save('./data/{}/imgs_r/img_r_{}.png'.format(U_FILENAME,iteration))

    LM = np.multiply(img_dark-L[:, :, 2], mask_cb)
    RM = np.multiply(img_dark-R[:, :, 2], mask_cb)

    LA = np.multiply(LM, mask_l)
    RA = np.multiply(RM, mask_r)

    sensor_value = (np.sum(LA) > 30) or (np.sum(RA) > 30) #fb check 30 threshold
    if sensor_value:
        leds.on()
    else:
        leds.off()

def update_sensor(iteration):
    x = threading.Thread(target=update_sensor_local(iteration))
    x.start()

def idle():
    """Waiting for starting signal
    """
    thresh_photodiode = 50 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        photodiode.update()

    leds.on()
    time.sleep(1)
    leds.off()
    time.sleep(4) # change to 1 or 7 for other fishes

leds.on()
sleep(1)
leds.off()
sleep(1)

idle()

for i in range(60): #fb 60
    update_sensor(i)


GPIO.cleanup()
