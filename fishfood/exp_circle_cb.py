from lib_fin_simple_c import *
from time import sleep

import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

import numpy as np
from lib_camera import Camera
import threading
from lib_leds import LEDS
from lib_depthsensor import DepthSensor
from lib_fin import Fin
from lib_photodiode import Photodiode

depth_sensor = DepthSensor()
depth_sensor.update()
surface_pressure = depth_sensor.pressure_mbar

leds = LEDS()

photodiode = Photodiode()

caudal = CaudalFin()
caudal.off()
pecl = PecLFin()
pecl.off()
pecr = PecRFin()
pecr.off()
dorsal = DorsalFin()
dorsal.off()



cam_l = Camera('left')
cam_r = Camera('right')
cam_l.colorbot_settings()
cam_r.colorbot_settings()

mask_l = np.loadtxt("/home/pi/fishfood/mask_l.txt", dtype=np.int32)
mask_r = np.loadtxt("/home/pi/fishfood/mask_r.txt", dtype=np.int32)
mask_cb = np.loadtxt("/home/pi/fishfood/mask_cb.txt", dtype=np.int32)
img_dark = 255*np.ones((192, 256))

sensor_value = 0
pecl_state = 0
pecr_state = 0
dorsal_state = 0

dive = 0

caudal.off()
dorsal.off()
pecl.down()
pecr.down()
sleep(1)
pecl.off()
pecr.off()
caudal.off()
dorsal.off()


def update_sensor_local():
    global sensor_value
    cam_l.capture()
    cam_r.capture()

    L = cam_l.img
    R = cam_r.img

    LM = np.multiply(img_dark-L[:, :, 2], mask_cb)
    RM = np.multiply(img_dark-R[:, :, 2], mask_cb)

    LA = np.multiply(LM, mask_l)
    RA = np.multiply(RM, mask_r)

    sensor_value = (np.sum(LA) > 30) or (np.sum(RA) > 30) #fb check 30 threshold

def update_sensor():
    x = threading.Thread(target=update_sensor_local)
    x.start()

def rotate_left(T_caudal, N_pectoral, N_dorsal, t_caudal, t_cam):
    global pecl_state
    global pecr_state
    global dorsal_state
    global dive

    caudal.left()

    pecl.off()

    if (dive == 1):
        if (dorsal_state == 0):
            dorsal.left()
        else:
            dorsal.right()

    # Ensure pectoral fin is in right position so first flip generates thrust
    if (pecr_state == 0):
        pecr.down()
    else:
        pecr.up()


    # --------------------------------------------------------
    # GENERATE TIMINGS AND EVENTS SEQUENCE
    #---------------------------------------------------------
    t_pectoral = np.linspace(0, T_caudal, N_pectoral + 1)
    t_pectoral = np.delete(t_pectoral, 0)

    t_dorsal = np.linspace(0, T_caudal, N_dorsal + 1)
    t_dorsal = np.delete(t_dorsal, 0)

    t = np.array([0])
    t = np.append(t, t_pectoral)
    t = np.append(t, t_dorsal)
    t = np.append(t, t_caudal)
    t = np.append(t, t_cam)
    waits = np.diff(np.sort(t))

    events = 1*np.ones(N_pectoral)
    events = np.append(events, 2*np.ones(N_dorsal))
    events = np.append(events, [3])
    events = np.append(events, [4])

    # SORT EVENTS
    t_events = np.delete(t, 0)
    inds = t_events.argsort()
    events = events[inds]
    events = events.astype(int)
    #----------------------------------------------------------

    for i in range(np.size(waits)):
        sleep(waits[i])
        if (events[i] == 1):
            if (pecr_state == 0):
                pecr.up()
            else:
                pecr.down()
            pecr_state = 1 - pecr_state
        if (events[i] == 2 and dive == 1):
            if (dorsal_state == 0):
                dorsal.right()
            else:
                dorsal.left()
            dorsal_state = 1 - dorsal_state
        if (events[i] == 3):
            caudal.right()
        if (events[i] == 4):
            update_sensor()

def rotate_right(T_caudal, N_pectoral, N_dorsal, t_caudal, t_cam):
    global pecl_state
    global pecr_state
    global dorsal_state
    global dive

    caudal.right()

    pecr.off()

    if (dive == 1):
        if (dorsal_state == 0):
            dorsal.right()
        else:
            dorsal.left()

    # Ensure pectoral fin is in right position so first flip generates thrust
    if (pecl_state == 0):
        pecl.down()
    else:
        pecl.up()


    # --------------------------------------------------------
    # GENERATE TIMINGS AND EVENTS SEQUENCE
    #---------------------------------------------------------
    t_pectoral = np.linspace(0, T_caudal, N_pectoral + 1)
    t_pectoral = np.delete(t_pectoral, 0)

    t_dorsal = np.linspace(0, T_caudal, N_dorsal + 1)
    t_dorsal = np.delete(t_dorsal, 0)

    t = np.array([0])
    t = np.append(t, t_pectoral)
    t = np.append(t, t_dorsal)
    t = np.append(t, t_caudal)
    t = np.append(t, t_cam)
    waits = np.diff(np.sort(t))

    events = 1*np.ones(N_pectoral)
    events = np.append(events, 2*np.ones(N_dorsal))
    events = np.append(events, [3])
    events = np.append(events, [4])

    # SORT EVENTS
    t_events = np.delete(t, 0)
    inds = t_events.argsort()
    events = events[inds]
    events = events.astype(int)
    #----------------------------------------------------------

    for i in range(np.size(waits)):
        sleep(waits[i])
        if (events[i] == 1):
            if (pecl_state == 0):
                pecl.up()
            else:
                pecl.down()
            pecl_state = 1 - pecl_state
        if (events[i] == 2 and dive == 1):
            if (dorsal_state == 0):
                dorsal.left()
            else:
                dorsal.right()
            dorsal_state = 1 - dorsal_state
        if (events[i] == 3):
            caudal.left()
        if (events[i] == 4):
            update_sensor()

def idle():
    """Waiting for starting signal
    """
    thresh_photodiode = 50 # lights off: 2, lights on: 400 -> better range!

    while photodiode.brightness > thresh_photodiode:
        photodiode.update()

    leds.on()
    sleep(1)
    leds.off()
    sleep(2)


leds.on()
sleep(1)
leds.off()
sleep(2)

idle()

'''
depth_mm = 0
pecl.off()
pecr.off()
caudal.off()
while (depth_mm < 600):
    dorsal.left()
    sleep(0.1)
    dorsal.right()
    sleep(0.1)
    depth_sensor.update()
    depth_mm = max(0, (depth_sensor.pressure_mbar - surface_pressure) * 10.197162129779)
dorsal.off()
'''

#for i in range(240):
for i in range(round(240*0.4/0.3)):
    if (sensor_value == True):
        #rotate_left(0.4, 5, 3, 0.25, 0.3)
        rotate_left(0.25, 5, 3, 0.15, 0.3)
    else:
        #rotate_right(0.4, 5, 3, 0.25, 0.3)
        rotate_right(0.25, 5, 3, 0.15, 0.3)

GPIO.cleanup()
