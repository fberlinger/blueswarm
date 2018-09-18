import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

from PIL import Image

import utils
from camera import Camera

camera = Camera(True)
img_right = camera.capture('right')
img_r = np.zeros((U_CAM_YRES, U_CAM_XRES, 3), dtype=np.uint8)
img_r = np.array(img_r)
im = Image.fromarray(img_r)
im.save("img_right_from_array.jpeg")

img_left = camera.capture('left')
img_l = np.zeros((U_CAM_YRES, U_CAM_XRES, 3), dtype=np.uint8)
img_l = np.array(img_l)
im = Image.fromarray(img_l)
im.save("img_left_from_array.jpeg")
