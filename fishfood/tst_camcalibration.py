from lib_utils import *
from lib_leds import LEDS

import time
import os
import numpy as np

import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(13, GPIO.OUT)
CAMLED = 40
GPIO.setup(CAMLED, GPIO.OUT)
GPIO.output(CAMLED, False)

from picamera import PiCamera
picam = PiCamera()
picam.resolution = (2592, 1944)
picam.framerate = 15
picam.rotation = 180


class Blob():

    """Blob detection. Returns coordinates of all blobs

    This class takes a camera image and returns the pixel coordinates of all blobs.
    It contains functions to convert the image to grayscale, threshold the image to
    separate blob pixels from background, create lists of indices of pixels which
    belong to single blobs, and calculate the center of each blob.
    """

    def __init__(self, img_raw, side, thresh=60, x_res=U_CAM_XRES, y_res=U_CAM_YRES):
        """Load a new image
        
        Arguments:
            img_raw {} -- camera image

        Keyword Arguments:
            thresh {int} -- detection threshold that separates background from blob pixels
            x_res {int} -- x-resolution of the image
            y_res {int} -- y-resolution of the image
        """

        # Parameters
        self.img_raw = img_raw
        self.side = side
        self.thresh = thresh
        self.x_res = x_res
        self.y_res = y_res

        # Initializations
        self.blob_size = 0
        self.blobs = np.zeros((1, 2))
        self.no_blobs = 0

    def __del__(self):
    	return

    def blob_detect(self):
        """Runs all subfunctions for blob detection"""
        img_gray = self._raw_to_gray()
        blob_pixels = self._thresholding(img_gray)
        self._continuity(blob_pixels)

    def _raw_to_gray(self):
        """Converts the image to grayscale"""
        img_rgb = np.zeros((self.y_res, self.x_res, 3), dtype=np.uint8)
        img_rgb = np.array(self.img_raw)
        img_gray = np.zeros((self.y_res, self.x_res))
        img_gray[:, :] = img_rgb[:, :, 2]

        return img_gray

    def _thresholding(self, img_gray):
        """Thresholds the gray image and returns blob pixels

        Arguments:
            img_gray {} -- grayscale image
        """
        blob_pixels = np.where(img_gray > self.thresh)
        blob_pixels = np.asarray(blob_pixels)

        return blob_pixels

    def _continuity(self, blob_pixels):
        """Clusters blob pixels and returns lists of single blob centroids

        This method checks all blob pixels for continuity in x-direction. It then checks the subsets which are continous in x-direction for continuity in y-direction. It finally returns an array that contains the centroids of individual blobs.
        
        Arguments:
            blob_pixels {} -- array of pixels that belong to blobs
        """

        # Total amount of blob pixels. If none, return.
        self.blob_size = blob_pixels.size
        if self.blob_size < 4:
            self.blobs = np.zeros(0)
            return

        # Find pixels that are continuous in x-direction
        x = blob_pixels[0, :]
        x_shifted = np.zeros(x.shape)
        x_shifted[1:-1] = np.copy(x[:-2])
        x_shifted[0] = -1
        x_shifted[-1] = -1

        blob_x = np.where(abs(x_shifted - x) > 1)
        blob_x = np.asarray(blob_x)
        blob_x[:, -1] += 1

        # For each continous set in x-direction, find pixels that are also continuous in y-direction
        for i in range(0, blob_x.shape[1]-1):
            x = blob_pixels[0, blob_x[0, i]:blob_x[0, i+1]]
            y = blob_pixels[1, blob_x[0, i]:blob_x[0, i+1]]
            arg_y = np.argsort(y)
            y_sorted = np.sort(y)
            
            y_shifted = np.zeros(y.shape)
            y_shifted[1:-1] = np.copy(y_sorted[:-2])
            y_shifted[0] = -1
            y_shifted[-1] = -1

            blob_y = np.where(abs(y_shifted - y_sorted) > 1)
            blob_y = np.asarray(blob_y)
            blob_y[:, -1] += 1
            
            # For pixels continuous in x- and y-direction, find centroids
            for j in range(0, blob_y.shape[1]-1):
                blob_indices = arg_y[np.asscalar(blob_y[:, j]):np.asscalar(blob_y[:, j+1])]
                x_center_temp = round(sum(x[blob_indices])/blob_indices.shape[0], 3)
                y_center_temp = round(sum(y[blob_indices])/blob_indices.shape[0], 3)

                # coordinate system with origin at image center, x-axis looking forward, y-axis looking upward
                # x-horizontal, y-vertical
                x_center = y_center_temp
                y_center = x_center_temp
                # x-forward, y-upward
                x_center = self.x_res - x_center
                y_center = self.y_res - y_center
                # x-y-centered
                x_center = x_center - (self.x_res / 2)
                y_center = y_center - (self.y_res / 2)
                # flip image 180 degrees bcs camera mounted upside down
                if self.side == 'right':
                    x_center = -x_center
                y_center = -y_center

                if self.no_blobs == 0:
                    #self.blobs[0, 0] = x_center
                    #self.blobs[0, 1] = y_center

                    # only for calib:
                    self.blobs[0, 0] = self.y_res - x_center_temp
                    self.blobs[0, 1] = self.x_res - y_center_temp

                else:
                    #self.blobs = np.append(self.blobs, [[x_center, y_center]], axis=0)

                    # only for calib:
                    self.blobs = np.append(self.blobs, [[self.y_res - x_center_temp, self.x_res - y_center_temp]], axis=0)
        
                self.no_blobs += 1


def flash_leds():
    leds.on()
    time.sleep(0.25)
    leds.off()

def calib_pictures(no_pictures):
    for i in range(no_pictures):
        # status
        print(i)

        # sleep
        time.sleep(3)
        
        # light led to ready people for photo
        flash_leds()
        # take photos
        picam.capture('./{}/calib{}.jpg'.format(U_FILENAME, i))
        flash_leds()

def known_blobs(detection_threshold):
    # cam settings
    picam.color_effects = (128, 128)
    picam.awb_mode = 'off'
    picam.awb_gains = (1, 1)
    picam.iso = 120
    picam.brightness = 30
    picam.contrast = 100

    # low res 192 by 144
    picam.resolution = (192, 144)
    picam.framerate = 60

    picam.rotation = 180
    picam.capture('low.jpg', use_video_port=True)
    picam.rotation = 0

    img_low = np.empty((144, 192, 3), dtype=np.uint8)
    picam.capture(img_low, 'rgb', use_video_port=True)
    blobs_right_low = Blob(img_low, 'right', detection_threshold, 192, 144)
    blobs_right_low.blob_detect()
    print('low res blob:')
    print(blobs_right_low.blobs)

    # full res 2592 by 1944
    picam.resolution = (2592, 1944)
    picam.framerate = 15

    picam.rotation = 180
    picam.capture('full.jpg')
    picam.rotation = 0


U_FILENAME = time.strftime("%y%m%d_UTC_%H%M%S")
leds = LEDS()
#os.makedirs('./{}/'.format(U_FILENAME))
#calib_pictures(20) # change number of pictures
known_blobs(10) # change detection_threshold based on distance [20-160]
GPIO.cleanup()
