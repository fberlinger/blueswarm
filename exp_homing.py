import RPi.GPIO as GPIO

import csv
import time
import datetime
import math
import numpy as np
from picamera import PiCamera

import utils
import move
from camera import Camera
from blob import Blob



class Homing(): 

    def __init__(self, run_time=20, exp_name='unnamed'):

        self.run_time = run_time
        self.t_capture_l = 0
        self.t_capture_r = 0
        self.t_blob_l = 0
        self.t_blob_r = 0
        self.t_loop = 0
        self.thresh_orbiting = 0.4

        # logger instance
        self.exp_name = exp_name
        with open('{}.log'.format(self.exp_name), 'w') as f:
            f.truncate()
            f.write('t_loop    :: t_capture_l :: t_capture_r :: t_blob_l :: t_blob_r ::(ABSOLUTE TIME)\n')
        
        with open('{}.csv'.format('blobs'), 'w') as f:
            f.truncate()



    def log_status(self):
        with open('{}.log'.format(self.exp_name), 'a') as f:
            f.write(
                '{:05} :: {:05} :: {:05} :: {:05} :: {:05} :: ({})\n'.format(
                    self.t_loop, self.t_capture_l, self.t_capture_r, self.t_blob_l, self.t_blob_r, datetime.datetime.now()
                    )
                )

    def log_blobs(self):
        with open('{}.csv'.format('blobs'), 'a') as f:
            writer = csv.writer(f, delimiter=',')
            row = []
            for i in range(self.blob_list.size):
                row.append(self.blob_list[0, i])
            writer.writerow(row)


    def run(self):
        self.homing()
        #self.orbiting()

    def homing(self):
        time_start = time.time()
        t_loop = 0

        total_no_pixels = (U_CAM_YRES / 2)** * math.pi

        while time.time() - time_start < self.run_time:
            # analyze right side
            t_capture_l = time.time()
            img = camera.capture('right')
            self.t_capture_l = time.time() - t_capture_l
            t_blob_l = time.time()
            blobs_right = Blob(img)
            blobs_right.blob_detect()
            self.t_blob_l = time.time() - t_blob_l
            print(blobs_right.blobs)
            print(blobs_right.blob_size)
            print(blobs_right.no_blobs)

            self.blob_list = 255 * np.ones((3, 2))
            self.blob_list[:blobs_right.blobs.shape[0], :blobs_right.blobs.shape[1]] = blobs_right.blobs
            self.blob_list = self.blob_list.reshape((1,6))

            self.log_blobs()

            # analyze left side
            img = camera.capture('left')
            blobs_left = Blob(img)
            blobs_left.blob_detect()
            print(blobs_left.blobs)
            print(blobs_left.blob_size)
            print(blobs_left.no_blobs)


            if blobs_right.blob_size and blobs_left.blob_size:
                print('fwd')
                #move.forward()
                
                # initialize orbiting
                total_blob_pixels = blobs_left.blob_size + blobs_right.blob_size
                blob_ratio =  total_blob_pixels / (2 * total_no_pixels)
                
                #if blob_ratio > self.thresh_orbiting:
                #    return

            elif blobs_right.blobs.size:
                print('left_fin')
                #move.cw()

            else:
                print('right_fin')
                #move.ccw()

            self.t_loop = time.time() - t_loop
            self.log_status()
            t_loop = time.time()

        move.stop()
        move.terminate()

    def orbiting(self):
        while blobs_left.blobs[0, 0] > 0:
            move.cw()

        time_start = time.time()
        while time.time() - time_start < self.run_time:
            move.forward()
            if blobs_left.blobs[0, 0] < U_CAM_YRES / 5:
                move.ccw()

    def depth_ctrl_from_cam(self):
        if blobs_left.blobs[0, 1] > 0:
                move.down()


if __name__ == "__main__":
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # arm motors
    move.initialize()

    # create camera and choose settings
    camera = Camera()
    
    homing = Homing(10, 'exp_1')
    homing.run()

move.stop()
move.terminate()
