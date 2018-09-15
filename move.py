# Source: https://stackoverflow.com/questions/12435211/python-threading-timer-repeat-function-every-n-seconds

from threading import Timer, Thread, Event
from datetime import datetime
import RPi.GPIO as GPIO
import time

class Move(Thread):
    def __init__(self, stop_flag, terminate_flag, PIN_1, PIN_2, period):
        Thread.__init__(self)
        self.stopped = stop_flag
        self.terminated = terminate_flag
        
        self.PIN_1 = PIN_1
        self.PIN_2 = PIN_2
        self.period = period
        
        self.state = 0
        self.direction = 0

    def set_period(self, period):
        self.period = period
        
    def run(self):
        while 1:
            if not self.stopped.wait(self.period / 2):
                self.state = 1
                if self.direction == 0:
                    GPIO.output(self.PIN_1, GPIO.HIGH)
                    GPIO.output(self.PIN_2, GPIO.LOW)
                else:
                    GPIO.output(self.PIN_1, GPIO.LOW)
                    GPIO.output(self.PIN_2, GPIO.HIGH)
                self.direction = 1 - self.direction
            elif self.state == 1:
                self.state = 0
                GPIO.output(self.PIN_1, GPIO.LOW)
                GPIO.output(self.PIN_2, GPIO.LOW)
                
            if not self.terminated.wait(0):
                GPIO.output(self.PIN_1, GPIO.LOW)
                GPIO.output(self.PIN_2, GPIO.LOW)
                break

terminate_flag = Event()
                
caudal_fin_flag = Event()
caudal_fin_flapper = Move(caudal_fin_flag, terminate_flag, 20, 21, 0.2)

dorsal_fin_flag = Event()
dorsal_fin_flapper = Move(dorsal_fin_flag, terminate_flag, 19, 26, 0.2)

pectoral_left_fin_flag = Event()
pectoral_left_fin_flapper = Move(pectoral_left_fin_flag, terminate_flag, 18, 23, 0.2)

pectoral_right_fin_flag = Event()
pectoral_right_fin_flapper = Move(pectoral_right_fin_flag, terminate_flag, 4, 17, 0.2)
                
def initialize():
    GPIO.setmode(GPIO.BCM)

    # Caudal
    GPIO.setup(20, GPIO.OUT)
    GPIO.setup(21, GPIO.OUT)
    # Dorsal
    GPIO.setup(19, GPIO.OUT)
    GPIO.setup(26, GPIO.OUT)
    # Pectoral left
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(23, GPIO.OUT)
    # Pectoral right
    GPIO.setup(4, GPIO.OUT)
    GPIO.setup(17, GPIO.OUT)

    terminate_flag.set()
    
    caudal_fin_flag.set()
    caudal_fin_flapper.start()

    dorsal_fin_flag.set()
    dorsal_fin_flapper.start()

    pectoral_left_fin_flag.set()
    pectoral_left_fin_flapper.start()

    pectoral_right_fin_flag.set()
    pectoral_right_fin_flapper.start()
    
def forward():
    caudal_fin_flag.clear()
    dorsal_fin_flag.set()
    pectoral_left_fin_flag.set()
    pectoral_right_fin_flag.set()
    
def backward():
    caudal_fin_flag.set()
    dorsal_fin_flag.set()
    pectoral_left_fin_flag.clear()
    pectoral_right_fin_flag.clear()
    
def down():
    caudal_fin_flag.set()
    dorsal_fin_flag.clear()
    pectoral_left_fin_flag.set()
    pectoral_right_fin_flag.set()
    
def ccw():
    caudal_fin_flag.set()
    dorsal_fin_flag.set()
    pectoral_left_fin_flag.set()
    pectoral_right_fin_flag.clear()

def cw():
    caudal_fin_flag.set()
    dorsal_fin_flag.set()
    pectoral_left_fin_flag.clear()
    pectoral_right_fin_flag.set()

def stop():
    caudal_fin_flag.set()
    dorsal_fin_flag.set()
    pectoral_left_fin_flag.set()
    pectoral_right_fin_flag.set()
    
def terminate():
    terminate_flag.clear()
    time.sleep(1)
    GPIO.cleanup()
