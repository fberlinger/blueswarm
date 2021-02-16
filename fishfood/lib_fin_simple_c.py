import RPi.GPIO as GPIO

from lib_utils import *

class CaudalFin():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(U_FIN_C1, GPIO.OUT)
        GPIO.setup(U_FIN_C2, GPIO.OUT)

    def left(self):
        GPIO.output(U_FIN_C2, GPIO.HIGH)
        GPIO.output(U_FIN_C1, GPIO.LOW)
        
    def right(self):
        GPIO.output(U_FIN_C2, GPIO.LOW)
        GPIO.output(U_FIN_C1, GPIO.HIGH)

    def off(self):
        GPIO.output(U_FIN_C1, GPIO.LOW)
        GPIO.output(U_FIN_C2, GPIO.LOW)

class PecLFin():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(U_FIN_PL1, GPIO.OUT)
        GPIO.setup(U_FIN_PL2, GPIO.OUT)

    def up(self):
        GPIO.output(U_FIN_PL1, GPIO.HIGH)
        GPIO.output(U_FIN_PL2, GPIO.LOW)
        
    def down(self):
        GPIO.output(U_FIN_PL1, GPIO.LOW)
        GPIO.output(U_FIN_PL2, GPIO.HIGH)

    def off(self):
        GPIO.output(U_FIN_PL1, GPIO.LOW)
        GPIO.output(U_FIN_PL2, GPIO.LOW)

class PecRFin():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(U_FIN_PR1, GPIO.OUT)
        GPIO.setup(U_FIN_PR2, GPIO.OUT)

    def up(self):
        GPIO.output(U_FIN_PR1, GPIO.HIGH)
        GPIO.output(U_FIN_PR2, GPIO.LOW)
        
    def down(self):
        GPIO.output(U_FIN_PR1, GPIO.LOW)
        GPIO.output(U_FIN_PR2, GPIO.HIGH)

    def off(self):
        GPIO.output(U_FIN_PR1, GPIO.LOW)
        GPIO.output(U_FIN_PR2, GPIO.LOW)

class DorsalFin():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(U_FIN_D1, GPIO.OUT)
        GPIO.setup(U_FIN_D2, GPIO.OUT)

    def left(self):
        GPIO.output(U_FIN_D1, GPIO.HIGH)
        GPIO.output(U_FIN_D2, GPIO.LOW)
        
    def right(self):
        GPIO.output(U_FIN_D1, GPIO.LOW)
        GPIO.output(U_FIN_D2, GPIO.HIGH)

    def off(self):
        GPIO.output(U_FIN_D1, GPIO.LOW)
        GPIO.output(U_FIN_D2, GPIO.LOW)
