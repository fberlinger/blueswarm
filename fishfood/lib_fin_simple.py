import RPi.GPIO as GPIO

from lib_utils import *

class CaudalFin():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(U_FIN_CL, GPIO.OUT)
        GPIO.setup(U_FIN_CR, GPIO.OUT)

    def left(self):
        GPIO.output(U_FIN_CL, GPIO.HIGH)
        GPIO.output(U_FIN_CR, GPIO.LOW)
        
    def right(self):
        GPIO.output(U_FIN_CL, GPIO.LOW)
        GPIO.output(U_FIN_CR, GPIO.HIGH)

    def off(self):
        GPIO.output(U_FIN_CL, GPIO.LOW)
        GPIO.output(U_FIN_CR, GPIO.LOW)

class PecLFin():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(U_FIN_PLU, GPIO.OUT)
        GPIO.setup(U_FIN_PLD, GPIO.OUT)

    def up(self):
        GPIO.output(U_FIN_PLU, GPIO.HIGH)
        GPIO.output(U_FIN_PLD, GPIO.LOW)
        
    def down(self):
        GPIO.output(U_FIN_PLU, GPIO.LOW)
        GPIO.output(U_FIN_PLD, GPIO.HIGH)

    def off(self):
        GPIO.output(U_FIN_PLU, GPIO.LOW)
        GPIO.output(U_FIN_PLD, GPIO.LOW)

class PecRFin():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(U_FIN_PRU, GPIO.OUT)
        GPIO.setup(U_FIN_PRD, GPIO.OUT)

    def up(self):
        GPIO.output(U_FIN_PRU, GPIO.HIGH)
        GPIO.output(U_FIN_PRD, GPIO.LOW)
        
    def down(self):
        GPIO.output(U_FIN_PRU, GPIO.LOW)
        GPIO.output(U_FIN_PRD, GPIO.HIGH)

    def off(self):
        GPIO.output(U_FIN_PRU, GPIO.LOW)
        GPIO.output(U_FIN_PRD, GPIO.LOW)

class DorsalFin():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(U_FIN_DL, GPIO.OUT)
        GPIO.setup(U_FIN_DR, GPIO.OUT)

    def left(self):
        GPIO.output(U_FIN_DL, GPIO.HIGH)
        GPIO.output(U_FIN_DR, GPIO.LOW)
        
    def right(self):
        GPIO.output(U_FIN_DL, GPIO.LOW)
        GPIO.output(U_FIN_DR, GPIO.HIGH)

    def off(self):
        GPIO.output(U_FIN_DL, GPIO.LOW)
        GPIO.output(U_FIN_DR, GPIO.LOW)
