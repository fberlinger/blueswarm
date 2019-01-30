"""Utils library, contains BlueBot parameters in a single place, such that all changes can be made here.

Attributes:
    U_BLOB_THRESH (int): Light intensity for pixel to be considered LED blob pixel, [0=LOW,255=HIGH]
    U_CAM_ALPHA (float): Camera mounting angle
    U_CAM_c (float): Camera calibration parameter
    U_CAM_d (float): Camera calibration parameter
    U_CAM_DX (int): Camera x-offset to center of rotation
    U_CAM_DY (int): Camera y-offset to center of rotation
    U_CAM_e (float): Camera calibration parameter
    U_CAM_MRES (int): Image resolution in m-direction (vertical)
    U_CAM_NRES (int): Image resolution in n-direction (horizontal)
    U_CAM_ss (float): Camera calibration parameter
    U_CAM_xc (float): Camera calibration parameter
    U_CAM_yc (float): Camera calibration parameter
    U_FILENAME (string): Time-stamped filename for logger files
    U_FIN_C1 (int): Caudal pin
    U_FIN_C2 (int): Caudal pin
    U_FIN_D1 (int): Dorsal pin
    U_FIN_D2 (int): Dorsal pin
    U_FIN_PL1 (int): Pectoral left pin
    U_FIN_PL2 (int): Pectoral left pin
    U_FIN_PR1 (int): Pectoral right pin
    U_FIN_PR2 (int): Pectoral right pin
    U_LED_DX (int): LEDs x-distance (horizontal)
    U_LED_DZ (int): LEDs z-distance (vertical)
"""
import time
import numpy as np
from math import *


###############################################################################
# ROBOT SPECS                                                                 #
###############################################################################
# Robot:                                                                      #
#  looking from above: x-fwd (roll), y-right (pitch), z-down (yaw)            #
#  x                                                                          #
#  .       z                                                                  #
#  .     .                                                                    #
#  .   .                                                                      #
#  . .                                                                        #
#  . . . . . y                                                                #
#-----------------------------------------------------------------------------#
# Image:                                                                      #
#  origin top left, m-vertically down, n-horizontally right                   #
#  . . . . n                                                                  #
#  .                                                                          #
#  .                                                                          #
#  m                                                                          #
#                                                                             #
# Projected image in camera frame:                                            #
#  origin middle, uv // mn, w inwards, unit sphere                            #
#        . . . . v                                                            #
#      . .                                                                    #
#    .   .                                                                    #
#  w     u                                                                    #
#                                                                             #
# Projected image in robot frame:                                             #
#  uvw rotated +/-10deg around u, uvw mapped on pqr such that pqr // xyz,     #
#  unit sphere                                                                #
#  p                                                                          #
#  .       r                                                                  #
#  .     .                                                                    #
#  .   .                                                                      #
#  . .                                                                        #
#  . . . . . q                                                                #
#                                                                             #
# World image in robot frame:                                                 #
#  xyz // pqr, [dx +/-dy 0]' camera offsets; world distances [mm]             #
#  x                                                                          #
#  .       z                                                                  #
#  .     .                                                                    #
#  .   .                                                                      #
#  . .                                                                        #
#  . . . . . y                                                                #
###############################################################################

# FILENAME
U_FILENAME = time.strftime("%y%m%d_%H%M%S") # date_time

# FINS
U_FIN_C1 = 20
U_FIN_C2 = 21
U_FIN_D1 = 19
U_FIN_D2 = 26
U_FIN_PR1 = 4
U_FIN_PR2 = 17
U_FIN_PL1 = 18
U_FIN_PL2 = 23

# CAMERA
# camera side
U_CAM_RIGHT = True # False for BB01
U_CAM_LEFT = False # True for BB01
# calibration parameters
U_CAM_xc = 9.890661037168528e+02
U_CAM_yc = 1.240887736413883e+03
U_CAM_c = 1.000095457005999
U_CAM_d = 1.348821585691861e-04
U_CAM_e = -2.766447240107194e-05
U_CAM_ss = 100 * np.array([-5.506324725325824, 0, 0.000004199699653, -0.000000000365763, 0.000000000002906])
U_CAM_ss = np.flip(U_CAM_ss, 0)
# resolution
U_CAM_MRES = 192 #144# U_CAM_MRES has to be multiple of 16, U_CAM_NRES of 32
U_CAM_NRES = 256 #192# aspect ratio U_CAM_NRES / U_CAM_MRES has to be 4 / 3
# mounting
U_CAM_ALPHA = radians(-10) # [rad] camera mounting angle
U_CAM_DX = 25 # [mm] camera x-offset to center of rotation
U_CAM_DY = 20 # [mm] camera y-offset to center of rotation

# LEDS
U_LED_DX = 86 # [mm] leds x-distance on BlueBot
U_LED_DZ = 86 # [mm] leds z-distance on BlueBot
#U_LED_DZ = 150 # [mm] leds z-distance on orbit-beacon

# BLOB
U_BLOB_THRESH = 60 # default detection threshold
