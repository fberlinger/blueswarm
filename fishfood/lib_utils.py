import time
from math import *

U_FILENAME = time.strftime("%y%m%d_%H%M%S") # date_time

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
# calibration parameters
U_CAM_xc = 9.890661037168528e+02
U_CAM_yc = 1.240887736413883e+03
U_CAM_c = 1.000095457005999
U_CAM_d = 1.348821585691861e-04
U_CAM_e = -2.766447240107194e-05
U_CAM_ss = 100 * np.array([-5.506324725325824, 0, 0.000004199699653, -0.000000000365763, 0.000000000002906])
U_CAM_ss = np.flip(ss, 0)
# resolution
U_CAM_MRES = 144 # U_CAM_MRES has to be multiple of 16, U_CAM_NRES of 32
U_CAM_NRES = 192 # aspect ratio U_CAM_NRES / U_CAM_MRES has to be 4 / 3
# mounting
U_CAM_ALPHA = 10 * pi / 180 # [rad] camera mounting angle
U_CAM_DX = 25 # [mm] camera x-offset to center of rotation
U_CAM_DY = 20 # [mm] camera y-offset to center of rotation

# LEDS
U_LED_DX = 84 # [mm] leds x-distance
U_LED_DZ = 84 # [mm] leds y-distance

# BLOB
U_BLOB_TRESH = 60 # default detection threshold
