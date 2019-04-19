import numpy as np
from math import *

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


def mn_to_uvw(mn):
    """Uses camera calibration to derive uvw rays in the camera frame from mn coordinates in the image plane.
    
    Args:
        mn (float): Pixel coordinates of blob centroids
    
    Returns:
        float: uvw rays in the camera frame 
    """
    scaling_factor = 2592 / U_CAM_NRES

    A_inv = np.array([[1., -U_CAM_d], [-U_CAM_e, U_CAM_c]])
    T = np.array([[U_CAM_xc, U_CAM_yc]]).T
    mn_dash = A_inv @ (scaling_factor * mn - T)

    rho = np.linalg.norm(mn_dash, axis=0)
    z = np.polyval(U_CAM_ss, rho)

    uvw = np.vstack((mn_dash, z))
    uvw_norm = np.linalg.norm(uvw, axis=0)
    uvw = uvw / uvw_norm[None, :]

    return uvw

def uvw_to_pqr_r(uvw):
    """Transforms from the camera frame to the robot frame by rotating the camera by 10deg and switching axes.
    
    Args:
        uvw (float): uvw rays in the camera frame
    
    Returns:
        float: pqr rays in the robot frame
    """
    rot_angle = np.array([[1, 0, 0], [0, cos(
        U_CAM_ALPHA), -sin(U_CAM_ALPHA)], [0, sin(U_CAM_ALPHA), cos(U_CAM_ALPHA)]])
    rot_axes = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])

    pqr = rot_axes @ rot_angle @ uvw
    pqr_norm = np.linalg.norm(pqr, axis=0)
    pqr = pqr / pqr_norm[None, :]

    return pqr


def uvw_to_pqr_l(uvw):
        """Transforms from the camera frame to the robot frame by rotating the camera by 10deg and switching axes.
        
        Args:
            uvw (float): uvw rays in the camera frame
        
        Returns:
            float: pqr rays in the robot frame
        """
        rot_angle = np.array([[1, 0, 0], [
                             0, cos(-U_CAM_ALPHA), -sin(-U_CAM_ALPHA)], [0, sin(-U_CAM_ALPHA), cos(-U_CAM_ALPHA)]])
        rot_axes = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])

        pqr = rot_axes @ rot_angle @ uvw
        pqr_norm = np.linalg.norm(pqr, axis=0)
        pqr = pqr / pqr_norm[None, :]

        return pqr




mask_r = np.zeros((192, 256))
mask_l = np.zeros((192, 256))

print("Hello world")

alpha = 10

for m in range(192):
    print(m)
    for n in range(256):
        mn = np.array([[m], [n]])

        uvw = mn_to_uvw(mn)

# RIGHT CAMERA

        pqr = uvw_to_pqr_r(uvw)

        p = pqr[0]
        q = pqr[1]

        angle = (atan2(q, p)*180/np.pi)

        if ((angle > 0) and (angle < alpha)):
            mask_r[U_CAM_MRES - m, U_CAM_NRES - n] = 1

# LEFT CAMERA

        pqr = uvw_to_pqr_l(uvw)     

        p = pqr[0]
        q = pqr[1]

        angle = (atan2(q, p)*180/np.pi)

        if ((angle < 0) and (angle > -alpha)):
            mask_l[U_CAM_MRES - m, U_CAM_NRES - n] = 1


f = open("mask_r.txt", "w")
for m in range(192):
    print(m)
    for n in range(256):
        f.write(str(mask_r[m, n]) + " ")
    f.write("\n")


f = open("mask_l.txt", "w")
for m in range(192):
    print(m)
    for n in range(256):
        f.write(str(mask_l[m, n]) + " ")
    f.write("\n")

print("DONE")