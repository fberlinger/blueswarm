"""Vision library, perceives BlueBots environement and returns coordinates of neihgboring BlueBots.
"""
import RPi.GPIO as GPIO
import numpy as np
import time

from math import *
from lib_utils import *
from lib_camera import Camera
from lib_blob import Blob


class Vision():

    """Vision updates BlueBot's visual perception, using both cameras, blob detection, and transformations from pixel coordinates to normalized rays (pqr) or true-distance rays (xyz) in the robot frame.
    
    Normalized rays allow for the computation of the angular direction of an observed object. True-distance rays also tell the distance to this object but are only possible to derive if the true distance of two points in the image is known, e.g., of two vertically stacked LEDs.
    
    Attributes:
        pqr_l (float): Normalized rays from the left camera in the robot frame
        pqr_r (float): Normalized rays from the right camera in the robot frame
        xyz_l (float): True-distance rays from the left camera in the robot frame [mm]
        xyz_r (float): True-distance rays from the right camera in the robot frame [mm]
    """

    def __init__(self, max_blobs):
        """Camera and Blob are classes for image taking and processing respectively.
        
        Args:
            max_blobs (int): maximum number of blobs in environemt
        """
        self._cam_r = Camera('right')
        self._cam_l = Camera('left')
        self._blob_r = Blob('right', max_blobs, 40)  # detection threshold
        self._blob_l = Blob('left', max_blobs, 40)  # detection threshold

        self.pqr_r = np.zeros((3, 1))
        self.pqr_l = np.zeros((3, 1))
        self.xyz_r = np.zeros((3, 1))
        self.xyz_l = np.zeros((3, 1))

    def update(self):
        """Takes images with both cameras, finds blob centroids, and transforms pixel coordinates to rays in the robot frame.
        """
        # check right side
        #t_now = time.time()
        self._cam_r.capture()
        #t_capture = time.time() - t_now
        #t_now = time.time()
        self._blob_r.detect(self._cam_r.img)
        #t_blob = time.time() - t_now

        # check left side
        self._cam_l.capture()
        self._blob_l.detect(self._cam_l.img)

        # if blobs detected, transform them to coordinates in the robot frame
        #t_uvw = -1
        #t_pqr = -1
        #t_xyz = -1
        if self._blob_r.no_blobs:
            # transform from image coordinates (mn) to coordinates on the unit sphere in the camera frame (uvw)
            #t_now = time.time()
            uvw_r = self._mn_to_uvw(self._blob_r.blobs)
            #t_uvw = time.time() - t_now
            # align camera frame with robot frame (uvw to pqr)
            #t_now = time.time()
            self.pqr_r = self._uvw_to_pqr_r(uvw_r)
            #t_pqr = time.time() - t_now
            # use blob pairs to get world coordinates in mm (xyz)
            if self.pqr_r.shape[1] == 2:
                #t_now = time.time()
                self.xyz_r = self._pqr_to_xyz(self.pqr_r)
                #t_xyz = time.time() - t_now
            else:
                self.xyz_r = np.zeros(0)
        else:
            self.pqr_r = np.zeros(0)
            self.xyz_r = np.zeros(0)

        if self._blob_l.no_blobs:
            # transform from image coordinates (mn) to coordinates on the unit
            # sphere in the camera frame (uvw)
            uvw_l = self._mn_to_uvw(self._blob_l.blobs)
            # align camera frame with robot frame (uvw to pqr)
            self.pqr_l = self._uvw_to_pqr_l(uvw_l)
            # use blob pairs to get world coordinates in mm (xyz)
            if self.pqr_l.shape[1] == 2:
                self.xyz_l = self._pqr_to_xyz(self.pqr_l)
            else:
                self.xyz_l = np.zeros(0)
        else:
            self.pqr_l = np.zeros(0)
            self.xyz_l = np.zeros(0)

        #return t_capture, t_blob, t_uvw, t_pqr, t_xyz

    def _mn_to_uvw(self, mn):
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

    def _uvw_to_pqr_r(self, uvw):
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

    def _uvw_to_pqr_l(self, uvw):
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

    def _pqr_to_xyz(self, pqr):
        """Uses projective geometry to derive a scale factor from two known objects. Scales normalized pqr rays and returns xyz distances in mm.
        
        Args:
            pqr (float): pqr coordinates of LED PAIRS whose relative positions are known in the real world
        
        Returns:
            float: xyz coordinates in mm to each of the two LEDs
        """
        p1 = pqr[0, 0]
        q1 = pqr[1, 0]
        r1 = pqr[2, 0]
        p2 = pqr[0, 1]
        q2 = pqr[1, 1]
        r2 = pqr[2, 1]

        if r2 < r1:
            ptemp = p1
            qtemp = q1
            rtemp = r1
            p1 = p2
            q1 = q2
            r1 = r2
            p2 = ptemp
            q2 = qtemp
            r2 = rtemp

        delta = U_LED_DZ

        a = r2**2 - r1**2
        b = 2 * delta * r1 * (r2**2 - 1)
        c = (delta**2) * (r2**2 - 1)

        d1_plus = (-b + sqrt(b**2 - 4 * a * c)) / (2 * a)
        d1_minus = (-b - sqrt(b**2 - 4 * a * c)) / (2 * a)

        d1 = d1_plus
        z = d1 * r1
        d2 = (z + delta) / r2
        diff_x = abs(d1 * p1 - d2 * p2)
        diff_y = abs(d1 * q1 - d2 * q2)
        diff_plus = diff_x + diff_y

        d1 = d1_minus
        z = d1 * r1
        d2 = (z + delta) / r2
        diff_x = abs(d1 * p1 - d2 * p2)
        diff_y = abs(d1 * q1 - d2 * q2)
        diff_minus = diff_x + diff_y

        if (diff_plus < diff_minus):
            xyz = d1_plus * pqr
        else:
            xyz = d1_minus * pqr

        return xyz

    def _xyz_cam_to_robot_r(self, xyz):
        """Subtracts the translational offset of the camera center from the robot center.
        
        Args:
            xyz (float): xyz which are offset from the robot center by the camera distance
        
        Returns:
            float: xyz which are in the robot center
        """
        trans = np.array([[U_CAM_DX], [U_CAM_DY], [0]])
        xyz + np.tile(trans, xyz.shape[1])

        return xyz

    def _xyz_cam_to_robot_l(self, xyz):
        """Subtracts the translational offset of the camera center from the robot center.
        
        Args:
            xyz (float): xyz which are offset from the robot center by the camera distance
        
        Returns:
            float: xyz which are in the robot center
        """
        trans = np.array([[U_CAM_DX], [-U_CAM_DY], [0]])
        xyz + np.tile(trans, xyz.shape[1])

        return xyz
