import RPi.GPIO as GPIO
import numpy as np

from math import *
from lib_utils import *
from lib_camera import Camera
from lib_globalblob import GBlob


class Vision():

    def __init__(self):
        self._cam_r = Camera('right', True)
        self._cam_l = Camera('left')
        self._blob_r = GBlob('right', 40) # detection threshold
        self._blob_l = GBlob('left', 40) # detection threshold

        self.pqr_r = np.zeros((3, 1))
        self.pqr_l = np.zeros((3, 1))

    def update(self):
        # check right side
        self._cam_r.capture()
        self._blob_r.detect(self._cam_r.img)

        # check left side
        self._cam_l.capture()
        self._blob_l.detect(self._cam_l.img)
        
        # if blobs detected, transform them to world coordinates in the robot frame
        if self._blob_r.blobs.size:
            # transform from image coordinates to coordinates on the unit sphere in the camera frame
            print(self._blob_r.blobs)
            uvw_r = self._mn_to_uvw(self._blob_r.blobs)
            print(uvw_r)
            # transform coordinates on the unit sphere from camera to robot frame
            self.pqr_r = self._uvw_to_pqr_r(uvw_r)
            print(self.pqr_r)
        else:
            self.pqr_r = self._blob_r.blobs

        if self._blob_l.blobs.size:
            # transform from image coordinates to coordinates on the unit sphere in the camera frame
            uvw_l = self._mn_to_uvw(self._blob_l.blobs)
            # transform coordinates on the unit sphere from camera to robot frame
            self.pqr_l = self._uvw_to_pqr_l(uvw_l)
        else:
            self.pqr_l = self._blob_l.blobs

    def _mn_to_uvw(self, mn):
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
        rot_angle = np.array([[1, 0, 0], [0, cos(U_CAM_ALPHA), -sin(U_CAM_ALPHA)], [0, sin(U_CAM_ALPHA), cos(U_CAM_ALPHA)]])
        rot_axes = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])

        pqr = rot_axes @ rot_angle @ uvw
        pqr_norm = np.linalg.norm(pqr, axis=0)
        pqr = pqr / pqr_norm[None, :]

        return pqr

    def _uvw_to_pqr_l(self, uvw):
        rot_angle = np.array([[1, 0, 0], [0, cos(-U_CAM_ALPHA), -sin(-U_CAM_ALPHA)], [0, sin(-U_CAM_ALPHA), cos(-U_CAM_ALPHA)]])
        rot_axes = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])

        pqr = rot_axes @ rot_angle @ uvw
        pqr_norm = np.linalg.norm(pqr, axis=0)
        pqr = pqr / pqr_norm[None, :]

        return pqr

    def _pqr_to_xyz_r(self, pqr):
        return

    def _pqr_to_xyz_l(self, pqr):
        return

    def _xyz_cam_to_robot_r(self, xyz):
        trans = np.array([[U_CAM_DX], [U_CAM_DY], [0]]) # right
        xyz + np.tile(trans, xyz.shape[1])

        return xyz

    def _xyz_cam_to_robot_l(self, xyz):
        trans = np.array([[U_CAM_DX], [-U_CAM_DY], [0]])
        xyz + np.tile(trans, xyz.shape[1])

        return xyz
