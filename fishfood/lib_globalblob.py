import RPi.GPIO as GPIO

from lib_utils import *
import numpy as np


class GBlob():

    """Blob detection. Returns coordinates of all blobs

    This class takes a camera image and returns the pixel coordinates of all blobs.
    It contains functions to convert the image to grayscale, threshold the image to
    separate blob pixels from background, create lists of indices of pixels which
    belong to single blobs, and calculate the center of each blob.
    """

    def __init__(self, side, thresh=U_BLOB_THRESH):
        """Load a new image
        
        Arguments:
            img_raw {} -- camera image

        Keyword Arguments:
            thresh {int} -- detection threshold that separates background from blob pixels
            x_res {int} -- x-resolution of the image
            y_res {int} -- y-resolution of the image
        """

        # Parameters
        self.side = side
        self.thresh = thresh

        # Initializations
        self.blob_size = 0
        self.blobs = np.zeros((2, 1))
        self.no_blobs = 0

    def detect(self, img):
        # Initializations
        self.blob_size = 0
        self.blobs = np.zeros((2, 1))
        self.no_blobs = 0

        """Runs all subfunctions for blob detection"""
        img_gray = self._raw_to_gray(img)
        blob_pixels = self._thresholding(img_gray)
        self._continuity(blob_pixels)

    def _raw_to_gray(self, img):
        """Converts the image to grayscale"""
        img_rgb = np.zeros((U_CAM_MRES, U_CAM_NRES, 3), dtype=np.uint8)
        img_rgb = np.array(img)
        img_gray = np.zeros((U_CAM_MRES, U_CAM_NRES))
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

        # Find pixels that are continuous in m-direction
        m = blob_pixels[0, :]
        m_shifted = np.zeros(m.shape)
        m_shifted[1:-1] = np.copy(m[:-2])
        m_shifted[0] = -1
        m_shifted[-1] = -1

        blob_m = np.where(abs(m_shifted - m) > 1)
        blob_m = np.asarray(blob_m)
        blob_m[:, -1] += 1

        # For each continous set in m-direction, find pixels that are also continuous in n-direction
        for i in range(0, blob_m.shape[1]-1):
            m = blob_pixels[0, blob_m[0, i]:blob_m[0, i+1]]
            n = blob_pixels[1, blob_m[0, i]:blob_m[0, i+1]]
            arg_n = np.argsort(n)
            n_sorted = np.sort(n)
            
            n_shifted = np.zeros(n.shape)
            n_shifted[1:-1] = np.copy(n_sorted[:-2])
            n_shifted[0] = -1
            n_shifted[-1] = -1

            blob_n = np.where(abs(n_shifted - n_sorted) > 1)
            blob_n = np.asarray(blob_n)
            blob_n[:, -1] += 1
            
            # For pixels continuous in m- and n-direction, find centroids
            for j in range(0, blob_n.shape[1]-1):
                blob_indices = arg_n[np.asscalar(blob_n[:, j]):np.asscalar(blob_n[:, j+1])]
                m_center = round(sum(m[blob_indices])/blob_indices.shape[0], 3)
                n_center = round(sum(n[blob_indices])/blob_indices.shape[0], 3)

                # flip image 180 degrees bcs camera mounted upside down
                m_center = U_CAM_MRES - m_center
                n_center = U_CAM_NRES - n_center

                if self.no_blobs == 0:
                    self.blobs[0, 0] = m_center
                    self.blobs[1, 0] = n_center

                else:
                    self.blobs = np.append(self.blobs, [[m_center], [n_center]], axis=1)
        
                self.no_blobs += 1

    def reflections(self):
        # discard blobs that are reflected on the surface, keep single lowest blob only
        if self.blobs.size:
            blob_ind = np.where(self.blobs == min(self.blobs[:, 1]))
            self.blobs = self.blobs[blob_ind[0], :]
