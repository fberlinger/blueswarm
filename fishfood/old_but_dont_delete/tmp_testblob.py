"""Blob library, a component of vision library. Detects LED pixels in images and returns centroids of individual LEDs.
"""

from lib_utils import *
import numpy as np


class Blob():

    """Blob takes in a camera image and returns the pixel coordinates (mn) of individual LED blobs.
    
    Blob contains functions to convert an image to grayscale, threshold the image to separate blob pixels from background, assign blob pixels to individual blobs, and discard blobs that are reflected at the water surface.
    
    Attributes:
        blob_size (int): Total amount of LED blob pixels
        blobs (float): Array of blob centroids, (2, no_blobs)
        max_blobs (int): Amount of blobs expected in image. Additional blobs will be considered reflections and discarded.
        no_blobs (int): Number of clustered LED blobs
        side (string): Camera side, right or left
        thresh (int, optional): Light intensity for pixel to be considered LED blob pixel, [0=LOW,255=HIGH]
    """

    def __init__(self, side, max_blobs, thresh=U_BLOB_THRESH):
        """One Blob object is instantiated for each side, i.e., the right and the left side.
        
        Args:
            side (string): Camera side, right or left
            max_blobs (int): Amount of blobs expected in image. Additional blobs will be considered reflections and discarded.
            thresh (int, optional): Light intensity for pixel to be considered LED blob pixel, [0=LOW,255=HIGH]
        """

        # Arguments
        self.side = side
        self.max_blobs = max_blobs 
        self.thresh = thresh

        # Initializations
        self.blob_size = 0
        self.blobs = np.zeros((2, 1))
        self.no_blobs = 0

    def detect(self, img):
        """Detect takes in an image and stores LED blob centroids in self.blobs.
        
        Args:
            img (int): Image array from lib_camera, (U_CAM_MRES, U_CAM_NRES, 3)
        """

        # Initializations
        self.blob_size = 0
        self.blobs = np.zeros((2, 1))
        self.no_blobs = 0

        # Run all subfunctions for blob detection
        img_gray = self._raw_to_gray(img)
        blob_pixels = self._thresholding(img_gray)
        self._continuity(blob_pixels)
        self.reflections()

    def _raw_to_gray(self, img):
        """Convert the rgb image to grayscale.
        
        Args:
            img (int): Raw rgb image array, (U_CAM_MRES, U_CAM_NRES, 3)
        
        Returns:
            int: Grayscale image array, (U_CAM_MRES, U_CAM_NRES)
        """
        img_rgb = np.zeros((U_CAM_MRES, U_CAM_NRES, 3), dtype=np.uint8)
        img_rgb = np.array(img)
        img_gray = np.zeros((U_CAM_MRES, U_CAM_NRES))
        img_gray[:, :] = img_rgb[:, :, 2]

        return img_gray

    def _thresholding(self, img_gray):
        """Keeps pixels with high enough light intensity to be considered LED blob pixels only.
        
        Args:
            img_gray (int): Grayscale image array, (U_CAM_MRES, U_CAM_NRES)
        
        Returns:
            int: Array of blob pixels
        """
        blob_pixels = np.where(img_gray > self.thresh)
        blob_pixels = np.asarray(blob_pixels)

        return blob_pixels

    def _continuity(self, blob_pixels):
        """Clusters blob pixels and returns lists of individual blob centroids
        
        _continuity checks all blob pixels for continuity in m-direction. It then checks the subsets which are continous in m-direction for continuity in n-direction. It finally returns an array that contains the centroids of individual blobs.
        
        Args:
            blob_pixels (int): Array of blob pixels
        
        Returns:
            float: Array of blob centroids including reflections, (2, no_blobs)
        
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
                #m_center = U_CAM_MRES - m_center #xx CHANGED
                #n_center = U_CAM_NRES - n_center #xx CHANGED

                if self.no_blobs == 0:
                    self.blobs[0, 0] = m_center
                    self.blobs[1, 0] = n_center

                else:
                    self.blobs = np.append(self.blobs, [[m_center], [n_center]], axis=1)
        
                self.no_blobs += 1

    def reflections(self):
        """Discards LED blob centroids that are considered reflections at the water surface. Reflections tend to appear higher up in the image than real centroids, i.e., they have lower m-coordinates. If the number of identified blobs is greater than the maximum number of expected blobs, the maximum number of expected blobs with the highest m-coodinates will be kept.
        """

        if self.no_blobs > self.max_blobs:
            blob_ind = np.argsort(self.blobs[0, :])[-self.max_blobs:]
            self.blobs = self.blobs[:, blob_ind]
