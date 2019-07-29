"""Blob library, a component of vision library. Detects LED pixels in images and returns centroids of individual LEDs.
"""
#import RPi.GPIO as GPIO
#from lib_utils import *
import numpy as np

import time
from PIL import Image
U_BLOB_THRESH = 40
U_CAM_NRES = 128
U_CAM_MRES = 96

import sys
np.set_printoptions(threshold=sys.maxsize)
#sys.setrecursionlimit(10000000)


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

        t_dfs_raw = time.time()
        self._DFS_raw(img_gray)
        dur_dfs_raw = time.time() - t_dfs_raw
        #print('dur_dfs_raw {:.6f}s'.format(dur_dfs_raw))

        t_thresh = time.time()
        blob_pixels = self._thresholding(img_gray)
        dur_thresh = time.time() - t_thresh
        #print('dur_thresh {:.6f}s'.format(dur_thresh))

        t_dfs = time.time()
        self._DFS(blob_pixels)
        dur_dfs = time.time() - t_dfs
        #print('dur_dfs {:.6f}s'.format(dur_dfs+dur_thresh))


        # Initializations
        self.blob_size = 0
        self.blobs = np.zeros((2, 1))
        self.no_blobs = 0

        t_cont = time.time()
        self._continuity(blob_pixels)
        dur_cont = time.time() - t_cont
        #print('dur_cont {:.6f}s'.format(dur_cont+dur_thresh))


        if self.max_blobs:
            self.reflections()

        return (dur_dfs_raw, dur_dfs, dur_cont)

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

    def _DFS(self, blob_pixels):
        # Total amount of blob pixels. If none, return.
        self.blob_size = blob_pixels.size
        if self.blob_size < 4:
            self.blobs = np.zeros(0)
            return

        # Create mn-dictionaries
        d_pix = {}
        for ii in range(int(self.blob_size/2)):
            m = blob_pixels[0,ii]
            n = blob_pixels[1,ii]
            if not m in d_pix:
                d_pix[m] = {n: 0}
            else:
                d_pix[m][n] = 0

        # Do DFS on blob_pixels using d_pix as the data structure
        def search(m_key, n_key):
            d_pix[m_key][n_key] = 1 # explored = 1
            scc.append((m_key, n_key))
            center[0] += m_key
            center[1] += n_key

            adjacency = [(m_key+1, n_key), (m_key-1, n_key), (m_key, n_key+1), (m_key, n_key-1)]
            for pix in adjacency:
                try:
                    if not d_pix[pix[0]][pix[1]]:
                        search(pix[0], pix[1])
                except:
                    pass

        blob_sccs = []
        blob_center = []
        # DFS
        for m_key,n_val in d_pix.items():
            for n_key,vis_val in n_val.items():
                if not vis_val:
                    scc = []
                    center = [0, 0]
                    search(m_key, n_key)
                    blob_sccs.append(scc)
                    no_b = len(scc)
                    c_m = center[0] / no_b
                    c_n = center[1] / no_b
                    blob_center.append((c_m, c_n))

        #for jj in range(len(blob_sccs)):
        #    print(blob_center[jj])
        #print('\n')

    def _DFS_raw(self, img_gray):
        # visited array with boarder pixels initialized to visited to avoid index errors
        vis = np.zeros((U_CAM_MRES, U_CAM_NRES))
        vis[:,0] = 1
        vis[:,-1] = 1
        vis[0,:] = 1
        vis[-1,:] = 1


        def search(m_key, n_key):
            vis[m_key,n_key] = 1 # explored = True
            scc.append((m_key, n_key))
            center[0] += m_key
            center[1] += n_key

            adjacency = [(m_key+1, n_key), (m_key-1, n_key), (m_key, n_key+1), (m_key, n_key-1)]
            for pix in adjacency:
                if not vis[pix[0],pix[1]]:
                    if img_gray[pix[0],pix[1]] > self.thresh:
                        search(pix[0], pix[1])
                    else:
                        vis[pix[0],pix[1]] = 1

        blob_sccs = []
        blob_center = []
        # DFS
        for ii in range(1, U_CAM_MRES-1):
            for jj in range(1, U_CAM_NRES-1):
                if not vis[ii,jj]:
                    if img_gray[ii,jj] > self.thresh:
                        scc = []
                        center = [0, 0]
                        search(ii, jj)
                        blob_sccs.append(scc)
                        no_b = len(scc)
                        c_m = center[0] / no_b
                        c_n = center[1] / no_b
                        blob_center.append((c_m, c_n))
                    else:
                        vis[ii,jj] = 1

        #for jj in range(len(blob_sccs)):
        #    print(blob_center[jj])
        #print('\n')


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

                #print(m_center, n_center)
                
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
        """Discards LED blob centroids that are considered reflections at the water surface. Reflections tend to appear higher up in the image than real centroids, i.e., they have lower m-coordinates. If the number of identified blobs is greater than the maximum number of expected blobs, the maximum number of expected blobs with the highest m-coodinates will be kept.
        """

        if self.no_blobs > self.max_blobs:
            blob_ind = np.argsort(self.blobs[0, :])[-self.max_blobs:]
            self.blobs = self.blobs[:, blob_ind]




if __name__ == "__main__":
    blob_dfs = Blob('right', 0)

    img = Image.open("4crosses_128_96.jpg")
    #img = Image.open("cross_128_96.jpg")
    img.load()
    img = np.asarray(img, dtype=np.uint32)

    dfs_raw = 0
    dfs = 0
    cont = 0
    for i in range(100):
        dur_dfs_raw, dur_dfs, dur_cont = blob_dfs.detect(img)
        dfs_raw += dur_dfs_raw
        dfs += dur_dfs
        cont += dur_cont
    dfs_raw /= 100
    dfs /= 100
    cont /= 100

    print(dfs_raw, dfs, cont)


    img = Image.open("cross_128_96.jpg")
    img.load()
    img = np.asarray(img, dtype=np.uint32)

    dfs_raw = 0
    dfs = 0
    cont = 0
    for i in range(100):
        dur_dfs_raw, dur_dfs, dur_cont = blob_dfs.detect(img)
        dfs_raw += dur_dfs_raw
        dfs += dur_dfs
        cont += dur_cont
    dfs_raw /= 100
    dfs /= 100
    cont /= 100

    print(dfs_raw, dfs, cont)


    img = Image.open("45dottedline_128_96.jpg")
    img.load()
    img = np.asarray(img, dtype=np.uint32)

    dfs_raw = 0
    dfs = 0
    cont = 0
    for i in range(100):
        dur_dfs_raw, dur_dfs, dur_cont = blob_dfs.detect(img)
        dfs_raw += dur_dfs_raw
        dfs += dur_dfs
        cont += dur_cont
    dfs_raw /= 100
    dfs /= 100
    cont /= 100

    print(dfs_raw, dfs, cont)


