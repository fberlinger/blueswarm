
from lib_blob import Blob
from math import sqrt
from lib_utils import *
import numpy as np

class ImgMatch():
    def __init__(self, no_images, max_blobs, dist_thresh):
        self.no_images = no_images
        self.d_thresh = dist_thresh
        self.blob_r = Blob('right', max_blobs, 40) # detection threshold
        self.blob_l = Blob('left', max_blobs, 40) # detection threshold
        self.blobs = []
        self.outliers = []

    def _blob_detection(self, imgs, side):
        self.blobs = []
        for ii in range(self.no_images):
            if side == 'right':
                self.blob_r.detect(imgs[ii])
                self.blobs.append(self.blob_r.blobs.tolist()) # make python list
            else:
                self.blob_l.detect(imgs[ii])
                self.blobs.append(self.blob_l.blobs.tolist()) # make python list

    def _matrix_outliers(self, blobs_1, blobs_2):
        blobs_x_1 = [item[0] for item in blobs_1]
        blobs_y_1 = [item[1] for item in blobs_1]
        blobs_x_2 = [item[0] for item in blobs_2]
        blobs_y_2 = [item[1] for item in blobs_2]
        
        N1 = len(blobs_x_1)
        N2 = len(blobs_x_2)
        D = np.zeros(N1, N2)

        for ii in range(N1):
            for jj in range(N2):
                D[ii,jj] = sqrt((blobs_x_1[ii]-blobs_x_2[jj])**2 + (blobs_y_1[ii]-blobs_y_2[jj])**2)

        flags_1 = np.ones((1, N1))
        flags_2 = np.ones((1,N2))

        while np.D.min() < self.d_thresh:
            ind = np.unravel_index(np.argmin(D, axis=None), D.shape)
            D[ind[0],:] = self.d_thresh
            D[:,ind[1]] = self.d_thresh
            flags_1[ind[0]] = 0
            flags_2[ind[1]] = 0

        if max(flags_1, flags_2) == 0
            outliers = []
            return outliers

        outliers_x = [blobs_x_1[np.nonzero(flags_1)]] + [blobs_x_2[np.nonzero(flags_2)]]
        outliers_y = [blobs_y_1[np.nonzero(flags_1)]] + [blobs_y_2[np.nonzero(flags_2)]]

        outliers = []

        for ii in range(len(outliers_x)):
            outliers.append([outliers_x[ii], outliers_y[ii]])

        return outliers


    def _outlier_detection(self):
        self.outliers = []
        for ii in range(self.no_images-1):
            # stop if both images have same no_blobs or one has zero blobs
            imgA = self.blobs[ii]
            imgB = self.blobs[ii+1]
            imgA = list(map(list, zip(*imgA))) # transpose
            imgB = list(map(list, zip(*imgB))) # transpose

            '''
            ## Remove blobs at the edges of the image
            scaling_factor = 2592 / U_CAM_NRES
            CIRCLE_THRESH = min(U_CAM_MRES - U_CAM_xc/scaling_factor, U_CAM_xc/scaling_factor) - 5
            
            imgA_copy = []
            for i in range(len(imgA)):
                pt = imgA[i]
                d = sqrt((pt[0] - U_CAM_xc/scaling_factor)**2 + (pt[1] - U_CAM_xc/scaling_factor)**2)
                if d < CIRCLE_THRESH:
                    imgA_copy.append(imgA[i])
            imgA = imgA_copy

            imgB_copy = []
            for i in range(len(imgB)):
                pt = imgB[i]
                d = sqrt((pt[0] - U_CAM_xc/scaling_factor)**2 + (pt[1] - U_CAM_xc/scaling_factor)**2)
                if d < CIRCLE_THRESH:
                    imgB_copy.append(imgB[i])
            imgB = imgB_copy
            #############
            '''

            imgA_len = len(imgA)        
            imgB_len = len(imgB)
            if not imgA_len and not imgB_len:
                self.outliers.append([])
                continue
            elif not imgA_len:
                self.outliers.append(imgB)
                continue
            elif not imgB_len:
                self.outliers.append(imgA)
                continue

            #new_outliers = _matrix_outliers(imgA, imgB)

            # greedily match blobs from 2 images
            new_outliers = []
            for indA, ptA in enumerate(imgA):
                d_min = (1000, 1000)
                for indB, ptB in enumerate(imgB):
                    # Manhattan
                    #dm = abs(ptA[0]-ptB[0])
                    #dn = abs(ptA[1]-ptB[1])
                    #d = dm + dn

                    # Eucledean
                    d = sqrt((ptA[0]-ptB[0])**2 + (ptA[1]-ptB[1])**2)

                    if d < d_min[0]:
                        d_min = (d, indB)

                if d_min[0] < self.d_thresh:
                    del imgB[d_min[1]]
                else:
                    new_outliers.append(ptA)
            if imgB:
                new_outliers += imgB


            self.outliers.append(new_outliers)

    def find_outliers(self, imgs, side):
        self._blob_detection(imgs, side)
        self._outlier_detection()
        return self.outliers
