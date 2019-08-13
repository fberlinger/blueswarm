
from lib_blob import Blob
from math import sqrt
from lib_utils import *
import numpy as np

class ImgMatch():
    def __init__(self, no_images, max_blobs, dist_thresh, light_sens, cont_pix, rec_depth=0, nhood_size=5, soft_thresh=5):
        self.no_images = no_images
        self.d_thresh = dist_thresh
        self.cont_pix = cont_pix
        self.rec_depth = rec_depth
        self.nhood_size = nhood_size
        self.soft_thresh = soft_thresh
        self.blob_r = Blob('right', max_blobs, light_sens)
        self.blob_l = Blob('left', max_blobs, light_sens)
        self.blobs = []
        self.outliers = []

    def _blob_detection(self, imgs, side):
        self.blobs = []
        for ii in range(self.no_images):
            if side == 'right':
                self.blob_r.detect(imgs[ii], self.cont_pix, self.rec_depth)
                # super hacky ugly, what a disgrace!
                try:
                    new_blobs = self.blob_r.blobs.tolist() # make python list
                except:
                    new_blobs = self.blob_r.blobs # is list already
                self.blobs.append(new_blobs)
            else:
                self.blob_l.detect(imgs[ii], self.cont_pix, self.rec_depth)
                try:
                    new_blobs = self.blob_l.blobs.tolist() # make python list
                except:
                    new_blobs = self.blob_l.blobs # is list already
                self.blobs.append(new_blobs)

    def _probe_neighborhood(self, img, m, n):
        # flip image back 180 degrees
        m_center = U_CAM_MRES - int(round(m))
        n_center = U_CAM_NRES - int(round(n))
        # get max value in neighborhood around m,n
        m_low = max(0, m_center-self.nhood_size)
        m_high = min(U_CAM_MRES, m_center+self.nhood_size+1)
        n_low = max(0, n_center-self.nhood_size)
        n_high = min(U_CAM_NRES, n_center+self.nhood_size+1)

        max_intensity = np.max(img[m_low:m_high, n_low:n_high])
        
        return max_intensity
    
    def _allA_to_allB_outliers(self, blobs_1, blobs_2):
        blobs_x_1 = [item[0] for item in blobs_1]
        blobs_y_1 = [item[1] for item in blobs_1]
        blobs_x_2 = [item[0] for item in blobs_2]
        blobs_y_2 = [item[1] for item in blobs_2]
        
        N1 = len(blobs_x_1)
        N2 = len(blobs_x_2)
        D = np.zeros((N1, N2))

        for ii in range(N1):
            for jj in range(N2):
                D[ii,jj] = sqrt((blobs_x_1[ii]-blobs_x_2[jj])**2 + (blobs_y_1[ii]-blobs_y_2[jj])**2)

        flags_1 = np.ones(N1)
        flags_2 = np.ones(N2)

        while np.min(D) < self.d_thresh:
            ind = np.unravel_index(np.argmin(D, axis=None), D.shape)
            D[ind[0],:] = self.d_thresh
            D[:,ind[1]] = self.d_thresh
            flags_1[ind[0]] = 0
            flags_2[ind[1]] = 0

        if max(flags_1) == 0 and max(flags_2) == 0:
            new_outliers = []
            return new_outliers

        ind_f1 = np.asarray(np.nonzero(flags_1)).tolist()[0]
        ind_f2 = np.asarray(np.nonzero(flags_2)).tolist()[0]

        new_outliers = []
        for outlier in ind_f1:
            new_outliers.append([blobs_x_1[outlier], blobs_y_1[outlier]])
        for outlier in ind_f2:
            new_outliers.append([blobs_x_2[outlier], blobs_y_2[outlier]])

        return new_outliers

    def _allA_to_allB_outliers(self, blobsA, blobsB, imgA, imgB):
        blobs_x_1 = [item[0] for item in blobsA]
        blobs_y_1 = [item[1] for item in blobsA]
        blobs_x_2 = [item[0] for item in blobsB]
        blobs_y_2 = [item[1] for item in blobsB]
        
        N1 = len(blobs_x_1)
        N2 = len(blobs_x_2)
        D = np.zeros((N1, N2))

        for ii in range(N1):
            for jj in range(N2):
                D[ii,jj] = sqrt((blobs_x_1[ii]-blobs_x_2[jj])**2 + (blobs_y_1[ii]-blobs_y_2[jj])**2)

        flags_1 = np.ones(N1)
        flags_2 = np.ones(N2)

        while np.min(D) < self.d_thresh:
            ind = np.unravel_index(np.argmin(D, axis=None), D.shape)
            D[ind[0],:] = self.d_thresh + 1 # add 1 to avoid float comparison errors
            D[:,ind[1]] = self.d_thresh + 1 # add 1 to avoid float comparison errors
            flags_1[ind[0]] = 0
            flags_2[ind[1]] = 0

        if max(flags_1) == 0 and max(flags_2) == 0:
            new_outliers = []
            return new_outliers

        ind_f1 = np.asarray(np.nonzero(flags_1)).tolist()[0]
        ind_f2 = np.asarray(np.nonzero(flags_2)).tolist()[0]

        new_outliers = []
        for outlier in ind_f1:
            if self._probe_neighborhood(imgB, blobs_x_1[outlier], blobs_y_1[outlier]) < self.soft_thresh:
                new_outliers.append([blobs_x_1[outlier], blobs_y_1[outlier]])
        for outlier in ind_f2:
            if self._probe_neighborhood(imgA, blobs_x_2[outlier], blobs_y_2[outlier]) < self.soft_thresh:
                new_outliers.append([blobs_x_2[outlier], blobs_y_2[outlier]])

        return new_outliers

    def _remove_img_edge(self, imgA, imgB):
        scaling_factor = 2592 / U_CAM_NRES
        CIRCLE_THRESH = min(U_CAM_MRES - U_CAM_xc/scaling_factor, U_CAM_xc/scaling_factor) - 5
        
        imgA_copy = []
        for i in range(len(imgA)):
            pt = imgA[i]
            d = sqrt((pt[0] - U_CAM_xc/scaling_factor)**2 + (pt[1] - U_CAM_xc/scaling_factor)**2)
            if d < CIRCLE_THRESH:
                imgA_copy.append(imgA[i])

        imgB_copy = []
        for i in range(len(imgB)):
            pt = imgB[i]
            d = sqrt((pt[0] - U_CAM_xc/scaling_factor)**2 + (pt[1] - U_CAM_xc/scaling_factor)**2)
            if d < CIRCLE_THRESH:
                imgB_copy.append(imgB[i])

        return (imgA_copy, imgB_copy)

    def _outlier_detection(self, imgs):
        self.outliers = []

        for ii in range(self.no_images-1):
            # stop if both images have same no_blobs or one has zero blobs
            imgA = imgs[ii][:, :, 2]
            imgB = imgs[ii+1][:, :, 2]
            blobsA = self.blobs[ii]
            blobsB = self.blobs[ii+1]
            blobsA = list(map(list, zip(*blobsA))) # transpose
            blobsB = list(map(list, zip(*blobsB))) # transpose

            #blobsA, blobsB = self._remove_img_edge(blobsA, blobsB)

            blobsA_len = len(blobsA)        
            blobsB_len = len(blobsB)
            if not blobsA_len and not blobsB_len:
                self.outliers.append([])
                continue
            elif not blobsA_len:
                new_outliers = []
                for jj in range(blobsB_len):
                    if self._probe_neighborhood(imgA, blobsB[jj][0], blobsB[jj][1]) < self.soft_thresh:
                        new_outliers.append(blobsB[jj])
                self.outliers.append(new_outliers)
                continue
            elif not blobsB_len:
                new_outliers = []
                for jj in range(blobsA_len):
                    if self._probe_neighborhood(imgB, blobsA[jj][0], blobsA[jj][1]) < self.soft_thresh:
                        new_outliers.append(blobsA[jj])
                self.outliers.append(new_outliers)
                continue

            new_outliers = self._allA_to_allB_outliers(blobsA, blobsB, imgA, imgB)
            
            self.outliers.append(new_outliers)


    def find_outliers(self, imgs, side):
        self._blob_detection(imgs, side)
        self._outlier_detection(imgs)
        return self.outliers
