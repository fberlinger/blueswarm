
from lib_blob import Blob

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

    def _outlier_detection(self):
        self.outliers = []
        for ii in range(self.no_images-1):
            # stop if both images have same no_blobs or one has zero blobs
            imgA = self.blobs[ii]
            imgB = self.blobs[ii+1]
            imgA = list(map(list, zip(*imgA))) # transpose
            imgB = list(map(list, zip(*imgB))) # transpose

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

            # greedily match blobs from 2 images
            new_outliers = []
            for indA, ptA in enumerate(imgA):
                d_min = (1000, 1000)
                for indB, ptB in enumerate(imgB):
                    dm = abs(ptA[0]-ptB[0])
                    dn = abs(ptA[1]-ptB[1])
                    d = dm + dn
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
