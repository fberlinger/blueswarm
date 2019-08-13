
from math import sqrt
import numpy as np
import copy

class FlashDetector:
    def __init__(self, dist_thresh):
        self.thresh = dist_thresh
        self.streaks = []
        self.sizes = []

    def distance_between(self, p1, p2):
        # return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) # Manhattan distance
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) # Euclidean distance

    def update(self, outliers):
        if not outliers:
            return

        if not self.streaks:
            for ii in range(len(outliers)):
                self.streaks.append(outliers[ii])
                self.sizes.append(1)
            return

        N1 = len(self.streaks)
        N2 = len(outliers)
        D = np.zeros((N1, N2))

        for ii in range(N1):
            for jj in range(N2):
                D[ii,jj] = sqrt((self.streaks[ii][0]-outliers[jj][0])**2 + (self.streaks[ii][1]-outliers[jj][1])**2)

        flags_2 = np.ones(N2)

        while np.min(D) < self.thresh:
            ind = np.unravel_index(np.argmin(D, axis=None), D.shape)
            self.streaks[ind[0]] = outliers[ind[1]]
            self.sizes[ind[0]] += 1

            D[ind[0],:] = self.thresh + 1 # add 1 to avoid float comparison errors
            D[:,ind[1]] = self.thresh + 1 # add 1 to avoid float comparison errors
            flags_2[ind[1]] = 0

        for kk in range(N2):
            if flags_2[kk] == 1:
                self.streaks.append(outliers[kk])
                self.sizes.append(1)

    def find_max_flashes(self, data_perm):
        self.streaks = []
        self.sizes = []

        data = copy.deepcopy(data_perm)
        
        for kk in range(len(data)):
            self.update(data[kk])
        
        if not self.sizes:
            return (0, 0)
        
        ind = self.sizes.index(max(self.sizes))
        mn = np.zeros((2,1))
        mn[0] = self.streaks[ind][0]
        mn[1] = self.streaks[ind][1]

        return (max(self.sizes), mn)
