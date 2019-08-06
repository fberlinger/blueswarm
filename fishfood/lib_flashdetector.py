
from math import sqrt
import numpy as np

class FlashDetector:
    def __init__(self, dist_thresh):
        self.thresh = dist_thresh
        self.clusters = []
        self.sizes = []

    def distance_between(self, p1, p2):
        # return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) # Manhattan distance
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) # Euclidean distance

    def update(self, discrepancy_list):
        if not discrepancy_list:
            return

        for e in discrepancy_list:
            if not e:
                return

        if not self.clusters:
            self.clusters.append(discrepancy_list[0])
            self.sizes.append(1)
            discrepancy_list.pop(0)

        for ii in range(len(discrepancy_list)):
            match_found = False
            for jj in range(len(self.clusters)):
                d = self.distance_between(discrepancy_list[ii], self.clusters[jj])
                if (d < self.thresh):
                    self.clusters[jj] = discrepancy_list[ii] # Replace representative of cluster by latest point
                    self.sizes[jj] = self.sizes[jj] + 1 # Increment number of observations in that cluster
                    match_found = True
                    break

            if not match_found:
                self.clusters.append(discrepancy_list[ii])
                self.sizes.append(1)

    def find_max_flashes(self, data_perm):
        self.clusters = []
        self.sizes = []

        data = data_perm

        for kk in range(len(data)):
            self.update(data[kk])
        
        if not self.sizes:
            return (0, 0)
        
        ind = self.sizes.index(max(self.sizes))
        mn = np.zeros((2,1))
        mn[0] = self.clusters[ind][0]
        mn[1] = self.clusters[ind][1]

        return (max(self.sizes), mn)
