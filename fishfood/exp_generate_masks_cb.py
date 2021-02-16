import numpy as np
from math import *
#import matplotlib.pyplot as plt

MRES = 192
NRES = 256
mc = 9.890661037168528e+02
nc = 1.240887736413883e+03
scaling_factor = 2592/NRES
mcs = mc/scaling_factor
ncs = nc/scaling_factor
rad = MRES/2*0.85


mask_cb = np.zeros((MRES, NRES))

for m in range(int(MRES/4), MRES): #fb top quarter remains black (or bottom, check)
    for n in range(NRES):
        if (m-mcs)**2 + (n-ncs)**2 < rad**2:
            mask_cb[MRES - m, NRES - n] = 1

f = open("mask_cb.txt", "w")
for m in range(MRES):
    for n in range(NRES):
        f.write(str(mask_cb[m, n]) + " ")
    f.write("\n")

#plt.imshow(mask_cb, cmap="gray") 
#plt.show()