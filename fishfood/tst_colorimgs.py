
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

def _continuity(blob_pixels, rec_level=0):
    """Clusters blob pixels and returns lists of individual blob centroids
    
    _continuity checks all blob pixels for continuity in m-direction. It then checks the subsets which are continous in m-direction for continuity in n-direction. It finally returns an array that contains the centroids of individual blobs.
    
    Args:
        blob_pixels (int): Array of blob pixels
    
    Returns:
        float: Array of blob centroids including reflections, (2, no_blobs)
    
    """
    if not blob_pixels.size:
        return

    cont_pix=1
    rec_depth=0

    no_blobs = 0
    blobs = []
    self_no_pixels = []

    # Find pixels that are continuous in m-direction
    m = blob_pixels[0, :]
    breaks_m = np.asarray(np.where(np.diff(m) > cont_pix))
    breaks_m += 1
    breaks_m = np.insert(breaks_m, 0, 0)
    breaks_m = np.append(breaks_m, len(m))

    # For each continous set in m-direction, find pixels that are also continuous in n-direction
    for i in range(0, len(breaks_m)-1):
        m = blob_pixels[0, breaks_m[i]:breaks_m[i+1]]
        n = blob_pixels[1, breaks_m[i]:breaks_m[i+1]]
        arg_n = np.argsort(n)
        n_sorted = np.sort(n)
        
        breaks_n = np.asarray(np.where(np.diff(n_sorted) > cont_pix))
        breaks_n += 1
        breaks_n = np.insert(breaks_n, 0, 0)
        breaks_n = np.append(breaks_n, len(n_sorted))            

        # For pixels continuous in m- and n-direction, find centroids
        for j in range(0, len(breaks_n)-1):
            blob_indices = arg_n[np.asscalar(breaks_n[j]):np.asscalar(breaks_n[j+1])]

            # run continuity test for each subcluster if recursion depth has not been reached
            if len(breaks_n) > 2 and rec_level < rec_depth:
                # m has to be sorted to avoid "artificial" gaps
                # n have to be listed in corresponding order
                order = np.argsort(m[blob_indices])
                new_blob_pix = np.array([np.sort(m[blob_indices]), n[blob_indices][order]])

                _continuity(new_blob_pix, rec_level+1)

            # no more splits? return centroid!
            else:
                # store number of pixels in that blob
                no_pixels = blob_indices.size
                self_no_pixels.append(no_pixels)
                # get centroid
                m_center = round(sum(m[blob_indices])/no_pixels, 1)
                n_center = round(sum(n[blob_indices])/no_pixels, 1)
                # flip image 180 degrees bcs camera mounted upside down
                
                #m_center = 192 - m_center - 1 #xx
                #n_center = 256 - n_center - 1 #xx

                if no_blobs == 0:
                    blobs = np.zeros((2, 1))
                    blobs[0, 0] = m_center
                    blobs[1, 0] = n_center
                else:
                    blobs = np.append(blobs, [[m_center], [n_center]], axis=1)
                
                no_blobs += 1

    return (no_blobs, blobs)

# test image
image = Image.open('test.png')
img = np.asarray(image)
img_dark = 255*np.ones((192, 256))
mask_cb = np.loadtxt("mask_cb.txt", dtype=np.int32)

# mask
img_cb = np.multiply(img_dark-img[:, :, 2], mask_cb)
plt.imshow(img_cb, cmap="gray")
plt.show()

# blobs
blob_pixels = np.where(img_cb > 50) # was 50
blob_pixels = np.asarray(blob_pixels)
no_blobs, blobs = _continuity(blob_pixels)
print(no_blobs, blob_pixels.size)

# illustration
plt.figure()
plt.imshow(img_cb, cmap="gray")
plt.scatter(blobs[1], blobs[0], s=50, c='red', marker='o')
plt.show()


