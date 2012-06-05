import cv2
import scipy.ndimage as ndi
import numpy as np

def disk(r):
    xs,ys = np.mgrid[-r:r+1,-r:r+1]
    return xs**2 + ys**2 <= r**2
    
def confusionMat(labels0, labels1):
    maxLabel = max(labels0.max(), labels1.max())
    bins = np.arange(-.5,maxLabel+1)
    return np.histogram2d(labels0.flatten(), labels1.flatten(), bins)[0].astype('int')
    
def get_cc_centers(mask,n,min_pix=2):
    labels, max_label = ndi.label(mask)

    if max_label < n: raise Exception("could not find %i regions"%n)
    
    counts = np.bincount(labels.flatten())
    good_labels = np.flatnonzero(counts >= min_pix)[1:]
    good_labels_sorted = good_labels[counts[good_labels].argsort()[::-1]]
    good_labels_sorted = good_labels_sorted[:n]
    print good_labels_sorted
    
    out = np.array(ndi.center_of_mass(mask,labels,good_labels_sorted))
    print out
    return out