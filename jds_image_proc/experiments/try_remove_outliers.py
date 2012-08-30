from jds_image_proc import pcl_utils, pcd_io, utils_images
import cPickle
from mlabraw_jds_image_processing import remove_holes
import cv2
import numpy as np
import scipy.ndimage as ndi
xyz, bgr = pcd_io.load_xyzrgb("/home/joschu/comm/pr2_knot/kinect/data000000000100.pcd")
bgr = bgr.copy()
mask = cv2.imread("/home/joschu/comm/pr2_knot/once/roi_mask.png")[:,:,0].copy()
with open("/home/joschu/comm/pr2_knot/human_labels/classifier.pkl","r") as fh: cls = cPickle.load(fh)
labels = cls.predict(bgr, mask=mask)


x,y,z = xyz[:,:,0], xyz[:,:,1], xyz[:,:,2]
depth = np.sqrt(x**2+y**2+z**2)
depth[np.isnan(depth)] = np.inf
bad = np.zeros(depth.shape,bool)
bad[1:] |= np.abs(depth[1:] - depth[:-1]) > .01
bad[:,1:] |= np.abs(depth[:,1:] - depth[:,:-1]) > .1
bad = ndi.binary_dilation(bad, utils_images.disk(2))
cv2.imshow('bad', bad.astype('uint8')*100)
cv2.imshow('labels',labels*100*~bad)
labels1 = labels*~bad
cv2.imshow('cleaned labels',labels1*100*~bad)
