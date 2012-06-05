import pcd_io
import numpy as np
import mlabraw_image_processing as mip
import cv2
import scipy.ndimage as ndi
import matplotlib.pyplot as plt

xyz,rgb = pcd_io.load_xyzrgb("/home/joschu/comm/rope_hands/kinect/data000000000000.pcd")
table_corners = np.loadtxt("/home/joschu/comm/rope_hands/once/table_corners.txt")
z_ax = np.cross(table_corners[1] - table_corners[0], table_corners[2]-table_corners[1])
z_ax *= -np.sign(z_ax[2])
height = (xyz * z_ax[None,None,:]).sum(axis=2)

M,N = height.shape

mip.initialize()
mip.put("height",height)
mip.evaluate("addpath('~/python/image_proc')")
mip.evaluate("height1 = inpaint_nans(height);")
height1 = mip.get("height1")

gradmag = ndi.gaussian_gradient_magnitude(height1,2)

labels = cv2.imread('/home/joschu/comm/rope_hands/labels/data000000000000.png')[:,:,0]
rope = (labels==1).copy()
vmin = height1[rope].min()
vmax = height1[rope].max()

plt.ion()
ridges = (np.log(gradmag)<-8)&rope&np.isfinite(height)
ridgeskel = mip.skeletonize(ridges)
plt.imshow(ridgeskel)
