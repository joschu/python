import pcd_io
import numpy as np
import mlabraw_jds_image_processing as mip
import cv2
import scipy.ndimage as ndi

xyz,rgb = pcd_io.load_xyzrgb("/home/joschu/comm/rope_hands/kinect/data000000000000.pcd")
table_corners = np.loadtxt("/home/joschu/comm/rope_hands/once/table_corners.txt")
z_ax = np.cross(table_corners[1] - table_corners[0], table_corners[2]-table_corners[1])
z_ax *= -np.sign(z_ax[2])
height = (xyz * z_ax[None,None,:]).sum(axis=2)

M,N = height.shape

#mip.initialize()
#mip.put(height,"height")
#mip.eval("addpath('~/python/jds_image_proc')")
#mip.eval("height1 = inpaint_nans(height);")
#height1 = mip.get("height1")
labels = cv2.imread('/home/joschu/comm/rope_hands/labels/data000000000000.png')[:,:,0]
rope = (labels==1).copy()

height1 = ndi.distance_transform_cdt(rope,return_distances=True,return_indices=False).astype('float32')


d0 = ndi.gaussian_filter1d(height1,sigma=3,axis=0,order=1)
d1 = ndi.gaussian_filter1d(height1,sigma=3,axis=1,order=1)
d00 = ndi.gaussian_filter1d(d0,sigma=3,axis=0,order=1)
d01 = ndi.gaussian_filter1d(d0,sigma=3,axis=1,order=1)
d11 = ndi.gaussian_filter1d(d1,sigma=3,axis=1,order=1)

D1 = np.empty((M,N,2))
D1[:,:,0] = d0
D1[:,:,1] = d1

D2 = np.empty((M,N,2,2))
D2[:,:,0,0] = d00
D2[:,:,0,1] = d01
D2[:,:,1,0] = d01
D2[:,:,1,1] = d11

smallEigVals = np.zeros((M,N))
smallEigVecs = np.zeros((M,N,2))
for m in xrange(M):
    for n in xrange(N):
        if rope[m,n]:
            vals,vecs = np.linalg.eigh(D2[m,n])        
            smallEigVals[m,n] = vals[0]
            smallEigVecs[m,n] = vecs[:,0]

p = (smallEigVecs * D1).sum(axis=2)
zero_crossings = ((p[:-1,:-1]*p[:-1,1:])<0) | ((p[:-1,:-1]*p[1:,:-1])<0)

ridges = zero_crossings & (smallEigVals[:-1,:-1]<0)

hmin = height1[rope].min()
hmax = height1[rope].max()


import matplotlib.pyplot as plt
plt.ion()

plt.imshow(ridges)
