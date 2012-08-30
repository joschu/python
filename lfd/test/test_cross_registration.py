from __future__ import division
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--downsample", action="store_true")
args = parser.parse_args()


import sys
import lfd
from lfd import registration, recognition
import h5py
import numpy as np
import os.path as osp
from jds_image_proc.clouds import voxel_downsample, pixel_downsample
import rospy, itertools, glob
from jds_utils.colorize import colorize
import h5py
import matplotlib.pyplot as plt

h5file = h5py.File(osp.join(osp.dirname(lfd.__file__), "data/knot_segments.h5"),"r")
seg_names = h5file.keys()


results = []

dists = []
ropes = []
for i in xrange(6):
    rope = np.squeeze(h5file[seg_names[i]]["cloud_xyz"])
    if args.downsample:
        rope_ds,ds_inds = voxel_downsample(rope,.03,return_inds = True)
        dist = recognition.calc_geodesic_distances_downsampled(rope, rope_ds, ds_inds)
        dists.append(dist)
        ropes.append(rope_ds)
    else:
        dist = recognition.calc_geodesic_distances(rope)
        dists.append(dist)
        ropes.append(rope)
                                                                    
for i0 in xrange(6):
    rope0 = ropes[i0]
    n0 = len(rope0)
    #pairs = zip(xrange(n0), np.random.randint(0,n0,n0))
    dists0 = dists[i0]

    for i1 in xrange(6):
        rope1 = ropes[i1]

        dists1 = dists[i1]
        
        n1 = len(rope1)
        
        
        print colorize("comparing %s to %s"%(seg_names[i0], seg_names[i1]), "red")
        cost = recognition.calc_match_score(rope0, rope1, dists0, dists1)
            
        results.append((i0, i1, cost))
        print i0, i1, cost
        
distmat = np.zeros((6,6))
for (i0, i1, cost) in results:
    distmat[i0, i1] = cost
distmat[xrange(6),xrange(6)] = np.nan
print distmat.argmin(axis=0)

a,b = np.meshgrid([0,3,1,4,2,5],[0,3,1,4,2,5])
distmat_rearr = distmat[a,b]
distmat_rearr[range(6), range(6)] = np.nan
plt.imshow(distmat_rearr,interpolation='nearest',cmap='gray')
plt.show()