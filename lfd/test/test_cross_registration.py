from __future__ import division
import sys
import lfd
from lfd import registration
import h5py
import numpy as np
import os.path as osp
from image_proc.clouds import voxel_downsample, pixel_downsample
import rospy, itertools, glob
from utils.colorize import colorize
import h5py
import matplotlib.pyplot as plt
from matplotlib.delaunay import Triangulation
import networkx as nx


h5file = h5py.File(osp.join(osp.dirname(lfd.__file__), "data/knot_segments.h5"),"r")
seg_names = h5file.keys()

def calc_geodesic_distances(xyz):
    x,y = xyz[:,:2].T
    tri = Triangulation(x,y)
    G = nx.Graph()
    for (i0, i1) in tri.edge_db:
        dist = np.linalg.norm(xyz[i1] - xyz[i0])
        if dist < .03:
            G.add_edge(i0, i1, weight = np.linalg.norm(xyz[i1] - xyz[i0]))
    distmat = nx.floyd_warshall_numpy(G)
    
    finitevals = distmat[np.isfinite(distmat)]
    distmat[~np.isfinite(distmat)] = finitevals.max() * 3
    return distmat
    
results = []

dists = []
for i in xrange(6):
    rope = np.squeeze(h5file[seg_names[i]]["cloud_xyz"])
    dists.append(calc_geodesic_distances(rope))

for i0 in xrange(6):
    rope0 = np.squeeze(h5file[seg_names[i0]]["cloud_xyz"])
    n0 = len(rope0)
    pairs = zip(xrange(n0), np.random.randint(0,n0,n0))
    dists0 = dists[i0]

    for i1 in xrange(6):
        rope1 = np.squeeze(h5file[seg_names[i1]]["cloud_xyz"])

        dists1 = dists[i1]
        
        n1 = len(rope1)
        
        
        print colorize("comparing %s to %s"%(seg_names[i0], seg_names[i1]), "red")
        f = registration.tps_icp(rope0, rope1, plotting=False,reg_init=1,reg_final=.1,n_iter=21, verbose=False)
    
        corr = f.corr
        cost = 0
        for (pt0, pt1) in pairs:
            j0 = corr[pt0].argmax()
            j1 = corr[pt1].argmax()
            cost += abs(dists0[pt0, pt1] - dists1[j0, j1])
            
        cost /= len(pairs)
    
            
        results.append((i0, i1, cost))
        print i0, i1, cost
        
distmat = np.zeros((6,6))
for (i0, i1, cost) in results:
    distmat[i0, i1] = cost
distmat[xrange(6),xrange(6)] = np.inf
print distmat.argmin(axis=0)