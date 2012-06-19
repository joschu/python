from __future__ import division
from lfd import trajectory_library
import lfd
import matplotlib.pyplot as plt
import numpy as np
import sys
from image_proc.clouds import voxel_downsample
from rope_vision import rope_initialization as ri
from kinematics.retiming import shortest_path
from utils.math_utils import norms

library = trajectory_library.TrajectoryLibrary("lm.h5","read")

plt.ion()
segment_group = library.root["segments"]

n_segs = len(segment_group.keys())
n_rows = int(round(np.sqrt(n_segs)))
n_cols = int(np.ceil(n_segs / n_rows))    
print n_rows, n_cols

plt.figure(1); plt.clf()
sim_ropes = []
for (i,(name, data)) in enumerate(segment_group.items()):
    plt.subplot(n_rows, n_cols, i+1)
    if "object_points" in segment_group:
        points_n3 = data["object_points"][0]
    else:
        print "rope field is deprecated. use object points"
        points_n3 = data["rope"][0]
    plt.plot(points_n3[:,1], points_n3[:,0],'.')
    plt.title(name)

    sim_ropes.append(points_n3)
    
i=int(sys.argv[1])
rope = np.dot(np.squeeze(np.loadtxt("/home/joschu/Data/rope/rope%i.txt"%i)),np.diag([1,-1,-1]))    
rope = voxel_downsample(rope,.02)
sim_rope = sim_ropes[i][::1]

#G = ri.points_to_graph(rope, .03)
#n = len(sim_rope)-1
#k = len(G)
#ecosts_nkk = np.empty((n,k,k))

#ropediff_kkd = rope[:,None,:] - rope[None,:,:]
#simdiff_nd = sim_rope[1:,:] - sim_rope[:-1,:]
#ecosts_nkk = (simdiff_nd[:,None,None,:]*ropediff_kkd[None,:,:,:]).sum(axis=3) / \
    #norms(simdiff_nd,1)[:,None,None] * norms(ropediff_kkd,2)[None,:,:]\
    #+ norms(simdiff_nd[:,None,None,:]-ropediff_kkd[None,:,:,:],3)*1
#ncosts_nk = np.zeros((n,k))
#path, cost =  shortest_path(ncosts_nk, ecosts_nkk)


import matplotlib.pyplot as plt
newrope = np.array([rope[i] for i in path])
plt.figure(2); plt.clf()
plt.plot(newrope[:,0], newrope[:,1],'g')
plt.plot(sim_rope[:,0], sim_rope[:,1],'r')
plt.plot(rope[:,0], rope[:,1], 'b.')

