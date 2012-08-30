# make graph of reasonably nearby points
# make a spanning tree

from jds_image_proc import pcd_io
import scipy.spatial.distance as ssd, numpy as np
import networkx as nx

MAX_DIST = .03

(xyz,) , rgb = pcd_io.load_xyzrgb("/tmp/comm/rope_pts/data000000000093.pcd")
pdists = ssd.squareform(ssd.pdist(xyz))

G = nx.Graph()
for (i_from, row) in enumerate(pdists):
    to_inds = np.flatnonzero(row[:i_from] < MAX_DIST)
    for i_to in to_inds:
        G.add_edge(i_from, i_to, weight = pdists[i_from, i_to])

A = nx.floyd_warshall_numpy(G)
A[np.isinf(A)] = 0
(i_from_long, i_to_long) = np.unravel_index(A.argmax(), A.shape)
path = nx.shortest_path(G, source=i_from_long, target=i_to_long)

xyz_path = xyz[path,:]
import enthought.mayavi.mlab as mlab
mlab.clf()
mlab.plot3d(*xyz_path.T,tube_radius=.02)
mlab.points3d(*xyz.T, scale_factor=.025, color=(1,0,0))
