"""
compare point clouds
"""


from __future__ import division
import numpy as np
import networkx as nx
from matplotlib.delaunay import Triangulation
from lfd import registration
from scipy import sparse



def make_sampling_matrix(inds_list, n_orig):
    row_inds = []
    col_inds = []
    vals = []
    for (i_col, inds) in enumerate(inds_list):
        col_inds.extend([i_col]*len(inds))
        row_inds.extend(inds)
        vals.extend([1/len(inds)]*len(inds))
    return sparse.csc_matrix((vals,np.array([row_inds, col_inds])), shape=(n_orig,len(inds_list)))
            
def calc_geodesic_distances_downsampled_old(xyz, xyz_ds, ds_inds):
    assert xyz.shape[1] == 3
    assert xyz_ds.shape[1] == 3
    D = calc_geodesic_distances(xyz)
    
    S = make_sampling_matrix(ds_inds, len(xyz))
    print S.shape
    return S.transpose().dot(S.transpose().dot(D).transpose()).T


def calc_geodesic_distances_downsampled(xyz, xyz_ds, ds_inds):
    assert xyz.shape[1] == 3
    assert xyz_ds.shape[1] == 3
    D = calc_geodesic_distances(np.concatenate([xyz_ds,xyz],0))
    
    return D[:len(xyz_ds), :len(xyz_ds)]

    
def calc_geodesic_distances(xyz):
    """
    Calculates pairwise geodesic distances.
    Note that we generate the graph by projecting to 2D
    """
    x,y = xyz[:,:2].T
    tri = Triangulation(x,y)
    G = nx.Graph()
    #G.add_nodes_from(xrange(len(xyz)))
    for i0 in xrange(len(xyz)):
        G.add_node(i0)
    for (i0, i1) in tri.edge_db:
        dist = np.linalg.norm(xyz[i1] - xyz[i0])
        if dist < .03:
            G.add_edge(i0, i1, weight = np.linalg.norm(xyz[i1] - xyz[i0]))
    distmat = np.asarray(nx.floyd_warshall_numpy(G))
    
    finitevals = distmat[np.isfinite(distmat)]
    distmat[~np.isfinite(distmat)] = finitevals.max() * 3
    return distmat


def calc_match_score(xyz0, xyz1, dists0 = None, dists1 = None):
    f = registration.tps_rpm(xyz0, xyz1, plotting=False,reg_init=1,reg_final=.1,n_iter=21, verbose=False)
    corr = f.corr
    partners = corr.argmax(axis=1)
    
    if dists0 is None: dists0 = calc_geodesic_distances(xyz0)
    if dists1 is None: dists1 = calc_geodesic_distances(xyz1)

    starts, ends = np.meshgrid(partners, partners)

    dists_targ = dists1[starts, ends]
    
    return np.abs(dists0 - dists_targ).sum()/dists0.size