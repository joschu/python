"""
compare point clouds
"""


from __future__ import division
import numpy as np
import networkx as nx
from matplotlib.delaunay import Triangulation
from lfd import registration
from scipy import sparse
import jds_utils.math_utils as mu



def make_sampling_matrix(inds_list, n_orig):
    """
    inds_list tells you which input inds correspond to each output ind
    "sampling matrix" is n_input_inds x n_output_inds such that
    out_{ij} = Indicator(input i -> output j)  * 1/n_input_inds
    i.e. it tells you how output is weighted avg of input
    """
    row_inds = []
    col_inds = []
    vals = []
    for (i_col, inds) in enumerate(inds_list):
        col_inds.extend([i_col]*len(inds))
        row_inds.extend(inds)
        vals.extend([1/len(inds)]*len(inds))
    return sparse.csc_matrix((vals,np.array([row_inds, col_inds])), shape=(n_orig,len(inds_list)))
            
def calc_geodesic_distances_downsampled_old(xyz, xyz_ds, ds_inds):
    """
    calculate geodesic distances between point xyz_ds using xyz to make graph
    """
    assert xyz.shape[1] == 3
    assert xyz_ds.shape[1] == 3
    D = calc_geodesic_distances(xyz)
    
    S = make_sampling_matrix(ds_inds, len(xyz))
    print S.shape
    return S.transpose().dot(S.transpose().dot(D).transpose()).T


def calc_geodesic_distances_downsampled(xyz, xyz_ds, ds_inds):
    """
    calculate geodesic distances between point xyz_ds using xyz to make graph
    """
    assert xyz.shape[1] == 3
    assert xyz_ds.shape[1] == 3
    D = calc_geodesic_distances(np.concatenate([xyz_ds,xyz],0))
    
    return D[:len(xyz_ds), :len(xyz_ds)]

    
def calc_geodesic_distances(xyz, res=.03):
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
        if dist < res:
            G.add_edge(i0, i1, weight = np.linalg.norm(xyz[i1] - xyz[i0]))
    distmat = np.asarray(nx.floyd_warshall_numpy(G))
    
    finitevals = distmat[np.isfinite(distmat)]
    distmat[~np.isfinite(distmat)] = finitevals.max() * 3
    return distmat


def calc_match_score(xyz0, xyz1, dists0 = None, dists1 = None, plotting = False):
    """
    calculate similarity between xyz0 and xyz1 using geodesic distances
    """
    
    f,info = registration.tps_rpm(xyz0, xyz1, plotting=plotting,reg_init=1,reg_final=.1,n_iter=21, verbose=False, return_full=True)
    partners = info["corr_nm"].argmax(axis=1)
    starts, ends = np.meshgrid(partners, partners)

    reses = [.05, .07, .09]
    nres = len(reses)

    targ_dist_mats, src_dist_mats = [],[]
    

    for i in xrange(nres):
    
        dists0 = calc_geodesic_distances(xyz0, reses[i])
        dists1 = calc_geodesic_distances(xyz1, reses[i])


        dists_targ = dists1[starts, ends]
        
        dists0_normed = dists0 / np.median(dists0)
        dists_targ_normed = dists_targ / np.median(dists1)
        
        src_dist_mats.append(dists0_normed)
        targ_dist_mats.append(dists_targ_normed)
    distmat = np.empty((nres, nres))
    for i in xrange(nres):
        for j in xrange(nres):
            distmat[i,j] = np.abs(src_dist_mats[i] - targ_dist_mats[j]).mean()
            

    print "dist at res:", distmat, distmat.min()
    return distmat.min()

