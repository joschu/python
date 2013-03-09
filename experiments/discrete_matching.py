import scipy.spatial.distance as ssd
from trajoptpy.kin_utils import shortest_paths
def match(xfeat_nk, xpos_nd, yfeat_mk, ypos_md, feature_metric = "euclidean", feature_coeff = 1, direction_metric = "cosine"):
    """
    M > N
    return list of length N
    """
    N,K = x_nk.shape
    D = xpos_nk.shape[1]
    M = yfeat_mk.shape[0]
    
    assert xfeat_nk.shape == (N,K)
    assert xpos_nk.shape == (N,D)
    assert yfeat_mk.shape == (M,K)
    assert ypos_md.shape == (M,D)
    
    nodecosts_nm = ssd.cdist(xfeat_nk, yfeat_mk, metric = feature_metric) * feature_coeff
    edgecosts_nmm = np.empty((N-1,M,M))
    for n in xrange(N-1):
        dir_d = xpos_nd[n+1] - xpos_nd[n]
        dir_mmd = ypos_md[:,None,:] - ypos_md[None,:,:]
        dir_mm_d = dir_mmd.reshape(M*M,D)
        dists_mm_1 = ssd.cdist(dir_mm_d, dir_d, metric = direction_metric)
        edgecosts_nmm[n] = dists_mm_1[:,0].reshape(M,M)
        
    paths_mn, costs_m = shortest_paths(nodecosts_nm, edgecosts_nmm)
    best_path = paths_mn[np.argmin(costs_m)]
    return best_path
    
    
    