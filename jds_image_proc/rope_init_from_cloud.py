import networkx as nx
import scipy.spatial.distance as ssd
from jds_image_proc import curves

MAX_DIST = .03
N_CTRL_PTS = 100

def initialize_rope(xyzs, plotting=False):
    pdists = ssd.squareform(ssd.pdist(xyzs))
    for (i_from, row) in enumerate(pdists):
        to_inds = np.flatnonzero(row[:i_from] < MAX_DIST)
        for i_to in to_inds:
            G.add_edge(i_from, i_to, weight = pdists[i_from, i_to])

    A = nx.floyd_warshall_numpy(G)
    A[np.isinf(A)] = 0
    (i_from_long, i_to_long) = np.unravel_index(A.argmax(), A.shape)
    path = nx.shortest_path(G, source=i_from_long, target=i_to_long)
    xyz_path = xyz[path,:]
    xyzs_unif = curves.unif_resample(total_path,N_CTRL_PTS,tol=.002)
    labels = np.ones(len(xyzs_unif),'int')
    labels[[1,-1]] = 2
    return xyzs_unif, labels
    
