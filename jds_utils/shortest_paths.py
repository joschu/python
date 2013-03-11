import numpy as np

def shortest_paths(ncost_nk,ecost_nkk):
    """       
    Find minimum cost paths through graph (one path for each end point)
    where all nodes in nth row are connected to all nodes in (n+1)st row
    ncost_nk: node costs
    ecost_nkk: edge costs
    
    returns: (paths, costs) where
    paths is a K x N array of integers. Each row gives the minimum-cost path
      to its final node.
    costs has length K and gives the cost of each path
    """
    N = len(ncost_nk)
    assert len(ecost_nkk) == N-1
    
    cost_nk = [None for _ in xrange(N)]
    prev_nk = [None for _ in xrange(N-1)]
    cost_nk[0] = ncost_nk[0]
    for n in xrange(1,N):
        cost_kk = ecost_nkk[n-1] + cost_nk[n-1][:,None] + ncost_nk[n][None,:]
        cost_nk[n] = cost_kk.min(axis=0)
        prev_nk[n-1] = cost_kk.argmin(axis=0)

    
    path_costs = cost_nk[N-1]
    paths = [None for _ in xrange(N)]

    paths[N-1] = np.arange(len(ncost_nk[-1]))
    for n in xrange(N-1,0,-1):
        paths[n-1] = prev_nk[n-1][paths[n]]

    return np.array(paths).T, path_costs