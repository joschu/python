import numpy as np
import scipy.spatial.distance as ssd



def nan2zero(x):
    np.putmask(x, ~np.isfinite(x), 0)
    return x
N = 20
x = np.random.randn(N,3)


K = np.empty((4*N, 4*N))

dists_nn = ssd.cdist(x,x)
diffs_nna = np.empty((N,N,3))
for a in xrange(3):
    diffs_nna[:,:,a] = x[:,a][None,:] - x[:,a][:,None]

K[:N,:N] = dists_nn

K[:N, N:2*N] = diffs_nna[:,:,0]
K[:N, 2*N:3*N] = diffs_nna[:,:,1]
K[:N, 3*N:4*N] = diffs_nna[:,:,2]

K[N:2*N, :N] = -diffs_nna[:,:,0]
K[2*N:3*N, :N] = -diffs_nna[:,:,1]
K[3*N:4*N, :N] = -diffs_nna[:,:,2]

for a in xrange(3):
    for g in xrange(3):
        K[(g+1)*N:(g+2)*N, (a+1)*N:(a+2)*N] = - nan2zero(diffs_nna[:,:,a] * diffs_nna[:,:,g] / dists_nn)**3
for a in xrange(3):
    K[(a+1)*N:(a+2)*N, (a+1)*N:(a+2)*N] += nan2zero(1/dists_nn)
    
print np.linalg.eigvalsh(K)
    



#u,s,vh = np.linalg.svd(x1)
#nullspace = u[:,4:]
#nullproj = np.dot(nullspace, nullspace.T)

#K = -ssd.cdist(x,x)

#print np.dot(nullproj, x1)
#print np.linalg.eigvalsh(np.dot(nullproj, np.dot(K,nullproj)))


           